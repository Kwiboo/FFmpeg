/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "config.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "libavutil/hwcontext_v4l2request.h"
#include "libavutil/mem.h"
#include "decode.h"
#include "v4l2_request_internal.h"

static const AVClass v4l2_request_context_class = {
    .class_name = "V4L2RequestContext",
    .item_name  = av_default_item_name,
    .version    = LIBAVUTIL_VERSION_INT,
};

int ff_v4l2_request_query_control(AVCodecContext *avctx,
                                  struct v4l2_query_ext_ctrl *control)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);

    if (ioctl(ctx->video_fd, VIDIOC_QUERY_EXT_CTRL, control) < 0) {
        // Skip error logging when driver does not support control id (EINVAL)
        if (errno != EINVAL) {
            av_log(ctx, AV_LOG_ERROR, "Failed to query control %u: %s (%d)\n",
                   control->id, strerror(errno), errno);
        }
        return AVERROR(errno);
    }

    return 0;
}

int ff_v4l2_request_query_control_default_value(AVCodecContext *avctx,
                                                uint32_t id)
{
    struct v4l2_query_ext_ctrl control = {
        .id = id,
    };
    int ret;

    ret = ff_v4l2_request_query_control(avctx, &control);
    if (ret < 0)
        return ret;

    return control.default_value;
}

int ff_v4l2_request_set_request_controls(V4L2RequestContext *ctx, int request_fd,
                                         struct v4l2_ext_control *control, int count)
{
    struct v4l2_ext_controls controls = {
        .controls = control,
        .count = count,
        .request_fd = request_fd,
        .which = (request_fd >= 0) ? V4L2_CTRL_WHICH_REQUEST_VAL : 0,
    };

    if (!control || !count)
        return 0;

    if (ioctl(ctx->video_fd, VIDIOC_S_EXT_CTRLS, &controls) < 0)
        return AVERROR(errno);

    return 0;
}

int ff_v4l2_request_set_controls(AVCodecContext *avctx,
                                 struct v4l2_ext_control *control, int count)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    int ret;

    ret = ff_v4l2_request_set_request_controls(ctx, -1, control, count);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to set %d control(s): %s (%d)\n",
               count, strerror(errno), errno);
    }

    return ret;
}

static int v4l2_request_buffer_alloc(V4L2RequestContext *ctx,
                                     V4L2RequestBuffer *buf,
                                     enum v4l2_buf_type type)
{
    struct v4l2_plane planes[1] = {};
    struct v4l2_create_buffers buffers = {
        .count = 1,
        .memory = V4L2_MEMORY_MMAP,
        .format.type = type,
    };

    // Get format details for the buffer to be created
    if (ioctl(ctx->video_fd, VIDIOC_G_FMT, &buffers.format) < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to get format of type %d: %s (%d)\n",
               type, strerror(errno), errno);
        return AVERROR(errno);
    }

    // Create the buffer
    if (ioctl(ctx->video_fd, VIDIOC_CREATE_BUFS, &buffers) < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to create buffer of type %d: %s (%d)\n",
               type, strerror(errno), errno);
        return AVERROR(errno);
    }

    if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
        buf->width = buffers.format.fmt.pix_mp.width;
        buf->height = buffers.format.fmt.pix_mp.height;
        buf->size = buffers.format.fmt.pix_mp.plane_fmt[0].sizeimage;
        buf->buffer.length = 1;
        buf->buffer.m.planes = planes;
    } else {
        buf->width = buffers.format.fmt.pix.width;
        buf->height = buffers.format.fmt.pix.height;
        buf->size = buffers.format.fmt.pix.sizeimage;
    }

    buf->index = buffers.index;
    buf->capabilities = buffers.capabilities;
    buf->used = 0;

    buf->buffer.type = type;
    buf->buffer.memory = V4L2_MEMORY_MMAP;
    buf->buffer.index = buf->index;

    // Query more details of the created buffer
    if (ioctl(ctx->video_fd, VIDIOC_QUERYBUF, &buf->buffer) < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to query buffer %d of type %d: %s (%d)\n",
               buf->index, type, strerror(errno), errno);
        return AVERROR(errno);
    }

    // Output buffers is mapped and capture buffers is exported
    if (V4L2_TYPE_IS_OUTPUT(type)) {
        off_t offset = V4L2_TYPE_IS_MULTIPLANAR(type) ?
                       buf->buffer.m.planes[0].m.mem_offset :
                       buf->buffer.m.offset;
        void *addr = mmap(NULL, buf->size, PROT_READ | PROT_WRITE, MAP_SHARED,
                          ctx->video_fd, offset);
        if (addr == MAP_FAILED) {
            av_log(ctx, AV_LOG_ERROR, "Failed to map output buffer %d: %s (%d)\n",
                   buf->index, strerror(errno), errno);
            return AVERROR(errno);
        }

        // Raw bitstream data is appended to output buffers
        buf->addr = (uint8_t *)addr;
    } else {
        struct v4l2_exportbuffer exportbuffer = {
            .type = type,
            .index = buf->index,
            .flags = O_RDONLY,
        };

        if (ioctl(ctx->video_fd, VIDIOC_EXPBUF, &exportbuffer) < 0) {
            av_log(ctx, AV_LOG_ERROR, "Failed to export capture buffer %d: %s (%d)\n",
                   buf->index, strerror(errno), errno);
            return AVERROR(errno);
        }

        // Used in the AVDRMFrameDescriptor for decoded frames
        buf->fd = exportbuffer.fd;

        /*
         * Use buffer index as base for V4L2 frame reference.
         * This works because a capture buffer is closely tied to a AVFrame
         * and FFmpeg handle all frame reference tracking for us.
         */
        buf->buffer.timestamp.tv_usec = buf->index + 1;
    }

    return 0;
}

static void v4l2_request_buffer_free(V4L2RequestBuffer *buf)
{
    // Unmap output buffers
    if (buf->addr) {
        munmap(buf->addr, buf->size);
        buf->addr = NULL;
    }

    // Close exported capture buffers or requests for output buffers
    if (buf->fd >= 0) {
        close(buf->fd);
        buf->fd = -1;
    }
}

static void v4l2_request_frame_free(void *opaque, uint8_t *data)
{
    V4L2RequestFrameDescriptor *desc = (V4L2RequestFrameDescriptor *)data;

    v4l2_request_buffer_free(&desc->capture);

    av_free(data);
}

static AVBufferRef *v4l2_request_frame_alloc(void *opaque, size_t size)
{
    V4L2RequestContext *ctx = opaque;
    V4L2RequestFrameDescriptor *desc;
    AVBufferRef *ref;
    uint8_t *data;

    data = av_mallocz(size);
    if (!data)
        return NULL;

    ref = av_buffer_create(data, size, v4l2_request_frame_free, ctx, 0);
    if (!ref) {
        av_free(data);
        return NULL;
    }

    desc = (V4L2RequestFrameDescriptor *)data;
    desc->capture.fd = -1;

    // Create a V4L2 capture buffer for this AVFrame
    if (v4l2_request_buffer_alloc(ctx, &desc->capture, ctx->format.type) < 0) {
        av_buffer_unref(&ref);
        return NULL;
    }

    // Set AVDRMFrameDescriptor of this AVFrame
    if (ff_v4l2_request_set_drm_descriptor(desc, &ctx->format) < 0) {
        av_buffer_unref(&ref);
        return NULL;
    }

    return ref;
}

static void v4l2_request_hwframe_ctx_free(AVHWFramesContext *hwfc)
{
    av_buffer_pool_uninit(&hwfc->pool);
}

int ff_v4l2_request_frame_params(AVCodecContext *avctx,
                                 AVBufferRef *hw_frames_ctx)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    AVHWFramesContext *hwfc = (AVHWFramesContext *)hw_frames_ctx->data;

    hwfc->format = AV_PIX_FMT_DRM_PRIME;
    hwfc->sw_format = ff_v4l2_request_get_sw_format(&ctx->format);

    if (V4L2_TYPE_IS_MULTIPLANAR(ctx->format.type)) {
        hwfc->width = ctx->format.fmt.pix_mp.width;
        hwfc->height = ctx->format.fmt.pix_mp.height;
    } else {
        hwfc->width = ctx->format.fmt.pix.width;
        hwfc->height = ctx->format.fmt.pix.height;
    }

    hwfc->pool = av_buffer_pool_init2(sizeof(V4L2RequestFrameDescriptor), ctx,
                                      v4l2_request_frame_alloc, NULL);
    if (!hwfc->pool)
        return AVERROR(ENOMEM);

    hwfc->free = v4l2_request_hwframe_ctx_free;

    hwfc->initial_pool_size = 1;

    switch (avctx->codec_id) {
    case AV_CODEC_ID_VP9:
        hwfc->initial_pool_size += 8;
        break;
    case AV_CODEC_ID_VP8:
        hwfc->initial_pool_size += 3;
        break;
    default:
        hwfc->initial_pool_size += 2;
    }

    return 0;
}

int ff_v4l2_request_uninit(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);

    if (ctx->video_fd >= 0) {
        // Flush and wait on all pending requests
        ff_v4l2_request_flush(avctx);

        // Stop output queue
        if (ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &ctx->output_type) < 0) {
            av_log(ctx, AV_LOG_WARNING, "Failed to stop output streaming: %s (%d)\n",
                   strerror(errno), errno);
        }

        // Stop capture queue
        if (ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &ctx->format.type) < 0) {
            av_log(ctx, AV_LOG_WARNING, "Failed to stop capture streaming: %s (%d)\n",
                   strerror(errno), errno);
        }

        // Release output buffers
        for (int i = 0; i < FF_ARRAY_ELEMS(ctx->output); i++) {
            v4l2_request_buffer_free(&ctx->output[i]);
        }

        // Close video device file descriptor
        close(ctx->video_fd);
        ctx->video_fd = -1;
    }

    // Ownership of media device file descriptor may belong to hwdevice
    if (ctx->device_ref) {
        av_buffer_unref(&ctx->device_ref);
        ctx->media_fd = -1;
    } else if (ctx->media_fd >= 0) {
        close(ctx->media_fd);
        ctx->media_fd = -1;
    }

    ff_mutex_destroy(&ctx->mutex);

    return 0;
}

static int v4l2_request_init_context(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    int ret;

    // Initialize context state
    ff_mutex_init(&ctx->mutex, NULL);
    for (int i = 0; i < FF_ARRAY_ELEMS(ctx->output); i++) {
        ctx->output[i].index = i;
        ctx->output[i].fd = -1;
    }
    atomic_init(&ctx->next_output, 0);
    atomic_init(&ctx->queued_output, 0);
    atomic_init(&ctx->queued_request, 0);
    atomic_init(&ctx->queued_capture, 0);

    // Get format details for capture buffers
    if (ioctl(ctx->video_fd, VIDIOC_G_FMT, &ctx->format) < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to get capture format: %s (%d)\n",
               strerror(errno), errno);
        ret = AVERROR(errno);
        goto fail;
    }

    // Create frame context and allocate initial capture buffers
    ret = ff_decode_get_hw_frames_ctx(avctx, AV_HWDEVICE_TYPE_V4L2REQUEST);
    if (ret < 0)
        goto fail;

    // Allocate output buffers for circular queue
    for (int i = 0; i < FF_ARRAY_ELEMS(ctx->output); i++) {
        ret = v4l2_request_buffer_alloc(ctx, &ctx->output[i], ctx->output_type);
        if (ret < 0)
            goto fail;
    }

    // Allocate requests for circular queue
    for (int i = 0; i < FF_ARRAY_ELEMS(ctx->output); i++) {
        if (ioctl(ctx->media_fd, MEDIA_IOC_REQUEST_ALLOC, &ctx->output[i].fd) < 0) {
            av_log(ctx, AV_LOG_ERROR, "Failed to allocate request %d: %s (%d)\n",
                   i, strerror(errno), errno);
            ret = AVERROR(errno);
            goto fail;
        }
    }

    // Start output queue
    if (ioctl(ctx->video_fd, VIDIOC_STREAMON, &ctx->output_type) < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to start output streaming: %s (%d)\n",
               strerror(errno), errno);
        ret = AVERROR(errno);
        goto fail;
    }

    // Start capture queue
    if (ioctl(ctx->video_fd, VIDIOC_STREAMON, &ctx->format.type) < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to start capture streaming: %s (%d)\n",
               strerror(errno), errno);
        ret = AVERROR(errno);
        goto fail;
    }

    return 0;

fail:
    ff_v4l2_request_uninit(avctx);
    return ret;
}

int ff_v4l2_request_init(AVCodecContext *avctx,
                         uint32_t pixelformat, uint32_t buffersize,
                         struct v4l2_ext_control *control, int count)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    AVV4L2RequestDeviceContext *hwctx = NULL;
    int ret;

    // Set initial default values
    ctx->av_class = &v4l2_request_context_class;
    ctx->media_fd = -1;
    ctx->video_fd = -1;

    // Try to use media device file descriptor opened and owned by hwdevice
    if (avctx->hw_device_ctx) {
        AVHWDeviceContext *device_ctx = (AVHWDeviceContext *)avctx->hw_device_ctx->data;
        if (device_ctx->type == AV_HWDEVICE_TYPE_V4L2REQUEST && device_ctx->hwctx) {
            hwctx = device_ctx->hwctx;
            ctx->media_fd = hwctx->media_fd;
        }
    }

    // Probe for a capable media and video device for the V4L2 codec pixelformat
    ret = ff_v4l2_request_probe(avctx, pixelformat, buffersize, control, count);
    if (ret < 0) {
        av_log(avctx, AV_LOG_INFO, "No V4L2 video device found for %s\n",
               av_fourcc2str(pixelformat));
        return ret;
    }

    // Transfer (or return) ownership of media device file descriptor to hwdevice
    if (hwctx) {
        hwctx->media_fd = ctx->media_fd;
        ctx->device_ref = av_buffer_ref(avctx->hw_device_ctx);
    }

    // Create buffers and finalize init
    return v4l2_request_init_context(avctx);
}
