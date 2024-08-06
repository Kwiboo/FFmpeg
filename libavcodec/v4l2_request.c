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

#include <linux/media.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "libavutil/hwcontext_v4l2request_internal.h"
#include "libavutil/mem.h"
#include "decode.h"
#include "internal.h"
#include "v4l2_request.h"

static const AVClass v4l2_request_context_class = {
    .class_name = "V4L2RequestContext",
    .item_name  = av_default_item_name,
    .version    = LIBAVUTIL_VERSION_INT,
};

static inline V4L2RequestContext *v4l2_request_context(AVCodecContext *avctx)
{
    return (V4L2RequestContext *)avctx->internal->hwaccel_priv_data;
}

static void v4l2_request_output_buffer_uninit(V4L2RequestOutputBuffer *output)
{
    // Close the request associated with the OUTPUT buffer
    if (output->request_fd >= 0) {
        close(output->request_fd);
        output->request_fd = -1;
    }

    // Umap the OUTPUT buffer memory
    if (output->addr) {
        munmap(output->addr, output->size);
        output->addr = NULL;
    }

    // Return the OUTPUT buffer to the frames context OUTPUT pool
    av_buffer_unref(&output->ref);
}

static int v4l2_request_output_buffer_init(V4L2RequestContext *ctx,
                                           V4L2RequestOutputBuffer *output)
{
    struct v4l2_format *format = &ctx->fctxi->output.format;
    struct v4l2_buffer *buffer;
    off_t offset;
    void *addr;
    int ret;

    // Get an OUTPUT buffer from frames context OUTPUT pool
    output->ref = av_buffer_pool_get(ctx->fctxi->output.pool);
    if (!output->ref)
        return AVERROR(ENOMEM);

    buffer = (struct v4l2_buffer *)output->ref->data;
    output->index = buffer->index;
    output->size = V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                   format->fmt.pix_mp.plane_fmt[0].sizeimage :
                   format->fmt.pix.sizeimage;
    output->bytesused = 0;

    // Map the OUTPUT buffer memory, raw bitstream data is written into it
    offset = V4L2_TYPE_IS_MULTIPLANAR(buffer->type) ?
             buffer->m.planes[0].m.mem_offset :
             buffer->m.offset;
    addr = mmap(NULL, output->size, PROT_READ | PROT_WRITE, MAP_SHARED,
                ctx->fctxi->video_fd, offset);
    if (addr == MAP_FAILED) {
        ret = AVERROR(errno);
        av_log(ctx, AV_LOG_ERROR, "Failed to map OUTPUT buffer %d: %s (%d)\n",
               output->index, strerror(errno), errno);
        goto fail;
    }
    output->addr = addr;

    // Allocate and associated a request for the OUTPUT buffer
    if (ioctl(ctx->fctxi->media_fd, MEDIA_IOC_REQUEST_ALLOC, &output->request_fd) < 0) {
        ret = AVERROR(errno);
        av_log(ctx, AV_LOG_ERROR, "Failed to allocate request for OUTPUT buffer %d: %s (%d)\n",
               output->index, strerror(errno), errno);
        goto fail;
    }

    return 0;

fail:
    v4l2_request_output_buffer_uninit(output);
    return ret;
}

int ff_v4l2_request_frame_params(AVCodecContext *avctx,
                                 AVBufferRef *hw_frames_ctx,
                                 uint32_t pixelformat,
                                 uint8_t bit_depth)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    AVHWFramesContext *hwfc = (AVHWFramesContext *)hw_frames_ctx->data;
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;

    // Set parameters used during frames context initialization
    fctx->pixelformat = pixelformat;
    fctx->bit_depth = bit_depth;
    if (ctx) {
        fctx->init_controls = ctx->init_controls;
        fctx->nb_init_controls = ctx->nb_init_controls;
    }

    hwfc->format = AV_PIX_FMT_DRM_PRIME;
    hwfc->sw_format = AV_PIX_FMT_NONE;
    hwfc->width = avctx->coded_width;
    hwfc->height = avctx->coded_height;

    // Pre-allocate CAPTURE buffers to ensure CAPTURE queue can be started
    hwfc->initial_pool_size = 1;

    return 0;
}

int ff_v4l2_request_uninit(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    enum v4l2_buf_type type;

    if (ctx->fctxi) {
        // TODO: Flush and wait on all pending requests

        // Stop streaming on OUTPUT queue
        type = ctx->fctxi->output.format.type;
        if (ioctl(ctx->fctxi->video_fd, VIDIOC_STREAMOFF, &type) < 0)
            av_log(ctx, AV_LOG_WARNING, "Failed to stop OUTPUT streaming: %s (%d)\n",
                   strerror(errno), errno);

        // Stop streaming on CAPTURE queue
        type = ctx->fctxi->capture.format.type;
        if (ioctl(ctx->fctxi->video_fd, VIDIOC_STREAMOFF, &type) < 0)
            av_log(ctx, AV_LOG_WARNING, "Failed to stop CAPTURE streaming: %s (%d)\n",
                   strerror(errno), errno);

        // Release OUTPUT buffers and requests
        for (int i = 0; i < FF_ARRAY_ELEMS(ctx->output); i++)
            v4l2_request_output_buffer_uninit(&ctx->output[i]);

        ctx->fctxi = NULL;
    }

    av_buffer_unref(&ctx->frames_ref);
    ff_mutex_destroy(&ctx->mutex);

    return 0;
}

int ff_v4l2_request_init(AVCodecContext *avctx,
                         struct v4l2_ext_control *control, int count,
                         int (*post_frames_ctx)(AVCodecContext *avctx))
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    AVHWFramesContext *hwfc;
    AVV4L2RequestFramesContext *fctx;
    enum v4l2_buf_type type;
    int ret;

    // Set initial default values
    ctx->av_class = &v4l2_request_context_class;
    ctx->init_controls = control;
    ctx->nb_init_controls = count;
    ff_mutex_init(&ctx->mutex, NULL);
    for (int i = 0; i < FF_ARRAY_ELEMS(ctx->output); i++) {
        ctx->output[i].index = i;
        ctx->output[i].request_fd = -1;
    }

    // Create frames context and allocate initial CAPTURE buffers
    ret = ff_decode_get_hw_frames_ctx(avctx, AV_HWDEVICE_TYPE_V4L2REQUEST);
    if (ret < 0)
        goto fail;

    ctx->frames_ref = av_buffer_ref(avctx->hw_frames_ctx);
    if (!ctx->frames_ref) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    // Get internal hwctx from frames context
    hwfc = (AVHWFramesContext *)ctx->frames_ref->data;
    fctx = hwfc->hwctx;
    ctx->fctxi = fctx->internal;

    // Reset init controls after successful frames context initialization
    ctx->init_controls = NULL;
    ctx->nb_init_controls = 0;

    // Check codec-specific controls, e.g. profile and level
    if (post_frames_ctx) {
        ret = post_frames_ctx(avctx);
        if (ret < 0)
            goto fail;
    }

    // Allocate OUTPUT buffers and requests for circular queue
    for (int i = 0; i < FF_ARRAY_ELEMS(ctx->output); i++) {
        ret = v4l2_request_output_buffer_init(ctx, &ctx->output[i]);
        if (ret < 0)
            goto fail;
    }

    // Start streaming on OUTPUT queue
    type = ctx->fctxi->output.format.type;
    if (ioctl(ctx->fctxi->video_fd, VIDIOC_STREAMON, &type) < 0) {
        ret = AVERROR(errno);
        av_log(ctx, AV_LOG_ERROR, "Failed to start OUTPUT streaming: %s (%d)\n",
               strerror(errno), errno);
        goto fail;
    }

    // Start streaming on CAPTURE queue
    type = ctx->fctxi->capture.format.type;
    if (ioctl(ctx->fctxi->video_fd, VIDIOC_STREAMON, &type) < 0) {
        ret = AVERROR(errno);
        av_log(ctx, AV_LOG_ERROR, "Failed to start CAPTURE streaming: %s (%d)\n",
               strerror(errno), errno);
        goto fail;
    }

    return 0;

fail:
    ff_v4l2_request_uninit(avctx);
    return ret;
}
