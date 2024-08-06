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
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "libavutil/hwcontext_v4l2request_internal.h"
#include "libavutil/mem.h"
#include "decode.h"
#include "internal.h"
#include "v4l2_request.h"

#define V4L2_PLANES_MAX 2

static const AVClass v4l2_request_context_class = {
    .class_name = "V4L2RequestContext",
    .item_name  = av_default_item_name,
    .version    = LIBAVUTIL_VERSION_INT,
};

static inline V4L2RequestContext *v4l2_request_context(AVCodecContext *avctx)
{
    return (V4L2RequestContext *)avctx->internal->hwaccel_priv_data;
}

static inline uint32_t v4l2_request_frameindex(AVFrame *frame)
{
    return (uint32_t)(uintptr_t)frame->data[1];
}

uint64_t ff_v4l2_request_get_capture_timestamp(AVFrame *frame)
{
    /*
     * The CAPTURE buffer index is used as a base for V4L2 frame reference.
     * This works because frames are decoded into a CAPTURE buffer that is
     * closely tied to an AVFrame.
     */
    struct timeval timestamp = {
        .tv_sec = 0,
        .tv_usec = v4l2_request_frameindex(frame) + 1,
    };
    return v4l2_timeval_to_ns(&timestamp);
}

int ff_v4l2_request_query_control(AVCodecContext *avctx,
                                  struct v4l2_query_ext_ctrl *control)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);

    if (ioctl(ctx->fctxi->video_fd, VIDIOC_QUERY_EXT_CTRL, control) < 0) {
        int ret = AVERROR(errno);
        // Skip error logging when driver does not support control id (EINVAL)
        if (errno != EINVAL)
            av_log(ctx, AV_LOG_ERROR, "Failed to query control %u: %s (%d)\n",
                   control->id, strerror(errno), errno);
        return ret;
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

static int v4l2_request_set_controls(V4L2RequestContext *ctx, int request_fd,
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

    if (ioctl(ctx->fctxi->video_fd, VIDIOC_S_EXT_CTRLS, &controls) < 0)
        return AVERROR(errno);

    return 0;
}

int ff_v4l2_request_set_controls(AVCodecContext *avctx,
                                 struct v4l2_ext_control *control, int count)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    int ret;

    ret = v4l2_request_set_controls(ctx, -1, control, count);
    if (ret < 0)
        av_log(ctx, AV_LOG_ERROR, "Failed to set %d control(s): %s (%d)\n",
               count, strerror(errno), errno);

    return ret;
}

static int v4l2_request_queue_buffer(V4L2RequestContext *ctx,
                                     struct v4l2_buffer *buffer)
{
    struct v4l2_plane planes[V4L2_PLANES_MAX] = {};

    if (V4L2_TYPE_IS_MULTIPLANAR(buffer->type)) {
        planes[0].bytesused = buffer->bytesused;
        buffer->bytesused = 0;
        buffer->length = FF_ARRAY_ELEMS(planes);
        buffer->m.planes = planes;
    }

    // Queue the buffer
    if (ioctl(ctx->fctxi->video_fd, VIDIOC_QBUF, buffer) < 0)
        return AVERROR(errno);

    // Mark the buffer as queued
    if (V4L2_TYPE_IS_OUTPUT(buffer->type))
        ctx->queued_output |= 1 << buffer->index;
    else
        ctx->queued_capture |= 1 << buffer->index;

    return 0;
}

static int v4l2_request_queue_capture_buffer(V4L2RequestContext *ctx,
                                             uint32_t index)
{
    struct v4l2_buffer buffer = {
        .index = index,
        .type = ctx->fctxi->capture.format.type,
        .memory = V4L2_MEMORY_MMAP,
    };
    return v4l2_request_queue_buffer(ctx, &buffer);
}

static int v4l2_request_queue_output_buffer(V4L2RequestContext *ctx,
                                            V4L2RequestOutputBuffer *output,
                                            uint32_t flags)
{
    struct v4l2_buffer buffer = {
        .index = output->index,
        .type = ctx->fctxi->output.format.type,
        .memory = V4L2_MEMORY_MMAP,
        .timestamp = output->timestamp,
        .bytesused = output->bytesused,
        .request_fd = output->request_fd,
        .flags = V4L2_BUF_FLAG_REQUEST_FD | flags,
    };
    return v4l2_request_queue_buffer(ctx, &buffer);
}

static int v4l2_request_dequeue_buffer(V4L2RequestContext *ctx,
                                       enum v4l2_buf_type type)
{
    struct v4l2_plane planes[V4L2_PLANES_MAX] = {};
    struct v4l2_buffer buffer = {
        .type = type,
        .memory = V4L2_MEMORY_MMAP,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(buffer.type)) {
        buffer.length = FF_ARRAY_ELEMS(planes);
        buffer.m.planes = planes;
    }

    // Dequeue next completed buffer
    if (ioctl(ctx->fctxi->video_fd, VIDIOC_DQBUF, &buffer) < 0)
        return AVERROR(errno);

    // Mark the buffer as dequeued
    if (V4L2_TYPE_IS_OUTPUT(buffer.type))
        ctx->queued_output &= ~(1 << buffer.index);
    else
        ctx->queued_capture &= ~(1 << buffer.index);

    return 0;
}

static inline int v4l2_request_dequeue_completed_buffers(V4L2RequestContext *ctx,
                                                         enum v4l2_buf_type type)
{
    int ret;

    do {
        ret = v4l2_request_dequeue_buffer(ctx, type);
    } while (!ret);

    return ret;
}

static int v4l2_request_wait_on_capture(V4L2RequestContext *ctx, uint32_t index)
{
    enum v4l2_buf_type type = ctx->fctxi->capture.format.type;
    struct pollfd pollfd = {
        .fd = ctx->fctxi->video_fd,
        .events = POLLIN,
    };

    ff_mutex_lock(&ctx->mutex);

    // Dequeue all completed CAPTURE buffers
    if (ctx->queued_capture)
        v4l2_request_dequeue_completed_buffers(ctx, type);

    // Wait on the specific CAPTURE buffer
    while (ctx->queued_capture & (1 << index)) {
        int ret = poll(&pollfd, 1, 2000);
        if (ret <= 0)
            goto fail;

        ret = v4l2_request_dequeue_buffer(ctx, type);
        if (ret < 0 && ret != AVERROR(EAGAIN))
            goto fail;
    }

    ff_mutex_unlock(&ctx->mutex);
    return 0;

fail:
    ff_mutex_unlock(&ctx->mutex);
    av_log(ctx, AV_LOG_ERROR, "Failed waiting on CAPTURE buffer %d\n", index);
    return AVERROR(EINVAL);
}

static V4L2RequestOutputBuffer *v4l2_request_next_output(V4L2RequestContext *ctx)
{
    enum v4l2_buf_type type = ctx->fctxi->output.format.type;
    V4L2RequestOutputBuffer *output;
    struct pollfd pollfd = {
        .fd = ctx->fctxi->video_fd,
        .events = POLLOUT,
    };
    uint8_t index;

    ff_mutex_lock(&ctx->mutex);

    // Use next OUTPUT buffer in the circular queue
    index = ctx->next_output;
    output = &ctx->output[index];
    ctx->next_output = (index + 1) % FF_ARRAY_ELEMS(ctx->output);

    // Dequeue all completed OUTPUT buffers
    if (ctx->queued_output)
        v4l2_request_dequeue_completed_buffers(ctx, type);

    // Wait on the specific OUTPUT buffer
    while (ctx->queued_output & (1 << output->index)) {
        int ret = poll(&pollfd, 1, 2000);
        if (ret <= 0)
            goto fail;

        ret = v4l2_request_dequeue_buffer(ctx, type);
        if (ret < 0 && ret != AVERROR(EAGAIN))
            goto fail;
    }

    ff_mutex_unlock(&ctx->mutex);

    // Reset bytesused state
    output->bytesused = 0;

    return output;

fail:
    ff_mutex_unlock(&ctx->mutex);
    av_log(ctx, AV_LOG_ERROR, "Failed waiting on OUTPUT buffer %d\n",
           output->index);
    return NULL;
}

static int v4l2_request_wait_on_request(V4L2RequestContext *ctx,
                                        V4L2RequestOutputBuffer *output)
{
    struct pollfd pollfd = {
        .fd = output->request_fd,
        .events = POLLPRI,
    };

    // Wait on the specific request to complete
    while (ctx->queued_request & (1 << output->index)) {
        int ret = poll(&pollfd, 1, 2000);
        if (ret <= 0)
            break;

        // Mark request as dequeued
        if (pollfd.revents & (POLLPRI | POLLERR)) {
            ctx->queued_request &= ~(1 << output->index);
            break;
        }
    }

    // Reinit the request object
    if (ioctl(output->request_fd, MEDIA_REQUEST_IOC_REINIT) < 0) {
        int ret = AVERROR(errno);
        av_log(ctx, AV_LOG_ERROR, "Failed to reinit request object %d: %s (%d)\n",
               output->request_fd, strerror(errno), errno);
        return ret;
    }

    // Ensure request is marked as dequeued
    ctx->queued_request &= ~(1 << output->index);

    return 0;
}

int ff_v4l2_request_append_output(AVCodecContext *avctx,
                                  V4L2RequestPictureContext *pic,
                                  const uint8_t *data, uint32_t size)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);

    // Append data to OUTPUT buffer and ensure there is enough space for padding
    if (pic->output->bytesused + size + AV_INPUT_BUFFER_PADDING_SIZE <= pic->output->size) {
        memcpy(pic->output->addr + pic->output->bytesused, data, size);
        pic->output->bytesused += size;
        return 0;
    } else {
        av_log(ctx, AV_LOG_ERROR,
               "Failed to append %u bytes data to OUTPUT buffer %d (%u of %u used)\n",
               size, pic->output->index, pic->output->bytesused, pic->output->size);
        return AVERROR(ENOMEM);
    }
}

static int v4l2_request_queue_decode(AVCodecContext *avctx,
                                     V4L2RequestPictureContext *pic,
                                     struct v4l2_ext_control *control, int count,
                                     bool first_slice, bool last_slice)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    uint32_t flags;
    int ret;

    if (first_slice) {
        /*
         * Wait on dequeue of the target CAPTURE buffer. Otherwise V4L2 decoder
         * may use a different CAPTURE buffer than hwaccel expects.
         *
         * Normally decoding has already completed when a CAPTURE buffer is
         * reused so this is more or less a no-op, however in some situations
         * FFmpeg may reuse an AVFrame early, i.e. when no output frame was
         * produced prior time, and a synchronization is necessary.
         */
        ret = v4l2_request_wait_on_capture(ctx, pic->capture_index);
        if (ret < 0)
            return ret;
    }

    ff_mutex_lock(&ctx->mutex);

    /*
     * The OUTPUT buffer tied to prior use of current request object can
     * independently be dequeued before the full decode request has been
     * completed. This may happen when a decoder use multi stage decoding,
     * e.g. rpi-hevc-dec. In such case we can start reusing the OUTPUT buffer,
     * however we must wait on the prior request to fully complete before we
     * can reuse the request object, and a synchronization is necessary.
     */
    ret = v4l2_request_wait_on_request(ctx, pic->output);
    if (ret < 0)
        goto fail;

    /*
     * Dequeue any completed OUTPUT buffers, this is strictly not necessary,
     * however if a synchronization was necessary for the CAPTURE and/or request
     * there is more than likely one or more OUTPUT buffers that can be dequeued.
     */
    if (ctx->queued_output)
        v4l2_request_dequeue_completed_buffers(ctx, ctx->fctxi->output.format.type);

    // Set codec controls for current request
    ret = v4l2_request_set_controls(ctx, pic->output->request_fd, control, count);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to set %d control(s) for request %d: %s (%d)\n",
               count, pic->output->request_fd, strerror(errno), errno);
        goto fail;
    }

    // Ensure there is zero padding at the end of bitstream data
    memset(pic->output->addr + pic->output->bytesused, 0, AV_INPUT_BUFFER_PADDING_SIZE);

    /*
     * Use CAPTURE buffer index as base for V4L2 frame reference.
     * This works because a CAPTURE buffer is closely tied to a AVFrame
     * and FFmpeg handle all frame reference tracking for us.
     */
    pic->output->timestamp = (struct timeval) {
        .tv_sec = 0,
        .tv_usec = pic->capture_index + 1,
    };

    /*
     * Queue the OUTPUT buffer of current request. The CAPTURE buffer may be
     * hold by the V4L2 decoder unless this is the last slice of a frame.
     */
    flags = last_slice ? 0 : V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;
    ret = v4l2_request_queue_output_buffer(ctx, pic->output, flags);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to queue OUTPUT buffer %d for request %d: %s (%d)\n",
               pic->output->index, pic->output->request_fd, strerror(errno), errno);
        goto fail;
    }

    if (first_slice) {
        /*
         * Queue the target CAPTURE buffer, hwaccel expect and depend on that
         * this specific CAPTURE buffer will be used as decode target for
         * current request, otherwise frames may be output in wrong order or
         * wrong CAPTURE buffer could get used as a reference frame.
         */
        ret = v4l2_request_queue_capture_buffer(ctx, pic->capture_index);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "Failed to queue CAPTURE buffer %d for request %d: %s (%d)\n",
                   pic->capture_index, pic->output->request_fd, strerror(errno), errno);
            goto fail;
        }
    }

    // Queue current request
    ret = ioctl(pic->output->request_fd, MEDIA_REQUEST_IOC_QUEUE);
    if (ret < 0) {
        ret = AVERROR(errno);
        av_log(ctx, AV_LOG_ERROR, "Failed to queue request object %d: %s (%d)\n",
               pic->output->request_fd, strerror(errno), errno);
        goto fail;
    }

    // Mark current request as queued
    ctx->queued_request |= 1 << pic->output->index;

    ret = 0;
fail:
    ff_mutex_unlock(&ctx->mutex);
    return ret;
}

int ff_v4l2_request_decode_slice(AVCodecContext *avctx,
                                 V4L2RequestPictureContext *pic,
                                 struct v4l2_ext_control *control, int count,
                                 bool first_slice, bool last_slice)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);

    /*
     * Fallback to queue each slice as a full frame when holding CAPTURE
     * buffers is not supported by the driver.
     */
    if ((ctx->fctxi->output.capabilities & V4L2_BUF_CAP_SUPPORTS_M2M_HOLD_CAPTURE_BUF) !=
        V4L2_BUF_CAP_SUPPORTS_M2M_HOLD_CAPTURE_BUF)
        return v4l2_request_queue_decode(avctx, pic, control, count, true, true);

    return v4l2_request_queue_decode(avctx, pic, control, count,
                                     first_slice, last_slice);
}

int ff_v4l2_request_decode_frame(AVCodecContext *avctx,
                                 V4L2RequestPictureContext *pic,
                                 struct v4l2_ext_control *control, int count)
{
    return v4l2_request_queue_decode(avctx, pic, control, count, true, true);
}

static int v4l2_request_post_process(void *logctx, AVFrame *frame)
{
    uint32_t index = v4l2_request_frameindex(frame);
    FrameDecodeData *fdd = (FrameDecodeData *)frame->private_ref->data;
    V4L2RequestContext *ctx = fdd->hwaccel_priv;

    // Wait on CAPTURE buffer before returning the frame to application
    return v4l2_request_wait_on_capture(ctx, index);
}

int ff_v4l2_request_reset_picture(AVCodecContext *avctx, V4L2RequestPictureContext *pic)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);

    // Get and wait on next OUTPUT buffer from circular queue
    pic->output = v4l2_request_next_output(ctx);
    if (!pic->output)
        return AVERROR(EINVAL);

    return 0;
}

int ff_v4l2_request_start_frame(AVCodecContext *avctx,
                                V4L2RequestPictureContext *pic,
                                AVFrame *frame)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    uint32_t index = v4l2_request_frameindex(frame);
    FrameDecodeData *fdd = (FrameDecodeData *)frame->private_ref->data;
    int ret;

    // Get next OUTPUT buffer from circular queue
    ret = ff_v4l2_request_reset_picture(avctx, pic);
    if (ret)
        return ret;

    // Ensure CAPTURE buffer is dequeued before reuse
    ret = v4l2_request_wait_on_capture(ctx, index);
    if (ret)
        return ret;

    // Wait on CAPTURE buffer in post_process() before returning to application
    fdd->hwaccel_priv = ctx;
    fdd->post_process = v4l2_request_post_process;

    // CAPTURE buffer used for current frame
    pic->capture_index = index;

    return 0;
}

void ff_v4l2_request_flush(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    enum v4l2_buf_type type = ctx->fctxi->output.format.type;
    struct pollfd pollfd = {
        .fd = ctx->fctxi->video_fd,
        .events = POLLOUT,
    };

    ff_mutex_lock(&ctx->mutex);

    // Dequeue all completed OUTPUT buffers
    if (ctx->queued_output)
        v4l2_request_dequeue_completed_buffers(ctx, type);

    // Wait on any remaining OUTPUT buffer
    while (ctx->queued_output) {
        int ret = poll(&pollfd, 1, 2000);
        if (ret <= 0)
            break;

        ret = v4l2_request_dequeue_buffer(ctx, type);
        if (ret < 0 && ret != AVERROR(EAGAIN))
            break;
    }

    // Dequeue all completed CAPTURE buffers
    if (ctx->queued_capture)
        v4l2_request_dequeue_completed_buffers(ctx, ctx->fctxi->capture.format.type);

    ff_mutex_unlock(&ctx->mutex);
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
        // Flush and wait on all pending requests
        ff_v4l2_request_flush(avctx);

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
