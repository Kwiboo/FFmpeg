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

#include <poll.h>
#include <sys/ioctl.h>

#include "libavutil/time.h"
#include "decode.h"
#include "v4l2_request_internal.h"

#define INPUT_BUFFER_PADDING_SIZE   (AV_INPUT_BUFFER_PADDING_SIZE * 64)

uint64_t ff_v4l2_request_get_capture_timestamp(AVFrame *frame)
{
    V4L2RequestFrameDescriptor *desc = v4l2_request_framedesc(frame);

    /*
     * The capture buffer index is used as a base for V4L2 frame reference.
     * This works because frames are decoded into a capture buffer that is
     * closely tied to an AVFrame.
     */
    return desc ? v4l2_timeval_to_ns(&desc->capture.buffer.timestamp) : 0;
}

static int v4l2_request_queue_buffer(V4L2RequestContext *ctx, int request_fd,
                                     V4L2RequestBuffer *buf, uint32_t flags)
{
    struct v4l2_plane planes[1] = {};
    struct v4l2_buffer buffer = {
        .index = buf->index,
        .type = buf->buffer.type,
        .memory = buf->buffer.memory,
        .timestamp = buf->buffer.timestamp,
        .bytesused = buf->used,
        .request_fd = request_fd,
        .flags = ((request_fd >= 0) ? V4L2_BUF_FLAG_REQUEST_FD : 0) | flags,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(buffer.type)) {
        planes[0].bytesused = buf->used;
        buffer.bytesused = 0;
        buffer.length = 1;
        buffer.m.planes = planes;
    }

    // Queue the buffer
    if (ioctl(ctx->video_fd, VIDIOC_QBUF, &buffer) < 0)
        return AVERROR(errno);

    return 0;
}

static int v4l2_request_dequeue_buffer(V4L2RequestContext *ctx,
                                       enum v4l2_buf_type type)
{
    struct v4l2_plane planes[1] = {};
    struct v4l2_buffer buffer = {
        .type = type,
        .memory = V4L2_MEMORY_MMAP,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(buffer.type)) {
        buffer.length = 1;
        buffer.m.planes = planes;
    }

    // Dequeue next completed buffer
    if (ioctl(ctx->video_fd, VIDIOC_DQBUF, &buffer) < 0)
        return AVERROR(errno);

    return 0;
}

int ff_v4l2_request_append_output(AVCodecContext *avctx,
                                  V4L2RequestPictureContext *pic,
                                  const uint8_t *data, uint32_t size)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);

    // Append data to output buffer and ensure there is enough space for padding
    if (pic->output->used + size + INPUT_BUFFER_PADDING_SIZE <= pic->output->size) {
        memcpy(pic->output->addr + pic->output->used, data, size);
        pic->output->used += size;
        return 0;
    } else {
        av_log(ctx, AV_LOG_ERROR,
               "Failed to append %u bytes data to output buffer %d (%u of %u used)\n",
               size, pic->output->index, pic->output->used, pic->output->size);
        return AVERROR(ENOMEM);
    }
}

static int v4l2_request_queue_decode(AVCodecContext *avctx,
                                     V4L2RequestPictureContext *pic,
                                     struct v4l2_ext_control *control, int count,
                                     bool first_slice, bool last_slice)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    bool request_done, output_done, capture_done;
    uint32_t flags;
    int ret;

    struct pollfd pollfd[] = {
        {
            .fd = pic->output->fd,
            .events = POLLPRI,
        },
        {
            .fd = ctx->video_fd,
            .events = POLLOUT | POLLIN,
        },
    };

    // Set codec controls for current request
    ret = ff_v4l2_request_set_request_controls(ctx, pic->output->fd, control, count);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to set %d control(s) for request %d: %s (%d)\n",
               count, pic->output->fd, strerror(errno), errno);
        goto fail;
    }

    // Ensure there is zero padding at the end of bitstream data
    memset(pic->output->addr + pic->output->used, 0, INPUT_BUFFER_PADDING_SIZE);

    // Use timestamp of the capture buffer for V4L2 frame reference
    pic->output->buffer.timestamp = pic->capture->buffer.timestamp;

    /*
     * Queue the output buffer of current request. The capture buffer may be
     * hold by the V4L2 decoder unless this is the last slice of a frame.
     */
    flags = last_slice ? 0 : V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;
    ret = v4l2_request_queue_buffer(ctx, pic->output->fd, pic->output, flags);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to queue output buffer %d for request %d: %s (%d)\n",
               pic->output->index, pic->output->fd, strerror(errno), errno);
        ret = AVERROR(errno);
        goto fail;
    }

    if (first_slice) {
        /*
         * Queue the target capture buffer, hwaccel expect and depend on that
         * this specific capture buffer will be used as decode target for
         * current request, otherwise frames may be output in wrong order or
         * wrong capture buffer could get used as a reference frame.
         */
        ret = v4l2_request_queue_buffer(ctx, -1, pic->capture, 0);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "Failed to queue capture buffer %d for request %d: %s (%d)\n",
                   pic->capture->index, pic->output->fd, strerror(errno), errno);
            ret = AVERROR(errno);
            goto fail;
        }
    }

    // Queue current request
    ret = ioctl(pic->output->fd, MEDIA_REQUEST_IOC_QUEUE, NULL);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to queue request object %d: %s (%d)\n",
               pic->output->fd, strerror(errno), errno);
        ret = AVERROR(errno);
        goto fail;
    }

    request_done = output_done = false;
    capture_done = !last_slice;

    do
    {
        ret = poll(pollfd, 2, 2000);
        if (ret <= 0)
            break;

        if (pollfd[0].revents & (POLLPRI|POLLERR))
            request_done = true;

        if (pollfd[1].revents & POLLOUT)
            output_done = true;

        if (pollfd[1].revents & POLLIN)
            capture_done = true;

    } while (!capture_done || !output_done || !request_done);

    if (ret <= 0 || !capture_done || !output_done || !request_done)
        av_log(ctx, AV_LOG_ERROR, "poll: ret=%d done=[%d,%d,%d]\n", ret, request_done, output_done, capture_done);

    ret = v4l2_request_dequeue_buffer(ctx, ctx->output_type);
    if (ret)
        goto fail;

    if (last_slice) {
        ret = v4l2_request_dequeue_buffer(ctx, ctx->format.type);
        if (ret)
            goto fail;
    }

    // Reinit the request object
    if (ioctl(pic->output->fd, MEDIA_REQUEST_IOC_REINIT, NULL) < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to reinit request object %d: %s (%d)\n",
               pic->output->fd, strerror(errno), errno);
        ret = AVERROR(errno);
        goto fail;
    }

    av_buffer_unref(&pic->output_ref);

    ret = 0;
fail:

    av_log(avctx, AV_LOG_DEBUG, "%s: ctx=%p used=%u controls=%d capture.index=%d output.index=%d request_fd=%d first_slice=%d last_slice=%d ret=%d\n", __func__,
           ctx, pic->output->used, count, pic->capture->index, pic->output->index, pic->output->fd, first_slice, last_slice, ret);
    return ret;
}

int ff_v4l2_request_decode_slice(AVCodecContext *avctx,
                                 V4L2RequestPictureContext *pic,
                                 struct v4l2_ext_control *control, int count,
                                 bool first_slice, bool last_slice)
{
    /*
     * Fallback to queue each slice as a full frame when holding capture
     * buffers is not supported by the driver.
     */
    if ((pic->output->capabilities & V4L2_BUF_CAP_SUPPORTS_M2M_HOLD_CAPTURE_BUF) !=
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

int ff_v4l2_request_reset_picture(AVCodecContext *avctx, V4L2RequestPictureContext *pic)
{
    // Reset used state
    pic->output->used = 0;

    // Use timestamp of the capture buffer for V4L2 frame reference
    pic->output->buffer.timestamp = pic->capture->buffer.timestamp;

    return 0;
}

int ff_v4l2_request_start_frame(AVCodecContext *avctx,
                                V4L2RequestPictureContext *pic,
                                AVFrame *frame)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    V4L2RequestFrameDescriptor *desc = v4l2_request_framedesc(frame);
    V4L2RequestOutputDescriptor *output;

    if (!desc)
        return AVERROR(EINVAL);

    // Output buffer used for current frame
    //pic->output = &ctx->output[desc->capture.index];
    pic->output_ref = av_buffer_pool_get(ctx->output_pool);
    if (!pic->output_ref)
        return AVERROR(ENOMEM);

    output = (V4L2RequestOutputDescriptor *)pic->output_ref->data;
    pic->output = &output->output;

    // Capture buffer used for current frame
    pic->capture = &desc->capture;

    return ff_v4l2_request_reset_picture(avctx, pic);
}
