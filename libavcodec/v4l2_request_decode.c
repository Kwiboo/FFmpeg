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

#include "decode.h"
#include "v4l2_request_internal.h"

uint64_t ff_v4l2_request_get_capture_timestamp(AVFrame *frame)
{
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)frame->data[0];
    return req ? v4l2_timeval_to_ns(&req->capture.buffer.timestamp) : 0;
}

int ff_v4l2_request_reset_frame(AVCodecContext *avctx, AVFrame *frame)
{
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)frame->data[0];
    memset(&req->drm, 0, sizeof(AVDRMFrameDescriptor));
    req->output.used = 0;
    return 0;
}

int ff_v4l2_request_start_frame(AVCodecContext *avctx,
                                V4L2RequestPictureContext *pic,
                                AVFrame *frame)
{
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)frame->data[0];

    pic->output = &req->output;
    pic->capture = &req->capture;

    return ff_v4l2_request_reset_frame(avctx, frame);
}

int ff_v4l2_request_append_output(AVCodecContext *avctx, AVFrame *frame, const uint8_t *data, uint32_t size)
{
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)frame->data[0];
    if (req->output.used + size + (AV_INPUT_BUFFER_PADDING_SIZE * 4) <= req->output.size) {
        memcpy(req->output.addr + req->output.used, data, size);
        req->output.used += size;
    } else {
        av_log(avctx, AV_LOG_ERROR, "%s: output.used=%u output.size=%u size=%u\n", __func__, req->output.used, req->output.size, size);
    }
    return 0;
}

static int v4l2_request_queue_buffer(V4L2RequestContext *ctx, int request_fd, V4L2RequestBuffer *buf, uint32_t flags)
{
    struct v4l2_plane planes[1] = {};
    struct v4l2_buffer buffer = {
        .type = buf->buffer.type,
        .memory = buf->buffer.memory,
        .index = buf->index,
        .timestamp.tv_usec = buf->index + 1,
        .bytesused = buf->used,
        .request_fd = request_fd,
        .flags = ((request_fd >= 0) ? V4L2_BUF_FLAG_REQUEST_FD : 0) | flags,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(buf->buffer.type)) {
        planes[0].bytesused = buf->used;
        buffer.bytesused = 0;
        buffer.length = 1;
        buffer.m.planes = planes;
    }

    return ioctl(ctx->video_fd, VIDIOC_QBUF, &buffer);
}

static int v4l2_request_dequeue_buffer(V4L2RequestContext *ctx, V4L2RequestBuffer *buf)
{
    int ret;
    struct v4l2_plane planes[1] = {};
    struct v4l2_buffer buffer = {
        .type = buf->buffer.type,
        .memory = buf->buffer.memory,
        .index = buf->index,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(buf->buffer.type)) {
        buffer.length = 1;
        buffer.m.planes = planes;
    }

    ret = ioctl(ctx->video_fd, VIDIOC_DQBUF, &buffer);
    if (ret < 0)
        return ret;

    buf->buffer.timestamp = buffer.timestamp;
    return 0;
}

static int v4l2_request_queue_decode(AVCodecContext *avctx, AVFrame *frame, struct v4l2_ext_control *control, int count, bool first_slice, bool last_slice)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)frame->data[0];
    struct timeval tv = { 2, 0 };
    fd_set except_fds;
    int ret;

    av_log(avctx, AV_LOG_DEBUG, "%s: ctx=%p used=%u controls=%d index=%d fd=%d request_fd=%d first_slice=%d last_slice=%d\n", __func__,
           ctx, req->output.used, count, req->capture.index, req->capture.fd, req->request_fd, first_slice, last_slice);

    ret = ff_v4l2_request_set_request_controls(ctx, req->request_fd, control, count);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: set controls failed for request %d, %s (%d)\n", __func__, req->request_fd, strerror(errno), errno);
        return -1;
    }

    memset(req->output.addr + req->output.used, 0, AV_INPUT_BUFFER_PADDING_SIZE * 4);

    ret = v4l2_request_queue_buffer(ctx, req->request_fd, &req->output, last_slice ? 0 : V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: queue output buffer %d failed for request %d, %s (%d)\n", __func__, req->output.index, req->request_fd, strerror(errno), errno);
        return -1;
    }

    if (first_slice) {
        ret = v4l2_request_queue_buffer(ctx, -1, &req->capture, 0);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "%s: queue capture buffer %d failed for request %d, %s (%d)\n", __func__, req->capture.index, req->request_fd, strerror(errno), errno);
            return -1;
        }
    }

    // NOTE: do we need to dequeue when request fails/timeout?

    // 4. queue request and wait
    ret = ioctl(req->request_fd, MEDIA_REQUEST_IOC_QUEUE, NULL);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: queue request %d failed, %s (%d)\n", __func__, req->request_fd, strerror(errno), errno);
        goto fail;
    }

    FD_ZERO(&except_fds);
    FD_SET(req->request_fd, &except_fds);

    ret = select(req->request_fd + 1, NULL, NULL, &except_fds, &tv);
    if (ret == 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: request %d timeout\n", __func__, req->request_fd);
        goto fail;
    } else if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: select request %d failed, %s (%d)\n", __func__, req->request_fd, strerror(errno), errno);
        goto fail;
    }

    ret = v4l2_request_dequeue_buffer(ctx, &req->output);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: dequeue output buffer %d failed for request %d, %s (%d)\n", __func__, req->output.index, req->request_fd, strerror(errno), errno);
        return -1;
    }

    if (last_slice) {
        ret = v4l2_request_dequeue_buffer(ctx, &req->capture);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "%s: dequeue capture buffer %d failed for request %d, %s (%d)\n", __func__, req->capture.index, req->request_fd, strerror(errno), errno);
            return -1;
        }
    }

    // TODO: check errors
    // buffer.flags & V4L2_BUF_FLAG_ERROR

    ret = ioctl(req->request_fd, MEDIA_REQUEST_IOC_REINIT, NULL);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: reinit request %d failed, %s (%d)\n", __func__, req->request_fd, strerror(errno), errno);
        return -1;
    }

    if (last_slice)
        return ff_v4l2_request_set_drm_descriptor(req, &ctx->format);

    return 0;

fail:
    ret = v4l2_request_dequeue_buffer(ctx, &req->output);
    if (ret < 0)
        av_log(avctx, AV_LOG_ERROR, "%s: dequeue output buffer %d failed for request %d, %s (%d)\n", __func__, req->output.index, req->request_fd, strerror(errno), errno);

    ret = v4l2_request_dequeue_buffer(ctx, &req->capture);
    if (ret < 0)
        av_log(avctx, AV_LOG_ERROR, "%s: dequeue capture buffer %d failed for request %d, %s (%d)\n", __func__, req->capture.index, req->request_fd, strerror(errno), errno);

    ret = ioctl(req->request_fd, MEDIA_REQUEST_IOC_REINIT, NULL);
    if (ret < 0)
        av_log(avctx, AV_LOG_ERROR, "%s: reinit request %d failed, %s (%d)\n", __func__, req->request_fd, strerror(errno), errno);

    return -1;
}

int ff_v4l2_request_decode_slice(AVCodecContext *avctx, AVFrame *frame, struct v4l2_ext_control *control, int count, bool first_slice, bool last_slice)
{
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)frame->data[0];

    // fall back to queue each slice as a full frame
    if ((req->output.capabilities & V4L2_BUF_CAP_SUPPORTS_M2M_HOLD_CAPTURE_BUF) != V4L2_BUF_CAP_SUPPORTS_M2M_HOLD_CAPTURE_BUF)
        return v4l2_request_queue_decode(avctx, frame, control, count, true, true);

    return v4l2_request_queue_decode(avctx, frame, control, count, first_slice, last_slice);
}

int ff_v4l2_request_decode_frame(AVCodecContext *avctx, AVFrame *frame, struct v4l2_ext_control *control, int count)
{
    return v4l2_request_queue_decode(avctx, frame, control, count, true, true);
}
