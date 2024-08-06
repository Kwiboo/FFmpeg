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

#ifndef AVCODEC_V4L2_REQUEST_H
#define AVCODEC_V4L2_REQUEST_H

#include <stdbool.h>
#include <stdint.h>
#include <linux/videodev2.h>

#include "libavutil/buffer.h"
#include "libavutil/log.h"
#include "libavutil/thread.h"
#include "avcodec.h"

typedef struct AVV4L2RequestFramesContextInternal AVV4L2RequestFramesContextInternal;

typedef struct V4L2RequestOutputBuffer {
    AVBufferRef *ref;
    uint32_t index;
    int request_fd;
    uint8_t *addr;
    uint32_t size;
    uint32_t bytesused;
    struct timeval timestamp;
} V4L2RequestOutputBuffer;

typedef struct V4L2RequestContext {
    const AVClass *av_class;
    AVBufferRef *frames_ref;
    AVV4L2RequestFramesContextInternal *fctxi;
    AVMutex mutex;
    V4L2RequestOutputBuffer output[4];
    uint8_t next_output;
    uint32_t queued_output;
    uint32_t queued_request;
    uint64_t queued_capture;
    struct v4l2_ext_control *init_controls;
    int nb_init_controls;
} V4L2RequestContext;

typedef struct V4L2RequestPictureContext {
    V4L2RequestOutputBuffer *output;
    uint32_t capture_index;
} V4L2RequestPictureContext;

uint64_t ff_v4l2_request_get_capture_timestamp(AVFrame *frame);

int ff_v4l2_request_query_control(AVCodecContext *avctx,
                                  struct v4l2_query_ext_ctrl *control);

int ff_v4l2_request_query_control_default_value(AVCodecContext *avctx,
                                                uint32_t id);

int ff_v4l2_request_set_controls(AVCodecContext *avctx,
                                 struct v4l2_ext_control *control, int count);

int ff_v4l2_request_append_output(AVCodecContext *avctx,
                                  V4L2RequestPictureContext *pic,
                                  const uint8_t *data, uint32_t size);

int ff_v4l2_request_decode_slice(AVCodecContext *avctx,
                                 V4L2RequestPictureContext *pic,
                                 struct v4l2_ext_control *control, int count,
                                 bool first_slice, bool last_slice);

int ff_v4l2_request_decode_frame(AVCodecContext *avctx,
                                 V4L2RequestPictureContext *pic,
                                 struct v4l2_ext_control *control, int count);

int ff_v4l2_request_reset_picture(AVCodecContext *avctx,
                                  V4L2RequestPictureContext *pic);

int ff_v4l2_request_start_frame(AVCodecContext *avctx,
                                V4L2RequestPictureContext *pic, AVFrame *frame);

void ff_v4l2_request_flush(AVCodecContext *avctx);

int ff_v4l2_request_frame_params(AVCodecContext *avctx,
                                 AVBufferRef *hw_frames_ctx,
                                 uint32_t pixelformat,
                                 uint8_t bit_depth);

int ff_v4l2_request_uninit(AVCodecContext *avctx);

int ff_v4l2_request_init(AVCodecContext *avctx,
                         struct v4l2_ext_control *control, int count,
                         int (*post_frames_ctx)(AVCodecContext *avctx));

#endif /* AVCODEC_V4L2_REQUEST_H */
