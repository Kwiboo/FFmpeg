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
    struct v4l2_ext_control *init_controls;
    int nb_init_controls;
} V4L2RequestContext;

int ff_v4l2_request_frame_params(AVCodecContext *avctx,
                                 AVBufferRef *hw_frames_ctx,
                                 uint32_t pixelformat,
                                 uint8_t bit_depth);

int ff_v4l2_request_uninit(AVCodecContext *avctx);

int ff_v4l2_request_init(AVCodecContext *avctx,
                         struct v4l2_ext_control *control, int count,
                         int (*post_frames_ctx)(AVCodecContext *avctx));

#endif /* AVCODEC_V4L2_REQUEST_H */
