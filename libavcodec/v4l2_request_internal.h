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

#ifndef AVCODEC_V4L2_REQUEST_INTERNAL_H
#define AVCODEC_V4L2_REQUEST_INTERNAL_H

#include <linux/media.h>

#include "internal.h"
#include "v4l2_request.h"

typedef struct V4L2RequestFrameDescriptor {
    AVDRMFrameDescriptor base;
    V4L2RequestBuffer capture;
} V4L2RequestFrameDescriptor;

static inline V4L2RequestContext *v4l2_request_context(AVCodecContext *avctx)
{
    return (V4L2RequestContext *)avctx->internal->hwaccel_priv_data;
}

static inline V4L2RequestFrameDescriptor *v4l2_request_framedesc(AVFrame *frame)
{
    return (V4L2RequestFrameDescriptor *)frame->data[0];
}

enum AVPixelFormat ff_v4l2_request_get_sw_format(struct v4l2_format *format);

int ff_v4l2_request_set_drm_descriptor(V4L2RequestFrameDescriptor *framedesc,
                                       struct v4l2_format *format);

int ff_v4l2_request_probe(AVCodecContext *avctx, uint32_t pixelformat,
                          uint32_t buffersize, struct v4l2_ext_control *control,
                          int count);

#endif /* AVCODEC_V4L2_REQUEST_INTERNAL_H */
