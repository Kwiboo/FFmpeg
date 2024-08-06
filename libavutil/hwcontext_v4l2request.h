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

#ifndef AVUTIL_HWCONTEXT_V4L2REQUEST_H
#define AVUTIL_HWCONTEXT_V4L2REQUEST_H

#include <stdint.h>
#include <linux/videodev2.h>

/**
 * @file
 * An API-specific header for AV_HWDEVICE_TYPE_V4L2REQUEST.
 */

typedef struct AVV4L2RequestFramesContextInternal AVV4L2RequestFramesContextInternal;

/**
 * V4L2 Request API frames context.
 *
 * This struct is allocated as AVHWFramesContext.hwctx
 */
typedef struct AVV4L2RequestFramesContext {
    /**
     * Internal context for the initialized V4L2 stateless decoder/encoder session.
     */
    AVV4L2RequestFramesContextInternal *internal;

    /**
     * V4L2_PIX_FMT_* coded pixel format to set on the OUTPUT queue (decoders)
     * or the CAPTURE queue (encoders) during initialization.
     *
     * This field must be set by caller before av_hwframe_ctx_init() is called.
     */
    uint32_t pixelformat;

    /**
     * Optional bit depth of the frame pixel format, e.g. 8 or 10.
     *
     * This field should be set by caller before av_hwframe_ctx_init() is called,
     * the field will be updated to match the selected frame pixel format after
     * successful initialization.
     */
    uint32_t bit_depth;

    /**
     * Optional codec-specific extended controls to be set during initialization.
     *
     * These fields should be set by caller before av_hwframe_ctx_init() is called,
     * fields are reset to NULL and 0 after successful initialization.
     */
    struct v4l2_ext_control *init_controls;
    int nb_init_controls;
} AVV4L2RequestFramesContext;

#endif /* AVUTIL_HWCONTEXT_V4L2REQUEST_H */
