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

#ifndef AVUTIL_HWCONTEXT_V4L2REQUEST_INTERNAL_H
#define AVUTIL_HWCONTEXT_V4L2REQUEST_INTERNAL_H

#include "buffer.h"
#include "hwcontext_v4l2request.h"

/**
 * @file
 * FFmpeg internal API-specific header for AV_HWDEVICE_TYPE_V4L2REQUEST.
 */

/**
 * Internal context for the initialized V4L2 stateless decoder/encoder session.
 */
struct AVV4L2RequestFramesContextInternal {
    /**
     * Media device file descriptor of the initialized session.
     */
    int media_fd;

    /**
     * Video device file descriptor of the initialized session.
     */
    int video_fd;

    /**
     * Details of the initialized CAPTURE and OUTPUT queues.
     */
    struct {
        /**
         * V4L2 buffer format.
         */
        struct v4l2_format format;

        /**
         * V4L2 buffer capabilities flags.
         */
        uint32_t capabilities;

        /**
        * Buffer pool of allocated V4L2 buffers.
        *
        * AVBufferRef.data points to a struct v4l2_buffer for the created buffer.
        */
        AVBufferPool *pool;
    } capture, output;
};

#endif /* AVUTIL_HWCONTEXT_V4L2REQUEST_INTERNAL_H */
