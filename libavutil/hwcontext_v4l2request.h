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

/**
 * @file
 * An API-specific header for AV_HWDEVICE_TYPE_V4L2REQUEST.
 */

/**
 * V4L2 Request API device details.
 *
 * Allocated as AVHWDeviceContext.hwctx
 */
typedef struct AVV4L2RequestDeviceContext {
    /**
     * File descriptor of media device.
     *
     * Defaults to -1 for auto-detect.
     */
    int media_fd;
} AVV4L2RequestDeviceContext;

#endif /* AVUTIL_HWCONTEXT_V4L2REQUEST_H */
