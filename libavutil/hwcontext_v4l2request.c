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

#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/media.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <drm_fourcc.h>
#include <libudev.h>

#include "avassert.h"
#include "hwcontext_drm.h"
#include "hwcontext_internal.h"
#include "hwcontext_v4l2request_internal.h"
#include "mem.h"

typedef struct V4L2RequestVideoDecoder {
    dev_t media_dev;
    dev_t video_dev;
    uint32_t *pixelformats;
    int nb_pixelformats;
} V4L2RequestVideoDecoder;

typedef struct V4L2RequestDeviceContext {
    V4L2RequestVideoDecoder *decoders;
    int nb_decoders;
} V4L2RequestDeviceContext;

typedef struct V4L2RequestFramesContext {
    AVV4L2RequestFramesContext p;
    AVV4L2RequestFramesContextInternal internal;
} V4L2RequestFramesContext;

typedef struct V4L2RequestFrameDescriptor {
    AVDRMFrameDescriptor base;
    AVBufferRef *ref;
    uint32_t index;
    int fd[AV_DRM_MAX_PLANES];
} V4L2RequestFrameDescriptor;

static const struct {
    uint32_t pixelformat;
    enum AVPixelFormat sw_format;
    uint32_t drm_format;
    uint64_t format_modifier;
    uint32_t bit_depth;
} v4l2request_capture_pixelformats[] = {
    { V4L2_PIX_FMT_NV12, AV_PIX_FMT_NV12, DRM_FORMAT_NV12, DRM_FORMAT_MOD_LINEAR, 8 },
#if defined(V4L2_PIX_FMT_NV12_32L32)
    { V4L2_PIX_FMT_NV12_32L32, AV_PIX_FMT_YUV420P, DRM_FORMAT_NV12, DRM_FORMAT_MOD_ALLWINNER_TILED, 8 },
#endif
#if defined(V4L2_PIX_FMT_NV15) && defined(DRM_FORMAT_NV15)
    { V4L2_PIX_FMT_NV15, AV_PIX_FMT_YUV420P10, DRM_FORMAT_NV15, DRM_FORMAT_MOD_LINEAR, 10 },
#endif
    { V4L2_PIX_FMT_NV16, AV_PIX_FMT_NV16, DRM_FORMAT_NV16, DRM_FORMAT_MOD_LINEAR, 8 },
#if defined(V4L2_PIX_FMT_NV20) && defined(DRM_FORMAT_NV20)
    { V4L2_PIX_FMT_NV20, AV_PIX_FMT_YUV422P10, DRM_FORMAT_NV20, DRM_FORMAT_MOD_LINEAR, 10 },
#endif
#if defined(V4L2_PIX_FMT_P010) && defined(DRM_FORMAT_P010)
    { V4L2_PIX_FMT_P010, AV_PIX_FMT_P010, DRM_FORMAT_P010, DRM_FORMAT_MOD_LINEAR, 10 },
#endif
#if defined(V4L2_PIX_FMT_NV12MT_COL128) && defined(V4L2_PIX_FMT_NV12MT_10_COL128)
    { V4L2_PIX_FMT_NV12MT_COL128, AV_PIX_FMT_YUV420P, DRM_FORMAT_NV12, DRM_FORMAT_MOD_BROADCOM_SAND128, 8 },
#if defined(DRM_FORMAT_P030)
    { V4L2_PIX_FMT_NV12MT_10_COL128, AV_PIX_FMT_YUV420P10, DRM_FORMAT_P030, DRM_FORMAT_MOD_BROADCOM_SAND128, 10 },
#endif
#endif
#if defined(V4L2_PIX_FMT_YUV420_8_AFBC_16X16_SPLIT)
    {
        .pixelformat = V4L2_PIX_FMT_YUV420_8_AFBC_16X16_SPLIT,
        .sw_format = AV_PIX_FMT_YUV420P,
        .drm_format = DRM_FORMAT_YUV420_8BIT,
        .format_modifier = DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
                                                   AFBC_FORMAT_MOD_SPARSE |
                                                   AFBC_FORMAT_MOD_SPLIT),
        .bit_depth = 8,
    },
#endif
#if defined(V4L2_PIX_FMT_YUV420_10_AFBC_16X16_SPLIT)
    {
        .pixelformat = V4L2_PIX_FMT_YUV420_10_AFBC_16X16_SPLIT,
        .sw_format = AV_PIX_FMT_YUV420P10,
        .drm_format = DRM_FORMAT_YUV420_10BIT,
        .format_modifier = DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
                                                   AFBC_FORMAT_MOD_SPARSE |
                                                   AFBC_FORMAT_MOD_SPLIT),
        .bit_depth = 10,
    },
#endif
};

static int v4l2request_set_drm_descriptor(AVDRMFrameDescriptor *desc,
                                          struct v4l2_format *format)
{
    AVDRMLayerDescriptor *layer = &desc->layers[0];
    uint32_t pixelformat = V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                           format->fmt.pix_mp.pixelformat :
                           format->fmt.pix.pixelformat;
    uint64_t format_modifier;

    layer->format = 0;
    for (int i = 0; i < FF_ARRAY_ELEMS(v4l2request_capture_pixelformats); i++) {
        if (pixelformat == v4l2request_capture_pixelformats[i].pixelformat) {
            layer->format = v4l2request_capture_pixelformats[i].drm_format;
            format_modifier = v4l2request_capture_pixelformats[i].format_modifier;
            break;
        }
    }
    if (!layer->format)
        return AVERROR(ENOENT);

    for (int i = 0; i < desc->nb_objects; i++) {
        desc->objects[i].format_modifier = format_modifier;
        desc->objects[i].size = V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                                format->fmt.pix_mp.plane_fmt[i].sizeimage :
                                format->fmt.pix.sizeimage;
    }

    desc->nb_layers = 1;
    layer->nb_planes = 1;

    layer->planes[0].object_index = 0;
    layer->planes[0].offset = 0;
    layer->planes[0].pitch = V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                             format->fmt.pix_mp.plane_fmt[0].bytesperline :
                             format->fmt.pix.bytesperline;

    // AFBC formats only use 1 plane, remaining use 2 planes
    if ((desc->objects[0].format_modifier >> 56) != DRM_FORMAT_MOD_VENDOR_ARM) {
        layer->nb_planes = 2;
        layer->planes[1].object_index = 0;
        layer->planes[1].offset = layer->planes[0].pitch *
                                  (V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                                   format->fmt.pix_mp.height :
                                   format->fmt.pix.height);
        layer->planes[1].pitch = layer->planes[0].pitch;
    }

#if defined(V4L2_PIX_FMT_NV12MT_COL128) && defined(V4L2_PIX_FMT_NV12MT_10_COL128)
    // Raspberry Pi formats need special handling
    if (pixelformat == V4L2_PIX_FMT_NV12MT_COL128 ||
        pixelformat == V4L2_PIX_FMT_NV12MT_10_COL128) {
        layer->planes[1].object_index = 1;
        layer->planes[1].offset = 0;
        layer->planes[0].pitch = (V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                                  format->fmt.pix_mp.height :
                                  format->fmt.pix.height);
        layer->planes[1].pitch = layer->planes[0].pitch / 2;
    }
#endif

    return 0;
}

static void v4l2request_device_uninit(AVHWDeviceContext *hwdev)
{
    V4L2RequestDeviceContext *hwctx = hwdev->hwctx;

    av_freep(&hwctx->decoders);
    hwctx->nb_decoders = 0;
}

static int v4l2request_device_create(AVHWDeviceContext *hwdev, const char *device,
                                     AVDictionary *opts, int flags)
{
    V4L2RequestDeviceContext *hwctx = hwdev->hwctx;

    hwctx->decoders = NULL;
    hwctx->nb_decoders = 0;

    // TODO: enumerate V4L2 Request API capable video decoders
    //       and fill hwctx->decoders and hwctx->nb_decoders,
    //       limit to decoders for the media 'device' when specified

    return 0;
}

static int v4l2request_set_format(AVHWFramesContext *hwfc,
                                  enum v4l2_buf_type type,
                                  uint32_t pixelformat,
                                  uint32_t buffersize)
{
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;
    struct v4l2_format format = {
        .type = type,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
        format.fmt.pix_mp.width = hwfc->width;
        format.fmt.pix_mp.height = hwfc->height;
        format.fmt.pix_mp.pixelformat = pixelformat;
        format.fmt.pix_mp.plane_fmt[0].sizeimage = buffersize;
        format.fmt.pix_mp.num_planes = 1;
    } else {
        format.fmt.pix.width = hwfc->width;
        format.fmt.pix.height = hwfc->height;
        format.fmt.pix.pixelformat = pixelformat;
        format.fmt.pix.sizeimage = buffersize;
    }

    if (ioctl(fctxi->video_fd, VIDIOC_S_FMT, &format) < 0)
        return AVERROR(errno);

    return 0;
}

static int v4l2request_select_capture_format(AVHWFramesContext *hwfc)
{
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;
    enum v4l2_buf_type type = fctxi->capture.format.type;
    uint32_t pixelformat, fallback = 0;
    struct v4l2_format format = {
        .type = type,
    };
    struct v4l2_fmtdesc fmtdesc = {
        .index = 0,
        .type = type,
    };

    // Get the driver preferred (or default) format
    if (ioctl(fctxi->video_fd, VIDIOC_G_FMT, &format) < 0)
        return AVERROR(errno);

    pixelformat = V4L2_TYPE_IS_MULTIPLANAR(type) ?
                  format.fmt.pix_mp.pixelformat :
                  format.fmt.pix.pixelformat;

    // Try to use the driver preferred format when it is a known format
    for (int i = 0; i < FF_ARRAY_ELEMS(v4l2request_capture_pixelformats); i++) {
        if (pixelformat == v4l2request_capture_pixelformats[i].pixelformat &&
            (fctx->bit_depth == v4l2request_capture_pixelformats[i].bit_depth ||
             !fctx->bit_depth))
            return v4l2request_set_format(hwfc, type, pixelformat, 0);
    }

    // Next try to use the first known format with matching bit depth
    while (ioctl(fctxi->video_fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0) {
        for (int i = 0; i < FF_ARRAY_ELEMS(v4l2request_capture_pixelformats); i++) {
            if (fmtdesc.pixelformat == v4l2request_capture_pixelformats[i].pixelformat) {
                if (fctx->bit_depth == v4l2request_capture_pixelformats[i].bit_depth ||
                    !fctx->bit_depth)
                    return v4l2request_set_format(hwfc, type, fmtdesc.pixelformat, 0);
                else if (!fallback)
                    fallback = fmtdesc.pixelformat;
            }
        }

        fmtdesc.index++;
    }

    // Fallback to use the first known format
    if (fallback)
        return v4l2request_set_format(hwfc, type, fallback, 0);

    return AVERROR(errno);
}

static int v4l2request_try_framesize(AVHWFramesContext *hwfc,
                                     uint32_t pixelformat)
{
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;
    struct v4l2_frmsizeenum frmsize = {
        .index = 0,
        .pixel_format = pixelformat,
    };

    // Enumerate and check if frame size is supported
    while (ioctl(fctxi->video_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) >= 0) {
        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE &&
            hwfc->width == frmsize.discrete.width &&
            hwfc->height == frmsize.discrete.height) {
            return 0;
        } else if ((frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE ||
                    frmsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) &&
                   hwfc->width >= frmsize.stepwise.min_width &&
                   hwfc->height >= frmsize.stepwise.min_height &&
                   hwfc->width <= frmsize.stepwise.max_width &&
                   hwfc->height <= frmsize.stepwise.max_height) {
            return 0;
        }

        frmsize.index++;
    }

    return AVERROR(errno);
}

static int v4l2request_try_format(AVHWFramesContext *hwfc,
                                  enum v4l2_buf_type type,
                                  uint32_t pixelformat)
{
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;
    struct v4l2_fmtdesc fmtdesc = {
        .index = 0,
        .type = type,
    };

    // Enumerate and check if format is supported
    while (ioctl(fctxi->video_fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0) {
        if (fmtdesc.pixelformat == pixelformat)
            return 0;

        fmtdesc.index++;
    }

    return AVERROR(errno);
}

static int v4l2request_set_controls(AVHWFramesContext *hwfc,
                                    struct v4l2_ext_control *control, int count)
{
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;
    struct v4l2_ext_controls controls = {
        .controls = control,
        .count = count,
    };

    if (!control || !count)
        return 0;

    if (ioctl(fctxi->video_fd, VIDIOC_S_EXT_CTRLS, &controls) < 0)
        return AVERROR(errno);

    return 0;
}

static int v4l2request_probe_video_device(AVHWFramesContext *hwfc,
                                          const char *path,
                                          uint32_t pixelformat,
                                          uint32_t buffersize)
{
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;
    struct v4l2_capability capability;
    struct v4l2_create_buffers buffers;
    unsigned int capabilities;
    int ret;

    /*
     * Open video device in non-blocking mode to support decoding using
     * multiple queued requests, required for e.g. multi stage decoding.
     */
    fctxi->video_fd = open(path, O_RDWR | O_NONBLOCK);
    if (fctxi->video_fd < 0) {
        ret = AVERROR(errno);
        av_log(hwfc, AV_LOG_ERROR, "Failed to open video device %s: %s (%d)\n",
               path, strerror(errno), errno);
        return ret;
    }

    // Query capabilities of the video device
    if (ioctl(fctxi->video_fd, VIDIOC_QUERYCAP, &capability) < 0) {
        ret = AVERROR(errno);
        av_log(hwfc, AV_LOG_ERROR, "Failed to query capabilities of %s: %s (%d)\n",
               path, strerror(errno), errno);
        goto fail;
    }

    // Use device capabilities of the opened device when supported
    capabilities = (capability.capabilities & V4L2_CAP_DEVICE_CAPS) ?
                   capability.device_caps : capability.capabilities;

    // Ensure streaming is supported on the video device
    if ((capabilities & V4L2_CAP_STREAMING) != V4L2_CAP_STREAMING) {
        ret = AVERROR(EINVAL);
        av_log(hwfc, AV_LOG_VERBOSE, "Device %s is missing streaming capability\n", path);
        goto fail;
    }

    // Ensure multi- or single-planar API can be used
    if ((capabilities & V4L2_CAP_VIDEO_M2M_MPLANE) == V4L2_CAP_VIDEO_M2M_MPLANE) {
        fctxi->output.format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        fctxi->capture.format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    } else if ((capabilities & V4L2_CAP_VIDEO_M2M) == V4L2_CAP_VIDEO_M2M) {
        fctxi->output.format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        fctxi->capture.format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    } else {
        ret = AVERROR(EINVAL);
        av_log(hwfc, AV_LOG_VERBOSE, "Device %s is missing mem2mem capability\n", path);
        goto fail;
    }

    // Query OUTPUT buffer capabilities
    buffers = (struct v4l2_create_buffers) {
        .count = 0,
        .memory = V4L2_MEMORY_MMAP,
        .format.type = fctxi->output.format.type,
    };
    if (ioctl(fctxi->video_fd, VIDIOC_CREATE_BUFS, &buffers) < 0) {
        ret = AVERROR(errno);
        av_log(hwfc, AV_LOG_ERROR,
               "Failed to query OUTPUT buffer capabilities of %s: %s (%d)\n",
               path, strerror(errno), errno);
        goto fail;
    }
    fctxi->output.capabilities = buffers.capabilities;

    // Ensure requests can be used
    if ((buffers.capabilities & V4L2_BUF_CAP_SUPPORTS_REQUESTS) !=
        V4L2_BUF_CAP_SUPPORTS_REQUESTS) {
        ret = AVERROR(EINVAL);
        av_log(hwfc, AV_LOG_VERBOSE, "Device %s is missing support for requests\n", path);
        goto fail;
    }

    // Ensure the codec pixelformat can be used
    ret = v4l2request_try_format(hwfc, fctxi->output.format.type, pixelformat);
    if (ret < 0) {
        av_log(hwfc, AV_LOG_VERBOSE, "Device %s is missing support for pixelformat %s\n",
               path, av_fourcc2str(pixelformat));
        goto fail;
    }

    // Ensure frame size is supported, when driver support ENUM_FRAMESIZES
    ret = v4l2request_try_framesize(hwfc, pixelformat);
    if (ret < 0 && ret != AVERROR(ENOTTY)) {
        av_log(hwfc, AV_LOG_VERBOSE,
               "Device %s is missing support for frame size %dx%d of pixelformat %s\n",
               path, hwfc->width, hwfc->height, av_fourcc2str(pixelformat));
        goto fail;
    }

    // Set the codec pixelformat and OUTPUT buffersize to be used
    ret = v4l2request_set_format(hwfc, fctxi->output.format.type, pixelformat, buffersize);
    if (ret < 0) {
        av_log(hwfc, AV_LOG_ERROR,
               "Failed to set OUTPUT pixelformat %s of %s: %s (%d)\n",
               av_fourcc2str(pixelformat), path, strerror(errno), errno);
        goto fail;
    }

    // Get format details for OUTPUT buffers
    if (ioctl(fctxi->video_fd, VIDIOC_G_FMT, &fctxi->output.format) < 0) {
        ret = AVERROR(errno);
        av_log(hwfc, AV_LOG_ERROR, "Failed to get OUTPUT format: %s (%d)\n",
               strerror(errno), errno);
        goto fail;
    }

    /*
     * Set any codec specific controls that can help assist the driver
     * make a decision on what CAPTURE buffer format can be used.
     */
    ret = v4l2request_set_controls(hwfc, fctx->init_controls, fctx->nb_init_controls);
    if (ret < 0) {
        av_log(hwfc, AV_LOG_VERBOSE,
               "Failed to set %d control(s): %s (%d)\n",
               fctx->nb_init_controls, strerror(errno), errno);
        goto fail;
    }

    // Select a supported CAPTURE buffer format
    ret = v4l2request_select_capture_format(hwfc);
    if (ret < 0) {
        av_log(hwfc, AV_LOG_VERBOSE,
               "Failed to select a CAPTURE format %s of %s: %s (%d)\n",
               av_fourcc2str(pixelformat), path, strerror(errno), errno);
        goto fail;
    }

    // Query CAPTURE buffer capabilities
    buffers = (struct v4l2_create_buffers) {
        .count = 0,
        .memory = V4L2_MEMORY_MMAP,
        .format.type = fctxi->capture.format.type,
    };
    if (ioctl(fctxi->video_fd, VIDIOC_CREATE_BUFS, &buffers) < 0) {
        ret = AVERROR(errno);
        av_log(hwfc, AV_LOG_ERROR,
               "Failed to query CAPTURE buffer capabilities of %s: %s (%d)\n",
               path, strerror(errno), errno);
        goto fail;
    }
    fctxi->capture.capabilities = buffers.capabilities;

    // Get format details for CAPTURE buffers
    if (ioctl(fctxi->video_fd, VIDIOC_G_FMT, &fctxi->capture.format) < 0) {
        ret = AVERROR(errno);
        av_log(hwfc, AV_LOG_ERROR, "Failed to get CAPTURE format: %s (%d)\n",
               strerror(errno), errno);
        goto fail;
    }

    // All tests passed, video device should be capable
    return 0;

fail:
    if (fctxi->video_fd >= 0) {
        close(fctxi->video_fd);
        fctxi->video_fd = -1;
    }
    return ret;
}

static int v4l2request_probe_video_devices(AVHWFramesContext *hwfc,
                                           struct udev *udev,
                                           uint32_t pixelformat,
                                           uint32_t buffersize)
{
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;
    struct media_device_info device_info;
    struct media_v2_topology topology = {0};
    struct media_v2_interface *interfaces;
    struct udev_device *device;
    const char *path;
    dev_t devnum;
    int ret;

    if (ioctl(fctxi->media_fd, MEDIA_IOC_DEVICE_INFO, &device_info) < 0)
        return AVERROR(errno);

    if (ioctl(fctxi->media_fd, MEDIA_IOC_G_TOPOLOGY, &topology) < 0) {
        ret = AVERROR(errno);
        av_log(hwfc, AV_LOG_ERROR, "Failed to get media topology: %s (%d)\n",
               strerror(errno), errno);
        return ret;
    }

    if (!topology.num_interfaces)
        return AVERROR(ENOENT);

    interfaces = av_calloc(topology.num_interfaces, sizeof(struct media_v2_interface));
    if (!interfaces)
        return AVERROR(ENOMEM);

    topology.ptr_interfaces = (__u64)(uintptr_t)interfaces;
    if (ioctl(fctxi->media_fd, MEDIA_IOC_G_TOPOLOGY, &topology) < 0) {
        ret = AVERROR(errno);
        av_log(hwfc, AV_LOG_ERROR, "Failed to get media topology: %s (%d)\n",
               strerror(errno), errno);
        goto fail;
    }

    ret = AVERROR(ENOENT);
    for (int i = 0; i < topology.num_interfaces; i++) {
        if (interfaces[i].intf_type != MEDIA_INTF_T_V4L_VIDEO)
            continue;

        devnum = makedev(interfaces[i].devnode.major, interfaces[i].devnode.minor);
        device = udev_device_new_from_devnum(udev, 'c', devnum);
        if (!device)
            continue;

        path = udev_device_get_devnode(device);
        if (path)
            ret = v4l2request_probe_video_device(hwfc, path, pixelformat, buffersize);
        udev_device_unref(device);

        // Stop when we have found a capable video device
        if (!ret) {
            av_log(hwfc, AV_LOG_INFO,
                   "Using V4L2 media driver %s (%u.%u.%u) for %s\n",
                   device_info.driver,
                   device_info.driver_version >> 16,
                   (device_info.driver_version >> 8) & 0xff,
                   device_info.driver_version & 0xff,
                   av_fourcc2str(pixelformat));
            break;
        }
    }

fail:
    av_free(interfaces);
    return ret;
}

static int v4l2request_probe_media_device(AVHWFramesContext *hwfc,
                                          struct udev_device *device,
                                          uint32_t pixelformat,
                                          uint32_t buffersize)
{
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;
    const char *path;
    int ret;

    path = udev_device_get_devnode(device);
    if (!path)
        return AVERROR(ENODEV);

    // Open enumerated media device
    fctxi->media_fd = open(path, O_RDWR);
    if (fctxi->media_fd < 0) {
        ret = AVERROR(errno);
        av_log(hwfc, AV_LOG_ERROR, "Failed to open media device %s: %s (%d)\n",
               path, strerror(errno), errno);
        return ret;
    }

    // Probe video devices of current media device
    ret = v4l2request_probe_video_devices(hwfc, udev_device_get_udev(device),
                                          pixelformat, buffersize);

    // Cleanup when no capable video device was found
    if (ret < 0) {
        close(fctxi->media_fd);
        fctxi->media_fd = -1;
    }

    return ret;
}

static int v4l2request_probe_media_devices(AVHWFramesContext *hwfc,
                                           struct udev *udev,
                                           uint32_t pixelformat,
                                           uint32_t buffersize)
{
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices;
    struct udev_list_entry *entry;
    struct udev_device *device;
    int ret;

    enumerate = udev_enumerate_new(udev);
    if (!enumerate)
        return AVERROR(ENOMEM);

    udev_enumerate_add_match_subsystem(enumerate, "media");
    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);

    ret = AVERROR(ENOENT);
    udev_list_entry_foreach(entry, devices) {
        const char *path = udev_list_entry_get_name(entry);
        if (!path)
            continue;

        device = udev_device_new_from_syspath(udev, path);
        if (!device)
            continue;

        // Probe media device for a capable video device
        ret = v4l2request_probe_media_device(hwfc, device, pixelformat, buffersize);
        udev_device_unref(device);

        // Stop when we have found a capable media and video device
        if (!ret)
            break;
    }

    udev_enumerate_unref(enumerate);
    return ret;
}

static int v4l2request_open_decoder(AVHWFramesContext *hwfc)
{
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    uint32_t buffersize;
    struct udev *udev;
    int ret;

    // Ensure codec pixelformat is set
    if (!fctx->pixelformat)
        return AVERROR(EINVAL);

    // FIXME: locate a decoder using hwdevice context decoders

    udev = udev_new();
    if (!udev)
        return AVERROR(ENOMEM);

    buffersize = FFMAX(hwfc->width * hwfc->height * 3 / 2, 256 * 1024);

    // Probe all media devices (auto-detection)
    ret = v4l2request_probe_media_devices(hwfc, udev, fctx->pixelformat, buffersize);

    udev_unref(udev);
    return ret;
}

static AVBufferRef *v4l2request_v4l2_buffer_alloc(AVHWFramesContext *hwfc,
                                                  struct v4l2_format *format)
{
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;
    struct v4l2_create_buffers buffers = {
        .count = 1,
        .memory = V4L2_MEMORY_MMAP,
        .format = *format,
    };
    struct v4l2_buffer *buffer;
    uint8_t num_planes;
    AVBufferRef *ref;

    num_planes = V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                 format->fmt.pix_mp.num_planes : 0;

    ref = av_buffer_allocz(sizeof(struct v4l2_buffer) +
                           (sizeof(struct v4l2_plane) * num_planes));
    if (!ref)
        return NULL;

    buffer = (struct v4l2_buffer *)ref->data;
    buffer->type = format->type;

    if (num_planes) {
        buffer->length = num_planes;
        buffer->m.planes = (struct v4l2_plane *)(buffer + 1);
    }

    // Create the buffer
    if (ioctl(fctxi->video_fd, VIDIOC_CREATE_BUFS, &buffers) < 0) {
        av_log(hwfc, AV_LOG_ERROR, "Failed to create buffer of type %d: %s (%d)\n",
               buffer->type, strerror(errno), errno);
        goto fail;
    }

    buffer->memory = buffers.memory;
    buffer->index = buffers.index;

    // Query more details of the created buffer
    if (ioctl(fctxi->video_fd, VIDIOC_QUERYBUF, buffer) < 0) {
        av_log(hwfc, AV_LOG_ERROR, "Failed to query buffer %d of type %d: %s (%d)\n",
               buffer->index, buffer->type, strerror(errno), errno);
        goto fail;
    }

    return ref;

fail:
    av_buffer_unref(&ref);
    return NULL;
}

static AVBufferRef *v4l2request_capture_buffer_alloc(void *opaque, size_t size)
{
    AVHWFramesContext *hwfc = opaque;
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;

    return v4l2request_v4l2_buffer_alloc(hwfc, &fctxi->capture.format);
}

static AVBufferRef *v4l2request_output_buffer_alloc(void *opaque, size_t size)
{
    AVHWFramesContext *hwfc = opaque;
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;

    return v4l2request_v4l2_buffer_alloc(hwfc, &fctxi->output.format);
}

static void v4l2request_frame_free(void *opaque, uint8_t *data)
{
    V4L2RequestFrameDescriptor *desc = (V4L2RequestFrameDescriptor *)data;

    // Close the exported CAPTURE buffer memory planes
    for (int i = 0; i < FF_ARRAY_ELEMS(desc->fd); i++) {
        if (desc->fd[i] >= 0) {
            close(desc->fd[i]);
            desc->fd[i] = -1;
        }
    }

    // Return the CAPTURE buffer to the frames context CAPTURE pool
    av_buffer_unref(&desc->ref);

    av_free(data);
}

static AVBufferRef *v4l2request_frame_alloc(void *opaque, size_t size)
{
    AVHWFramesContext *hwfc = opaque;
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;
    struct v4l2_format *format = &fctxi->capture.format;
    V4L2RequestFrameDescriptor *desc;
    struct v4l2_buffer *buffer;
    AVBufferRef *ref;
    uint8_t *data;

    data = av_mallocz(size);
    if (!data)
        return NULL;

    ref = av_buffer_create(data, size, v4l2request_frame_free,
                           hwfc, AV_BUFFER_FLAG_READONLY);
    if (!ref) {
        av_free(data);
        return NULL;
    }

    // Set initial default values
    desc = (V4L2RequestFrameDescriptor *)data;
    for (int i = 0; i < FF_ARRAY_ELEMS(desc->fd); i++)
        desc->fd[i] = -1;

    // Get a CAPTURE buffer from frames context CAPTURE pool
    desc->ref = av_buffer_pool_get(fctxi->capture.pool);
    if (!desc->ref)
       goto fail;

    buffer = (struct v4l2_buffer *)desc->ref->data;
    desc->index = buffer->index;

    // Export CAPTURE buffer memory planes
    desc->base.nb_objects = V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                            format->fmt.pix_mp.num_planes : 1;
    av_assert0(desc->base.nb_objects <= AV_DRM_MAX_PLANES);
    for (int i = 0; i < desc->base.nb_objects; i++) {
        struct v4l2_exportbuffer exportbuffer = {
            .type = buffer->type,
            .index = buffer->index,
            .plane = i,
            .flags = O_RDONLY,
        };
        if (ioctl(fctxi->video_fd, VIDIOC_EXPBUF, &exportbuffer) < 0) {
            av_log(hwfc, AV_LOG_ERROR, "Failed to export memory plane %d (%d): %s (%d)\n",
                   i, buffer->index, strerror(errno), errno);
            goto fail;
        }
        desc->base.objects[i].fd = desc->fd[i] = exportbuffer.fd;
    }

    // Set AVDRMFrameDescriptor based on CAPTURE buffer format
    if (v4l2request_set_drm_descriptor(&desc->base, format) < 0)
        goto fail;

    return ref;

fail:
    av_buffer_unref(&ref);
    return NULL;
}

static int v4l2request_frames_init(AVHWFramesContext *hwfc)
{
    V4L2RequestFramesContext *hwctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi;
    uint32_t pixelformat;
    int ret;

    // Set initial default values
    fctxi = &hwctx->internal;
    hwctx->p.internal = fctxi;
    fctxi->media_fd = -1;
    fctxi->video_fd = -1;

    // Locate and open a capable video decoder device
    ret = v4l2request_open_decoder(hwfc);
    if (ret < 0)
        return ret;

    // Reset init controls after video device is opened
    hwctx->p.init_controls = NULL;
    hwctx->p.nb_init_controls = 0;

    // Update frames context with CAPTURE format details
    if (V4L2_TYPE_IS_MULTIPLANAR(fctxi->capture.format.type)) {
        hwfc->width = fctxi->capture.format.fmt.pix_mp.width;
        hwfc->height = fctxi->capture.format.fmt.pix_mp.height;
        pixelformat = fctxi->capture.format.fmt.pix_mp.pixelformat;
    } else {
        hwfc->width = fctxi->capture.format.fmt.pix.width;
        hwfc->height = fctxi->capture.format.fmt.pix.height;
        pixelformat = fctxi->capture.format.fmt.pix.pixelformat;
    }

    hwfc->sw_format = AV_PIX_FMT_NONE;
    for (int i = 0; i < FF_ARRAY_ELEMS(v4l2request_capture_pixelformats); i++) {
        if (pixelformat == v4l2request_capture_pixelformats[i].pixelformat) {
            hwctx->p.bit_depth = v4l2request_capture_pixelformats[i].bit_depth;
            hwfc->sw_format = v4l2request_capture_pixelformats[i].sw_format;
            break;
        }
    }

    // Initialize buffer pool for CAPTURE buffers
    fctxi->capture.pool = av_buffer_pool_init2(sizeof(struct v4l2_buffer), hwfc,
                                               v4l2request_capture_buffer_alloc, NULL);
    if (!fctxi->capture.pool)
        return AVERROR(ENOMEM);

    // Initialize buffer pool for OUTPUT buffers
    fctxi->output.pool = av_buffer_pool_init2(sizeof(struct v4l2_buffer), hwfc,
                                              v4l2request_output_buffer_alloc, NULL);
    if (!fctxi->output.pool)
        return AVERROR(ENOMEM);

    // Initialize buffer pool for frame descriptors
    ffhwframesctx(hwfc)->pool_internal =
                av_buffer_pool_init2(sizeof(V4L2RequestFrameDescriptor), hwfc,
                                     v4l2request_frame_alloc, NULL);
    if (!ffhwframesctx(hwfc)->pool_internal)
        return AVERROR(ENOMEM);

    av_log(hwfc, AV_LOG_VERBOSE, "Using CAPTURE buffer format %s (%dx%d)\n",
           av_fourcc2str(pixelformat), hwfc->width, hwfc->height);

    return 0;
}

static void v4l2request_frames_uninit(AVHWFramesContext *hwfc)
{
    AVV4L2RequestFramesContext *fctx = hwfc->hwctx;
    AVV4L2RequestFramesContextInternal *fctxi = fctx->internal;

    av_buffer_pool_uninit(&fctxi->capture.pool);
    av_buffer_pool_uninit(&fctxi->output.pool);

    if (fctxi->video_fd >= 0) {
        close(fctxi->video_fd);
        fctxi->video_fd = -1;
    }

    if (fctxi->media_fd) {
        close(fctxi->media_fd);
        fctxi->media_fd = -1;
    }
}

static int v4l2request_get_buffer(AVHWFramesContext *hwfc, AVFrame *frame)
{
    V4L2RequestFrameDescriptor *desc;

    frame->buf[0] = av_buffer_pool_get(hwfc->pool);
    if (!frame->buf[0])
        return AVERROR(ENOMEM);

    desc = (V4L2RequestFrameDescriptor *)frame->buf[0]->data;
    frame->data[0] = (uint8_t *)&desc->base;
    frame->data[1] = (uint8_t *)(uintptr_t)desc->index;

    frame->format = AV_PIX_FMT_DRM_PRIME;
    frame->width  = hwfc->width;
    frame->height = hwfc->height;

    return 0;
}

typedef struct V4L2RequestMapping {
    // Address and length of each mmap()ed region.
    int nb_regions;
    int object[AV_DRM_MAX_PLANES];
    void *address[AV_DRM_MAX_PLANES];
    size_t length[AV_DRM_MAX_PLANES];
} V4L2RequestMapping;

static void v4l2request_unmap_frame(AVHWFramesContext *hwfc,
                                    HWMapDescriptor *hwmap)
{
    V4L2RequestMapping *map = hwmap->priv;

    for (int i = 0; i < map->nb_regions; i++) {
        struct dma_buf_sync sync = {
            .flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ,
        };
        ioctl(map->object[i], DMA_BUF_IOCTL_SYNC, &sync);
        munmap(map->address[i], map->length[i]);
    }

    av_free(map);
}

static int v4l2request_map_frame(AVHWFramesContext *hwfc,
                                 AVFrame *dst, const AVFrame *src)
{
    const AVDRMFrameDescriptor *desc = (AVDRMFrameDescriptor *)src->data[0];
    struct dma_buf_sync sync = {
        .flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ,
    };
    V4L2RequestMapping *map;
    int ret, i, p, plane;
    void *addr;

    map = av_mallocz(sizeof(*map));
    if (!map)
        return AVERROR(ENOMEM);

    av_assert0(desc->nb_objects <= AV_DRM_MAX_PLANES);
    for (i = 0; i < desc->nb_objects; i++) {
        addr = mmap(NULL, desc->objects[i].size, PROT_READ, MAP_SHARED,
                    desc->objects[i].fd, 0);
        if (addr == MAP_FAILED) {
            ret = AVERROR(errno);
            av_log(hwfc, AV_LOG_ERROR, "Failed to map DRM object %d to memory: %s (%d)\n",
                   desc->objects[i].fd, strerror(errno), errno);
            goto fail;
        }

        map->address[i] = addr;
        map->length[i]  = desc->objects[i].size;
        map->object[i]  = desc->objects[i].fd;

        /*
         * We're not checking for errors here because the kernel may not
         * support the ioctl, in which case its okay to carry on
         */
        ioctl(desc->objects[i].fd, DMA_BUF_IOCTL_SYNC, &sync);
    }
    map->nb_regions = i;

    plane = 0;
    for (i = 0; i < desc->nb_layers; i++) {
        const AVDRMLayerDescriptor *layer = &desc->layers[i];
        for (p = 0; p < layer->nb_planes; p++) {
            dst->data[plane] =
                (uint8_t *)map->address[layer->planes[p].object_index] +
                                        layer->planes[p].offset;
            dst->linesize[plane] =      layer->planes[p].pitch;
            ++plane;
        }
    }
    av_assert0(plane <= AV_DRM_MAX_PLANES);

    dst->width  = src->width;
    dst->height = src->height;

    ret = ff_hwframe_map_create(src->hw_frames_ctx, dst, src,
                                v4l2request_unmap_frame, map);
    if (ret < 0)
        goto fail;

    return 0;

fail:
    for (i = 0; i < desc->nb_objects; i++) {
        if (map->address[i])
            munmap(map->address[i], map->length[i]);
    }
    av_free(map);
    return ret;
}

static int v4l2request_transfer_get_formats(AVHWFramesContext *hwfc,
                                            enum AVHWFrameTransferDirection dir,
                                            enum AVPixelFormat **formats)
{
    enum AVPixelFormat *fmts;

    if (dir == AV_HWFRAME_TRANSFER_DIRECTION_TO)
        return AVERROR(ENOSYS);

    fmts = av_malloc_array(2, sizeof(*fmts));
    if (!fmts)
        return AVERROR(ENOMEM);

    fmts[0] = hwfc->sw_format;
    fmts[1] = AV_PIX_FMT_NONE;

    if (hwfc->sw_format == AV_PIX_FMT_YUV420P ||
        hwfc->sw_format == AV_PIX_FMT_YUV420P10 ||
        hwfc->sw_format == AV_PIX_FMT_YUV422P10)
        fmts[0] = AV_PIX_FMT_NONE;

    *formats = fmts;
    return 0;
}

static int v4l2request_transfer_data_from(AVHWFramesContext *hwfc,
                                          AVFrame *dst, const AVFrame *src)
{
    AVFrame *map;
    int ret;

    if (dst->width > hwfc->width || dst->height > hwfc->height)
        return AVERROR(EINVAL);

    map = av_frame_alloc();
    if (!map)
        return AVERROR(ENOMEM);
    map->format = dst->format;

    ret = v4l2request_map_frame(hwfc, map, src);
    if (ret)
        goto fail;

    map->width  = dst->width;
    map->height = dst->height;

    ret = av_frame_copy(dst, map);
    if (ret)
        goto fail;

    ret = 0;
fail:
    av_frame_free(&map);
    return ret;
}

static int v4l2request_map_from(AVHWFramesContext *hwfc, AVFrame *dst,
                                const AVFrame *src, int flags)
{
    int ret;

    if (!(flags & AV_HWFRAME_MAP_READ))
        return AVERROR(ENOSYS);

    if (hwfc->sw_format == AV_PIX_FMT_NONE ||
        hwfc->sw_format == AV_PIX_FMT_YUV420P ||
        hwfc->sw_format == AV_PIX_FMT_YUV420P10 ||
        hwfc->sw_format == AV_PIX_FMT_YUV422P10)
        return AVERROR(ENOSYS);
    else if (dst->format == AV_PIX_FMT_NONE)
        dst->format = hwfc->sw_format;
    else if (hwfc->sw_format != dst->format)
        return AVERROR(ENOSYS);

    ret = v4l2request_map_frame(hwfc, dst, src);
    if (ret)
        return ret;

    return av_frame_copy_props(dst, src);
}

const HWContextType ff_hwcontext_type_v4l2request = {
    .type                   = AV_HWDEVICE_TYPE_V4L2REQUEST,
    .name                   = "V4L2 Request API",

    .device_hwctx_size      = sizeof(V4L2RequestDeviceContext),
    .device_create          = v4l2request_device_create,
    .device_uninit          = v4l2request_device_uninit,

    .frames_hwctx_size      = sizeof(V4L2RequestFramesContext),
    .frames_init            = v4l2request_frames_init,
    .frames_uninit          = v4l2request_frames_uninit,
    .frames_get_buffer      = v4l2request_get_buffer,
    .transfer_get_formats   = v4l2request_transfer_get_formats,
    .transfer_data_from     = v4l2request_transfer_data_from,
    .map_from               = v4l2request_map_from,

    .pix_fmts = (const enum AVPixelFormat[]) {
        AV_PIX_FMT_DRM_PRIME,
        AV_PIX_FMT_NONE
    },
};
