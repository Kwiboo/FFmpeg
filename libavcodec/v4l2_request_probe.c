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
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include <drm_fourcc.h>
#include <libudev.h>

#include "libavutil/hwcontext_v4l2request.h"
#include "v4l2_request_internal.h"

static const struct {
    uint32_t pixelformat;
    enum AVPixelFormat sw_format;
    uint32_t drm_format;
    uint64_t format_modifier;
} v4l2_request_capture_pixelformats[] = {
    { V4L2_PIX_FMT_NV12, AV_PIX_FMT_NV12, DRM_FORMAT_NV12, DRM_FORMAT_MOD_LINEAR },
    { V4L2_PIX_FMT_SUNXI_TILED_NV12, AV_PIX_FMT_NV12, DRM_FORMAT_NV12, DRM_FORMAT_MOD_ALLWINNER_TILED },
#if defined(V4L2_PIX_FMT_NV15) && defined(DRM_FORMAT_NV15)
    { V4L2_PIX_FMT_NV15, AV_PIX_FMT_NONE, DRM_FORMAT_NV15, DRM_FORMAT_MOD_LINEAR },
#endif
    { V4L2_PIX_FMT_NV16, AV_PIX_FMT_NV16, DRM_FORMAT_NV16, DRM_FORMAT_MOD_LINEAR },
#if defined(V4L2_PIX_FMT_NV20) && defined(DRM_FORMAT_NV20)
    { V4L2_PIX_FMT_NV20, AV_PIX_FMT_NONE, DRM_FORMAT_NV20, DRM_FORMAT_MOD_LINEAR },
#endif
    { V4L2_PIX_FMT_P010, AV_PIX_FMT_P010, DRM_FORMAT_P010, DRM_FORMAT_MOD_LINEAR },
#if defined(V4L2_PIX_FMT_YUV420_10_AFBC_16X16_SPLIT)
    {
        .pixelformat = V4L2_PIX_FMT_YUV420_10_AFBC_16X16_SPLIT,
        .sw_format = AV_PIX_FMT_NONE,
        .drm_format = DRM_FORMAT_YUV420_10BIT,
        .format_modifier = DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
                                                   AFBC_FORMAT_MOD_SPARSE |
                                                   AFBC_FORMAT_MOD_SPLIT),
    },
#endif
#if defined(V4L2_PIX_FMT_YUV420_8_AFBC_16X16_SPLIT)
    {
        .pixelformat = V4L2_PIX_FMT_YUV420_8_AFBC_16X16_SPLIT,
        .sw_format = AV_PIX_FMT_NONE,
        .drm_format = DRM_FORMAT_YUV420_8BIT,
        .format_modifier = DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
                                                   AFBC_FORMAT_MOD_SPARSE |
                                                   AFBC_FORMAT_MOD_SPLIT),
    },
#endif
};

enum AVPixelFormat ff_v4l2_request_get_sw_format(struct v4l2_format *format)
{
    uint32_t pixelformat = V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                           format->fmt.pix_mp.pixelformat :
                           format->fmt.pix.pixelformat;

    for (int i = 0; i < FF_ARRAY_ELEMS(v4l2_request_capture_pixelformats); i++) {
        if (pixelformat == v4l2_request_capture_pixelformats[i].pixelformat)
            return v4l2_request_capture_pixelformats[i].sw_format;
    }

    return AV_PIX_FMT_NONE;
}

int ff_v4l2_request_set_drm_descriptor(V4L2RequestFrameDescriptor *framedesc,
                                       struct v4l2_format *format)
{
    AVDRMFrameDescriptor *desc = &framedesc->base;
    AVDRMLayerDescriptor *layer = &desc->layers[0];
    uint32_t pixelformat = V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                           format->fmt.pix_mp.pixelformat :
                           format->fmt.pix.pixelformat;

    layer->format = 0;
    for (int i = 0; i < FF_ARRAY_ELEMS(v4l2_request_capture_pixelformats); i++) {
        if (pixelformat == v4l2_request_capture_pixelformats[i].pixelformat) {
            layer->format = v4l2_request_capture_pixelformats[i].drm_format;
            desc->objects[0].format_modifier =
                        v4l2_request_capture_pixelformats[i].format_modifier;
            break;
        }
    }

    if (!layer->format)
        return -1;

    desc->nb_objects = 1;
    desc->objects[0].fd = framedesc->capture.fd;
    desc->objects[0].size = framedesc->capture.size;

    desc->nb_layers = 1;
    layer->nb_planes = 1;

    layer->planes[0].object_index = 0;
    layer->planes[0].offset = 0;
    layer->planes[0].pitch = V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                             format->fmt.pix_mp.plane_fmt[0].bytesperline :
                             format->fmt.pix.bytesperline;

    if (!fourcc_mod_is_vendor(desc->objects[0].format_modifier, ARM)) {
        layer->nb_planes = 2;
        layer->planes[1].object_index = 0;
        layer->planes[1].offset = layer->planes[0].pitch *
                                  (V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
                                   format->fmt.pix_mp.height :
                                   format->fmt.pix.height);
        layer->planes[1].pitch = layer->planes[0].pitch;
    }

    return 0;
}

static int v4l2_request_set_format(AVCodecContext *avctx,
                                   enum v4l2_buf_type type,
                                   uint32_t pixelformat,
                                   uint32_t buffersize)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    struct v4l2_format format = {
        .type = type,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
        format.fmt.pix_mp.width = avctx->coded_width;
        format.fmt.pix_mp.height = avctx->coded_height;
        format.fmt.pix_mp.pixelformat = pixelformat;
        format.fmt.pix_mp.plane_fmt[0].sizeimage = buffersize;
        format.fmt.pix_mp.num_planes = 1;
    } else {
        format.fmt.pix.width = avctx->coded_width;
        format.fmt.pix.height = avctx->coded_height;
        format.fmt.pix.pixelformat = pixelformat;
        format.fmt.pix.sizeimage = buffersize;
    }

    return ioctl(ctx->video_fd, VIDIOC_S_FMT, &format);
}

static int v4l2_request_select_capture_format(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    enum v4l2_buf_type type = ctx->format.type;

#if 1
    struct v4l2_format format = {
        .type = type,
    };
    struct v4l2_fmtdesc fmtdesc = {
        .index = 0,
        .type = type,
    };
    uint32_t pixelformat;

    if (ioctl(ctx->video_fd, VIDIOC_G_FMT, &format) < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get capture format failed, %s (%d)\n", __func__, strerror(errno), errno);
        return -1;
    }

    pixelformat = V4L2_TYPE_IS_MULTIPLANAR(type) ?
                  format.fmt.pix_mp.pixelformat :
                  format.fmt.pix.pixelformat;

    for (int i = 0; i < FF_ARRAY_ELEMS(v4l2_request_capture_pixelformats); i++) {
        if (pixelformat == v4l2_request_capture_pixelformats[i].pixelformat)
            return v4l2_request_set_format(avctx, type, pixelformat, 0);
    }

    while (ioctl(ctx->video_fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0) {
        for (int i = 0; i < FF_ARRAY_ELEMS(v4l2_request_capture_pixelformats); i++) {
            if (fmtdesc.pixelformat == v4l2_request_capture_pixelformats[i].pixelformat)
                return v4l2_request_set_format(avctx, type, fmtdesc.pixelformat, 0);
        }

        fmtdesc.index++;
    }
#else
    for (int i = 0; i < FF_ARRAY_ELEMS(v4l2_request_capture_pixelformats); i++) {
        uint32_t pixelformat = v4l2_request_capture_pixelformats[i].pixelformat;
        if (!v4l2_request_try_format(avctx, type, pixelformat))
            return v4l2_request_set_format(avctx, type, pixelformat, 0);
    }
#endif

    return -1;
}

static int v4l2_request_try_framesize(AVCodecContext *avctx,
                                      uint32_t pixelformat)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    struct v4l2_frmsizeenum frmsize = {
        .index = 0,
        .pixel_format = pixelformat,
    };

    if (ioctl(ctx->video_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) < 0)
        return 0;

    /*
     * We only validate min/max framesize for V4L2_FRMSIZE_TYPE_STEPWISE here, since the alignment
     * which is eventually needed will be done driver-side later in VIDIOC_S_FMT and there is no need
     * validate step_width/step_height here
     */
    do {
        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE && frmsize.discrete.width == avctx->coded_width &&
            frmsize.discrete.height == avctx->coded_height)
            return 0;
        else if ((frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE || frmsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) &&
                 avctx->coded_width >= frmsize.stepwise.min_width && avctx->coded_height >= frmsize.stepwise.min_height &&
                 avctx->coded_width <= frmsize.stepwise.max_width && avctx->coded_height <= frmsize.stepwise.max_height)
            return 0;

        frmsize.index++;
    } while (ioctl(ctx->video_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) >= 0);

    av_log(avctx, AV_LOG_INFO, "%s: pixelformat %s not supported for width %u height %u\n", __func__, av_fourcc2str(pixelformat), avctx->coded_width, avctx->coded_height);
    return -1;
}

static int v4l2_request_try_format(AVCodecContext *avctx,
                                   enum v4l2_buf_type type,
                                   uint32_t pixelformat)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    struct v4l2_fmtdesc fmtdesc = {
        .index = 0,
        .type = type,
    };

    if (V4L2_TYPE_IS_OUTPUT(type)) {
        struct v4l2_create_buffers buffers = {
            .count = 0,
            .memory = V4L2_MEMORY_MMAP,
            .format.type = type,
        };

        if (ioctl(ctx->video_fd, VIDIOC_CREATE_BUFS, &buffers) < 0) {
            av_log(avctx, AV_LOG_ERROR, "%s: create buffers failed for type %u, %s (%d)\n", __func__, type, strerror(errno), errno);
            return -1;
        }

        if ((buffers.capabilities & V4L2_BUF_CAP_SUPPORTS_REQUESTS) != V4L2_BUF_CAP_SUPPORTS_REQUESTS) {
            av_log(avctx, AV_LOG_INFO, "%s: output buffer type do not support requests, capabilities %u\n", __func__, buffers.capabilities);
            return -1;
        }
    }

    while (ioctl(ctx->video_fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0) {
        if (fmtdesc.pixelformat == pixelformat)
            return 0;

        fmtdesc.index++;
    }

    av_log(avctx, AV_LOG_DEBUG, "%s: pixelformat %s not supported for type %u\n", __func__, av_fourcc2str(pixelformat), type);
    return -1;
}

static int v4l2_request_probe_video_device(struct udev_device *device,
                                           AVCodecContext *avctx,
                                           uint32_t pixelformat,
                                           uint32_t buffersize,
                                           struct v4l2_ext_control *control,
                                           int count)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    int ret = AVERROR(EINVAL);
    struct v4l2_capability capability = {0};
    unsigned int capabilities = 0;

    const char *path = udev_device_get_devnode(device);
    if (!path) {
        av_log(avctx, AV_LOG_ERROR, "%s: get video device devnode failed\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ctx->video_fd = open(path, O_RDWR | O_NONBLOCK, 0);
    if (ctx->video_fd < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: opening %s failed, %s (%d)\n", __func__, path, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = ioctl(ctx->video_fd, VIDIOC_QUERYCAP, &capability);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get video capability failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if (capability.capabilities & V4L2_CAP_DEVICE_CAPS)
        capabilities = capability.device_caps;
    else
        capabilities = capability.capabilities;

    av_log(avctx, AV_LOG_DEBUG, "%s: ctx=%p path=%s capabilities=%u\n", __func__, ctx, path, capabilities);

    if ((capabilities & V4L2_CAP_STREAMING) != V4L2_CAP_STREAMING) {
        av_log(avctx, AV_LOG_ERROR, "%s: missing required streaming capability\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if ((capabilities & V4L2_CAP_VIDEO_M2M_MPLANE) == V4L2_CAP_VIDEO_M2M_MPLANE) {
        ctx->output_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        ctx->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    } else if ((capabilities & V4L2_CAP_VIDEO_M2M) == V4L2_CAP_VIDEO_M2M) {
        ctx->output_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        ctx->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    } else {
        av_log(avctx, AV_LOG_ERROR, "%s: missing required mem2mem capability\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = v4l2_request_try_format(avctx, ctx->output_type, pixelformat);
    if (ret < 0) {
        av_log(avctx, AV_LOG_DEBUG, "%s: try output format failed\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = v4l2_request_try_framesize(avctx, pixelformat);
    if (ret < 0) {
        av_log(avctx, AV_LOG_DEBUG, "%s: try framesize failed\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = v4l2_request_set_format(avctx, ctx->output_type, pixelformat, buffersize);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: set output format failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = ff_v4l2_request_set_controls(avctx, control, count);
    if (ret < 0) {
        av_log(avctx, AV_LOG_DEBUG, "%s: set controls failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = v4l2_request_select_capture_format(avctx);
    if (ret < 0) {
        av_log(avctx, AV_LOG_DEBUG, "%s: select capture format failed\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if (ctx->post_probe) {
        ret = ctx->post_probe(avctx);
        if (ret < 0)
            goto fail;
    }

    return 0;

fail:
    if (ctx->video_fd >= 0) {
        close(ctx->video_fd);
        ctx->video_fd = -1;
    }
    return ret;
}

int ff_v4l2_request_probe_media_device(struct udev_device *device, AVCodecContext *avctx, uint32_t pixelformat, uint32_t buffersize, struct v4l2_ext_control *control, int count)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    int ret;
    struct media_device_info device_info = {0};
    struct media_v2_topology topology = {0};
    struct media_v2_interface *interfaces = NULL;
    struct udev *udev = udev_device_get_udev(device);
    struct udev_device *video_device;
    dev_t devnum;

    const char *path = udev_device_get_devnode(device);
    if (!path) {
        av_log(avctx, AV_LOG_ERROR, "%s: get media device devnode failed\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ctx->media_fd = open(path, O_RDWR, 0);
    if (ctx->media_fd < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: opening %s failed, %s (%d)\n", __func__, path, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = ioctl(ctx->media_fd, MEDIA_IOC_DEVICE_INFO, &device_info);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get media device info failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    av_log(avctx, AV_LOG_DEBUG, "%s: ctx=%p path=%s driver=%s\n", __func__, ctx, path, device_info.driver);

    ret = ioctl(ctx->media_fd, MEDIA_IOC_G_TOPOLOGY, &topology);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get media topology failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if (topology.num_interfaces <= 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: media device has no interfaces\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    interfaces = av_mallocz(topology.num_interfaces * sizeof(struct media_v2_interface));
    if (!interfaces) {
        av_log(avctx, AV_LOG_ERROR, "%s: allocating media interface struct failed\n", __func__);
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    topology.ptr_interfaces = (__u64)(uintptr_t)interfaces;
    ret = ioctl(ctx->media_fd, MEDIA_IOC_G_TOPOLOGY, &topology);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get media topology failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = AVERROR(EINVAL);
    for (int i = 0; i < topology.num_interfaces; i++) {
        if (interfaces[i].intf_type != MEDIA_INTF_T_V4L_VIDEO)
            continue;

        devnum = makedev(interfaces[i].devnode.major, interfaces[i].devnode.minor);
        video_device = udev_device_new_from_devnum(udev, 'c', devnum);
        if (!video_device) {
            av_log(avctx, AV_LOG_ERROR, "%s: video_device=%p\n", __func__, video_device);
            continue;
        }

        ret = v4l2_request_probe_video_device(video_device, avctx, pixelformat, buffersize, control, count);
        udev_device_unref(video_device);

        if (!ret) {
            av_freep(&interfaces);
            av_log(avctx, AV_LOG_INFO, "Using V4L2 media device %s (%s) for %s\n", path, device_info.driver, av_fourcc2str(pixelformat));
            return 0;
        }
    }

fail:
    av_freep(&interfaces);
    if (ctx->media_fd >= 0) {
        close(ctx->media_fd);
        ctx->media_fd = -1;
    }
    return ret;
}
