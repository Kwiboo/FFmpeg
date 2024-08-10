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
#include <libudev.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/types.h>
#include <unistd.h>

#include "libavutil/hwcontext_v4l2request.h"
#include "decode.h"
#include "v4l2_request_internal.h"

static const AVClass v4l2_request_context_class = {
    .class_name = "V4L2RequestContext",
    .item_name  = av_default_item_name,
    .version    = LIBAVUTIL_VERSION_INT,
};

int ff_v4l2_request_query_control(AVCodecContext *avctx,
                                  struct v4l2_query_ext_ctrl *control)
{
    int ret;
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;

    ret = ioctl(ctx->video_fd, VIDIOC_QUERY_EXT_CTRL, control);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: query control failed, %s (%d)\n", __func__, strerror(errno), errno);
        return AVERROR(EINVAL);
    }

    return 0;
}

int ff_v4l2_request_query_control_default_value(AVCodecContext *avctx,
                                                uint32_t id)
{
    int ret;
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    struct v4l2_queryctrl control = {
        .id = id,
    };

    ret = ioctl(ctx->video_fd, VIDIOC_QUERYCTRL, &control);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: query control failed, %s (%d)\n", __func__, strerror(errno), errno);
        return AVERROR(EINVAL);
    }

    return control.default_value;
}

static int v4l2_request_controls(V4L2RequestContext *ctx, int request_fd,
                                 unsigned long type,
                                 struct v4l2_ext_control *control, int count)
{
    struct v4l2_ext_controls controls = {
        .controls = control,
        .count = count,
        .request_fd = request_fd,
        .which = (request_fd >= 0) ? V4L2_CTRL_WHICH_REQUEST_VAL : 0,
    };

    if (!control || !count)
        return 0;

    return ioctl(ctx->video_fd, type, &controls);
}

int ff_v4l2_request_set_request_controls(V4L2RequestContext *ctx, int request_fd,
                                         struct v4l2_ext_control *control, int count)
{
    return v4l2_request_controls(ctx, request_fd, VIDIOC_S_EXT_CTRLS, control, count);
}

int ff_v4l2_request_set_controls(AVCodecContext *avctx,
                                 struct v4l2_ext_control *control, int count)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    int ret;

    ret = v4l2_request_controls(ctx, -1, VIDIOC_S_EXT_CTRLS, control, count);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: set controls failed, %s (%d)\n", __func__, strerror(errno), errno);
        return AVERROR(EINVAL);
    }

    return ret;
}

static int v4l2_request_buffer_alloc(V4L2RequestContext *ctx,
                                     V4L2RequestBuffer *buf,
                                     enum v4l2_buf_type type)
{
    int ret;
    struct v4l2_plane planes[1] = {};
    struct v4l2_create_buffers buffers = {
        .count = 1,
        .memory = V4L2_MEMORY_MMAP,
        .format.type = type,
    };

    av_log(ctx, AV_LOG_DEBUG, "%s: buf=%p type=%u\n", __func__, buf, type);

    ret = ioctl(ctx->video_fd, VIDIOC_G_FMT, &buffers.format);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "%s: get format failed for type %u, %s (%d)\n", __func__, type, strerror(errno), errno);
        return ret;
    }

    if (V4L2_TYPE_IS_MULTIPLANAR(buffers.format.type)) {
        struct v4l2_pix_format_mplane *fmt = &buffers.format.fmt.pix_mp;
        av_log(ctx, AV_LOG_DEBUG, "%s: pixelformat=%s width=%u height=%u bytesperline=%u sizeimage=%u num_planes=%u\n", __func__,
               av_fourcc2str(fmt->pixelformat), fmt->width, fmt->height, fmt->plane_fmt[0].bytesperline, fmt->plane_fmt[0].sizeimage, fmt->num_planes);
    } else {
        struct v4l2_pix_format *fmt = &buffers.format.fmt.pix;
        av_log(ctx, AV_LOG_DEBUG, "%s: pixelformat=%s width=%u height=%u bytesperline=%u sizeimage=%u\n", __func__,
               av_fourcc2str(fmt->pixelformat), fmt->width, fmt->height, fmt->bytesperline, fmt->sizeimage);
    }

    ret = ioctl(ctx->video_fd, VIDIOC_CREATE_BUFS, &buffers);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "%s: create buffers failed for type %u, %s (%d)\n", __func__, type, strerror(errno), errno);
        return ret;
    }

    if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
        buf->width = buffers.format.fmt.pix_mp.width;
        buf->height = buffers.format.fmt.pix_mp.height;
        buf->size = buffers.format.fmt.pix_mp.plane_fmt[0].sizeimage;
        buf->buffer.length = 1;
        buf->buffer.m.planes = planes;
    } else {
        buf->width = buffers.format.fmt.pix.width;
        buf->height = buffers.format.fmt.pix.height;
        buf->size = buffers.format.fmt.pix.sizeimage;
    }

    buf->index = buffers.index;
    buf->capabilities = buffers.capabilities;
    buf->used = 0;

    buf->buffer.type = type;
    buf->buffer.memory = V4L2_MEMORY_MMAP;
    buf->buffer.index = buf->index;

    ret = ioctl(ctx->video_fd, VIDIOC_QUERYBUF, &buf->buffer);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "%s: query buffer %d failed, %s (%d)\n", __func__, buf->index, strerror(errno), errno);
        return ret;
    }

    buf->buffer.timestamp.tv_usec = buf->index + 1;

    if (V4L2_TYPE_IS_OUTPUT(type)) {
        uint32_t offset = V4L2_TYPE_IS_MULTIPLANAR(type) ?
                       buf->buffer.m.planes[0].m.mem_offset :
                       buf->buffer.m.offset;
        void *addr = mmap(NULL, buf->size, PROT_READ | PROT_WRITE, MAP_SHARED,
                          ctx->video_fd, offset);
        if (addr == MAP_FAILED) {
            av_log(ctx, AV_LOG_ERROR, "%s: mmap failed, %s (%d)\n", __func__, strerror(errno), errno);
            return -1;
        }

        buf->addr = (uint8_t *)addr;
    } else {
        struct v4l2_exportbuffer exportbuffer = {
            .type = type,
            .index = buf->index,
            .flags = O_RDONLY,
        };

        ret = ioctl(ctx->video_fd, VIDIOC_EXPBUF, &exportbuffer);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "%s: export buffer %d failed, %s (%d)\n", __func__, buf->index, strerror(errno), errno);
            return ret;
        }

        buf->fd = exportbuffer.fd;
    }

    av_log(ctx, AV_LOG_DEBUG, "%s: buf=%p index=%d fd=%d addr=%p width=%u height=%u size=%u\n", __func__,
           buf, buf->index, buf->fd, buf->addr, buf->width, buf->height, buf->size);
    return 0;
}

static void v4l2_request_buffer_free(V4L2RequestBuffer *buf)
{
    av_log(NULL, AV_LOG_DEBUG, "%s: buf=%p index=%d fd=%d addr=%p width=%u height=%u size=%u\n", __func__,
           buf, buf->index, buf->fd, buf->addr, buf->width, buf->height, buf->size);

    if (buf->addr)
        munmap(buf->addr, buf->size);

    if (buf->fd >= 0)
        close(buf->fd);
}

static void v4l2_request_frame_free(void *opaque, uint8_t *data)
{
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)data;

    av_log(NULL, AV_LOG_DEBUG, "%s: opaque=%p data=%p request_fd=%d\n", __func__, opaque, data, req->request_fd);

    if (req->request_fd >= 0)
        close(req->request_fd);

    v4l2_request_buffer_free(&req->capture);
    v4l2_request_buffer_free(&req->output);

    av_free(data);
}

static AVBufferRef *v4l2_request_frame_alloc(void *opaque, size_t size)
{
    V4L2RequestContext *ctx = opaque;
    V4L2RequestDescriptor *req;
    AVBufferRef *ref;
    uint8_t *data;
    int ret;

    data = av_mallocz(size);
    if (!data)
        return NULL;

    av_log(ctx, AV_LOG_DEBUG, "%s: size=%zu data=%p\n", __func__, size, data);

    ref = av_buffer_create(data, size, v4l2_request_frame_free, ctx, 0);
    if (!ref) {
        av_freep(&data);
        return NULL;
    }

    req = (V4L2RequestDescriptor*)data;
    req->request_fd = -1;
    req->output.fd = -1;
    req->capture.fd = -1;

    ret = v4l2_request_buffer_alloc(ctx, &req->output, ctx->output_type);
    if (ret < 0) {
        av_buffer_unref(&ref);
        return NULL;
    }

    ret = v4l2_request_buffer_alloc(ctx, &req->capture, ctx->format.type);
    if (ret < 0) {
        av_buffer_unref(&ref);
        return NULL;
    }

    ret = ioctl(ctx->media_fd, MEDIA_IOC_REQUEST_ALLOC, &req->request_fd);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "%s: request alloc failed, %s (%d)\n", __func__, strerror(errno), errno);
        av_buffer_unref(&ref);
        return NULL;
    }

    av_log(ctx, AV_LOG_DEBUG, "%s: size=%zu data=%p request_fd=%d\n", __func__, size, data, req->request_fd);
    return ref;
}

static void v4l2_request_pool_free(void *opaque)
{
    av_log(NULL, AV_LOG_DEBUG, "%s: opaque=%p\n", __func__, opaque);
}

static void v4l2_request_hwframe_ctx_free(AVHWFramesContext *hwfc)
{
    av_log(NULL, AV_LOG_DEBUG, "%s: hwfc=%p pool=%p\n", __func__, hwfc, hwfc->pool);

    av_buffer_pool_flush(hwfc->pool);
    av_buffer_pool_uninit(&hwfc->pool);
}

int ff_v4l2_request_frame_params(AVCodecContext *avctx, AVBufferRef *hw_frames_ctx)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    AVHWFramesContext *hwfc = (AVHWFramesContext*)hw_frames_ctx->data;
    uint32_t pixelformat;

    hwfc->format = AV_PIX_FMT_DRM_PRIME;
    hwfc->sw_format = ff_v4l2_request_get_sw_format(&ctx->format);

    if (V4L2_TYPE_IS_MULTIPLANAR(ctx->format.type)) {
        hwfc->width = ctx->format.fmt.pix_mp.width;
        hwfc->height = ctx->format.fmt.pix_mp.height;
        pixelformat = ctx->format.fmt.pix_mp.pixelformat;
    } else {
        hwfc->width = ctx->format.fmt.pix.width;
        hwfc->height = ctx->format.fmt.pix.height;
        pixelformat = ctx->format.fmt.pix.pixelformat;
    }

    hwfc->pool = av_buffer_pool_init2(sizeof(V4L2RequestDescriptor), ctx,
                                      v4l2_request_frame_alloc, v4l2_request_pool_free);
    if (!hwfc->pool)
        return AVERROR(ENOMEM);

    hwfc->free = v4l2_request_hwframe_ctx_free;

    hwfc->initial_pool_size = 1;

    switch (avctx->codec_id) {
    case AV_CODEC_ID_VP9:
        hwfc->initial_pool_size += 8;
        break;
    case AV_CODEC_ID_VP8:
        hwfc->initial_pool_size += 3;
        break;
    default:
        hwfc->initial_pool_size += 2;
    }

    av_log(ctx, AV_LOG_DEBUG, "%s: avctx=%p hw_frames_ctx=%p hwfc=%p pool=%p width=%d height=%d initial_pool_size=%d\n", __func__,
           avctx, hw_frames_ctx, hwfc, hwfc->pool, hwfc->width, hwfc->height, hwfc->initial_pool_size);
    return 0;
}

int ff_v4l2_request_uninit(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    int ret;

    av_log(ctx, AV_LOG_DEBUG, "%s: avctx=%p\n", __func__, avctx);

    if (ctx->video_fd >= 0) {
        ret = ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &ctx->output_type);
        if (ret < 0)
            av_log(avctx, AV_LOG_ERROR, "%s: output stream off failed, %s (%d)\n", __func__, strerror(errno), errno);

        ret = ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &ctx->format.type);
        if (ret < 0)
            av_log(avctx, AV_LOG_ERROR, "%s: capture stream off failed, %s (%d)\n", __func__, strerror(errno), errno);
    }

    if (avctx->hw_frames_ctx) {
        AVHWFramesContext *hwfc = (AVHWFramesContext*)avctx->hw_frames_ctx->data;
        av_buffer_pool_flush(hwfc->pool);
    }

    if (ctx->video_fd >= 0)
        close(ctx->video_fd);

    if (ctx->media_fd >= 0)
        close(ctx->media_fd);

    return 0;
}

static int v4l2_request_init_context(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    int ret;

    ret = ioctl(ctx->video_fd, VIDIOC_G_FMT, &ctx->format);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get capture format failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if (V4L2_TYPE_IS_MULTIPLANAR(ctx->format.type)) {
        struct v4l2_pix_format_mplane *fmt = &ctx->format.fmt.pix_mp;
        av_log(ctx, AV_LOG_DEBUG, "%s: pixelformat=%s width=%u height=%u bytesperline=%u sizeimage=%u num_planes=%u\n", __func__,
               av_fourcc2str(fmt->pixelformat), fmt->width, fmt->height, fmt->plane_fmt[0].bytesperline, fmt->plane_fmt[0].sizeimage, fmt->num_planes);
    } else {
        struct v4l2_pix_format *fmt = &ctx->format.fmt.pix;
        av_log(ctx, AV_LOG_DEBUG, "%s: pixelformat=%s width=%u height=%u bytesperline=%u sizeimage=%u\n", __func__,
               av_fourcc2str(fmt->pixelformat), fmt->width, fmt->height, fmt->bytesperline, fmt->sizeimage);
    }

    ret = ff_decode_get_hw_frames_ctx(avctx, AV_HWDEVICE_TYPE_V4L2REQUEST);
    if (ret < 0)
        goto fail;

    ret = ioctl(ctx->video_fd, VIDIOC_STREAMON, &ctx->output_type);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: output stream on failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = ioctl(ctx->video_fd, VIDIOC_STREAMON, &ctx->format.type);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: capture stream on failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    return 0;

fail:
    ff_v4l2_request_uninit(avctx);
    return ret;
}

int ff_v4l2_request_init(AVCodecContext *avctx,
                         uint32_t pixelformat, uint32_t buffersize,
                         struct v4l2_ext_control *control, int count)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    int ret = AVERROR(EINVAL);
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices;
    struct udev_list_entry *entry;
    struct udev_device *device;

    av_log(avctx, AV_LOG_DEBUG, "%s: ctx=%p hw_device_ctx=%p hw_frames_ctx=%p\n", __func__, ctx, avctx->hw_device_ctx, avctx->hw_frames_ctx);

    ctx->av_class = &v4l2_request_context_class;
    ctx->media_fd = -1;
    ctx->video_fd = -1;

    udev = udev_new();
    if (!udev) {
        av_log(avctx, AV_LOG_ERROR, "%s: allocating udev context failed\n", __func__);
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    enumerate = udev_enumerate_new(udev);
    if (!enumerate) {
        av_log(avctx, AV_LOG_ERROR, "%s: allocating udev enumerator failed\n", __func__);
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    udev_enumerate_add_match_subsystem(enumerate, "media");
    udev_enumerate_scan_devices(enumerate);

    devices = udev_enumerate_get_list_entry(enumerate);
    udev_list_entry_foreach(entry, devices) {
        const char *path = udev_list_entry_get_name(entry);
        if (!path)
            continue;

        device = udev_device_new_from_syspath(udev, path);
        if (!device)
            continue;

        ret = ff_v4l2_request_probe_media_device(device, avctx, pixelformat, buffersize, control, count);
        udev_device_unref(device);

        if (!ret)
            break;
    }

    udev_enumerate_unref(enumerate);

    if (!ret)
        ret = v4l2_request_init_context(avctx);
    else
        av_log(avctx, AV_LOG_INFO, "No V4L2 media device found for %s\n", av_fourcc2str(pixelformat));

fail:
    udev_unref(udev);
    return ret;
}
