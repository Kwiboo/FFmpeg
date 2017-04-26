#include <libdrm/drm_fourcc.h>
#include <rockchip/mpp_buffer.h>
#include <pthread.h>
#include <rockchip/rk_mpi.h>
#include <unistd.h>
#include <time.h>

#include "avcodec.h"
#include "internal.h"
#include "drmprime.h"
#include "libavutil/log.h"
#include "libavutil/common.h"
#include "libavutil/buffer.h"
#include "libavutil/frame.h"
#include "libavutil/imgutils.h"

#define RECEIVE_FRAME_TIMEOUT	100

typedef struct {

    AVBufferRef *ref;

    MppCtx ctx;
    MppApi *mpi;
    MppBufferGroup frame_group;

    char first_packet;
    char eos_reached;

} RKMPPDecoder;

typedef struct {
    AVClass *av_class;

    // bitstream filter in case we need some annexb compliant stream
    AVBSFContext *bsf;

    RKMPPDecoder *decoder;

    int framecount;
    int fill_decoder;

} RKMPPDecodeContext;

static void profile_log(AVCodecContext *avctx, const char *fmt, ...)
{
    struct timespec time;
    static struct timespec lasttime,starttime;
    static int firstcall = 1;
    int msec, usec, deltams;
    va_list ap;
    char stamp[32];
    char msg[512];

    if (firstcall) {
        clock_gettime(CLOCK_REALTIME, &starttime);
        firstcall = 0;
    }

    clock_gettime(CLOCK_REALTIME, &time);

    msec = time.tv_nsec / 1000000;
    usec = (time.tv_nsec - msec * 1000000) / 1000;
    deltams = ((time.tv_sec * 1000000000 + time.tv_nsec) - (lasttime.tv_sec * 1000000000 + lasttime.tv_nsec)) / 1000000;

    sprintf(stamp, "%03d:%03d.%03d (+%03d ms): ", (int)(time.tv_sec -  starttime.tv_sec), msec, usec, deltams);

    lasttime.tv_sec = time.tv_sec;
    lasttime.tv_nsec = time.tv_nsec;

    va_start(ap, fmt);
    vsprintf(msg, fmt, ap);
    va_end(ap);

    av_log(avctx, AV_LOG_DEBUG, "%s%s\n", stamp, msg);
}

static double ffrkmpp_compute_framerate(AVCodecContext *avctx)
{
    static struct timespec reftime;
    static int refframecount;
    struct timespec time;
    double timediff;

    RKMPPDecodeContext *rk_context = avctx->priv_data;

    clock_gettime(CLOCK_REALTIME, &time);

    if (rk_context->framecount == 1 || rk_context->framecount == 2 || rk_context->framecount % 1000 == 0) {
        refframecount = rk_context->framecount;
        reftime.tv_sec  = time.tv_sec;
        reftime.tv_nsec = time.tv_nsec;
    }
    timediff = ((double)time.tv_sec + ((double)time.tv_nsec / 1000000000.0)) -
               ((double)reftime.tv_sec + ((double)reftime.tv_nsec / 1000000000.0));

    if (timediff != 0)
        return (double)(rk_context->framecount - refframecount) / timediff;
    else
        return 0;
}

static MppCodingType ffrkmpp_get_codingtype(AVCodecContext *avctx)
{
    switch(avctx->codec_id) {
        case AV_CODEC_ID_H264:
        return MPP_VIDEO_CodingAVC;

        case AV_CODEC_ID_HEVC:
        return MPP_VIDEO_CodingHEVC;

        case AV_CODEC_ID_VP8:
        return MPP_VIDEO_CodingVP8;

        default:
        return MPP_VIDEO_CodingUnused;
    }
}

static int ffrkmpp_get_frameformat(MppFrameFormat mppformat)
{
    switch(mppformat) {
        case MPP_FMT_YUV420SP:
        return DRM_FORMAT_NV12;

        case MPP_FMT_YUV420SP_10BIT:
        return DRM_FORMAT_NV12_10;

        default:
        return 0;
    }
}

static int ffrkmpp_init_bitstream(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    int ret = 0;

    if (!rk_context->bsf) {
        const AVBitStreamFilter *bsf;

        // check if we need a bitstream filter
        switch(avctx->codec_id) {
            case AV_CODEC_ID_H264:
            bsf = av_bsf_get_by_name("h264_mp4toannexb");
            break;

            case AV_CODEC_ID_HEVC:
            bsf = av_bsf_get_by_name("hevc_mp4toannexb");
            break;

            default:
            av_log(avctx, AV_LOG_DEBUG, "Not using any bitstream filter\n");
            return 0;
        }

        if(!bsf)
            return AVERROR_BSF_NOT_FOUND;

        av_log(avctx, AV_LOG_DEBUG, "using bitstream filter %s\n", bsf->name);

        if ((ret = av_bsf_alloc(bsf, &rk_context->bsf)))
            return ret;

        if (((ret = avcodec_parameters_from_context(rk_context->bsf->par_in, avctx)) < 0) ||
            ((ret = av_bsf_init(rk_context->bsf)) < 0)) {
            av_bsf_free(&rk_context->bsf);
            return ret;
        }
    }

    return 0;
}

static int ffrkmpp_write_data(AVCodecContext *avctx, char *buffer, int size, int64_t pts)
{
    MppPacket packet;
    MPP_RET ret = MPP_OK;
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = rk_context->decoder;

    // create the MPP packet
    ret = mpp_packet_init(&packet, buffer, size);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to init MPP packet (code = %d)\n", ret);
        return ret;
    }

    // set the pts
    mpp_packet_set_pts(packet, pts);

    // if we have a NULL packet, then assume EOS
    if (!buffer)
        mpp_packet_set_eos(packet);

    // write it to decoder
    ret = decoder->mpi->decode_put_packet(decoder->ctx, packet);

    if (ret != MPP_ERR_BUFFER_FULL)
        profile_log(avctx, "Wrote %d bytes to decoder", size);
    else
        profile_log(avctx, "Buffer full writing %d bytes to decoder", size);

    mpp_packet_deinit(&packet);

    return ret;
}

static int ffrkmpp_close_decoder(AVCodecContext *avctx)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = rk_context->decoder;

    av_buffer_unref(&decoder->ref);
    av_bsf_free(&rk_context->bsf);

    return 0;
}

static void ffrkmpp_release_decoder(void *opaque, uint8_t *data)
{
    RKMPPDecoder *decoder = (RKMPPDecoder *)data;

    if (decoder) {
        decoder->mpi->reset(decoder->ctx);
        mpp_destroy(decoder->ctx);
        decoder->ctx = NULL;
    }

}

static int ffrkmpp_init_decoder(AVCodecContext *avctx)
{
    MPP_RET ret = MPP_OK;
    MppCodingType codectype = MPP_VIDEO_CodingUnused;

    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder;
    RK_S64 paramS64;
    RK_S32 paramS32;

    if ((ret = ff_get_format(avctx, avctx->codec->pix_fmts)) < 0)
        return ret;

    avctx->pix_fmt = ret;

    // create a decoder and a ref to it
    decoder = av_mallocz(sizeof(RKMPPDecoder));
    if (!decoder) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    rk_context->decoder = decoder;

    decoder->ref = av_buffer_create((uint8_t*)decoder, sizeof(*decoder), ffrkmpp_release_decoder,
                                    NULL, AV_BUFFER_FLAG_READONLY);
    if (!decoder->ref) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    av_log(avctx, AV_LOG_DEBUG, "Initializing RKMPP decoder.\n");

    // Create the MPP context
    ret = mpp_create(&decoder->ctx, &decoder->mpi);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to create MPP context (code = %d).\n", ret);
        goto fail;
    }

    // initialize the context
    codectype = ffrkmpp_get_codingtype(avctx);
    if (codectype == MPP_VIDEO_CodingUnused) {
        av_log(avctx, AV_LOG_ERROR, "Unknown codec type (%d).\n", ret);
        goto fail;
    }

    // initialize mpp
    ret = mpp_init(decoder->ctx, MPP_CTX_DEC, codectype);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to initialize MPP context (code = %d).\n", ret);
        goto fail;
    }

    // make decode calls blocking
    paramS32 = MPP_POLL_BLOCK;
    ret = decoder->mpi->control(decoder->ctx, MPP_SET_OUTPUT_BLOCK, &paramS32);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to set blocking mode on MPI (code = %d).\n", ret);
        goto fail;
    }

    paramS64 = RECEIVE_FRAME_TIMEOUT;
    ret = decoder->mpi->control(decoder->ctx, MPP_SET_OUTPUT_BLOCK_TIMEOUT, &paramS64);
    if (ret != MPP_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to set block timeout on MPI (code = %d).\n", ret);
        goto fail;
    }

    // eventually create a bistream filter for formats that require it
    ret = ffrkmpp_init_bitstream(avctx);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to initialize RKMPP bitstream.\n");
        goto fail;
    }

    decoder->first_packet = 1;
    decoder->eos_reached = 0;
    rk_context->fill_decoder = 1;

    av_log(avctx, AV_LOG_DEBUG, "RKMPP decoder initialized successfully.\n");
        return 0;

fail:
    if ((decoder) && (decoder->ref))
        av_buffer_unref(&decoder->ref);

    av_log(avctx, AV_LOG_ERROR, "Failed to initialize RKMPP decoder.\n");
    ffrkmpp_close_decoder(avctx);
    return ret;
}

static int ffrkmpp_send_packet(AVCodecContext *avctx, const AVPacket *avpkt)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = rk_context->decoder;
    AVPacket filter_pkt = {0};
    AVPacket filtered_packet = {0};
    MPP_RET  ret = MPP_OK;
    int retcode = 0;

    // handle EOF
    if (avpkt == NULL) {
        decoder->eos_reached = 1;
        ret = ffrkmpp_write_data(avctx, NULL, 0, 0);

        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "Failed to send EOS to decoder\n");
            goto fail;
        }

        return ret;
    }

    // first we bitstream the packet if it's required
    // this seems to be required for H264 / HEVC / H265
    if (rk_context->bsf) {
        // if we use a bitstream filter, then use it
        if ((ret = av_packet_ref(&filter_pkt, avpkt)) < 0)
            return ret;

        if ((ret = av_bsf_send_packet(rk_context->bsf, &filter_pkt)) < 0) {
            av_packet_unref(&filter_pkt);
            return ret;
        }

        if ((ret = av_bsf_receive_packet(rk_context->bsf, &filtered_packet)) < 0)
            return ret;

        avpkt = &filtered_packet;
    }

    // on first packet, send extradata
    if (decoder->first_packet) {

        if (rk_context->bsf && rk_context->bsf->par_out->extradata_size) {
            ret = ffrkmpp_write_data(avctx, rk_context->bsf->par_out->extradata,
                                          rk_context->bsf->par_out->extradata_size,
                                          avpkt->pts);
            if (ret) {
                av_log(avctx, AV_LOG_ERROR, "Failed to write extradata to decoder\n");
                goto fail;
            }
        }

        else if (avctx->extradata && avctx->extradata_size) {
            ret = ffrkmpp_write_data(avctx, avctx->extradata,
                                          avctx->extradata_size,
                                          avpkt->pts);
            if (ret) {
                av_log(avctx, AV_LOG_ERROR, "Failed to write extradata to decoder\n");
                goto fail;
            }
        }

        decoder->first_packet = 0;
    }

    // now send packet
    ret = ffrkmpp_write_data(avctx, avpkt->data, avpkt->size, avpkt->pts);
    if (ret != MPP_OK) {
        if (ret == MPP_ERR_BUFFER_FULL) {
            retcode = AVERROR(EAGAIN);
            rk_context->fill_decoder = 0;
        }
        else {
            av_log(avctx, AV_LOG_ERROR, "Failed to write data to decoder (%d)\n", ret);
            goto fail;
        }
    }

    // release the ref created by filtered packet
    if (rk_context->bsf)
        av_packet_unref(&filtered_packet);

    return retcode;

fail:
    // release the ref created by filtered packet
    if (rk_context->bsf)
        av_packet_unref(&filtered_packet);

    return ret;
}


static void ffrkmpp_release_frame(void *opaque, uint8_t *data)
{
    MppFrame mppframe = (MppFrame)opaque;

    if (mppframe)
        mpp_frame_deinit(&mppframe);
}

static int ffrkmpp_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = rk_context->decoder;
    MPP_RET  ret = MPP_OK;
    MppFrame mppframe = NULL;
    MppBuffer buffer = NULL;
    MppBufferInfo bufferinfo;
    av_drmprime *primedata = NULL;
    double fps;
    int retrycount;

    // if we want to fill the decoder, switch to send_packet
    if ((rk_context->fill_decoder) && (!decoder->eos_reached))
        return AVERROR(EAGAIN);

    // now we will try to get a frame back
    retrycount = 0;

retry :
    ret = decoder->mpi->decode_get_frame(decoder->ctx, &mppframe);
    if ((ret != MPP_OK) && (ret != MPP_ERR_TIMEOUT)) {
        if (retrycount < 5) {
            // on decoding, MPP can return -1, then we need to retry
            usleep(50000);
            retrycount++;
            goto retry;
        }
        else {
            av_log(avctx, AV_LOG_ERROR, "Failed to get a frame from MPP (code = %d)\n", ret);
            goto fail;
        }

    }

    if (mppframe) {
        if (mpp_frame_get_info_change(mppframe)) {
            av_log(avctx, AV_LOG_INFO, "Decoder noticed an info change (%dx%d), format=%d\n",
                                        mpp_frame_get_width(mppframe),  mpp_frame_get_height(mppframe),
                                        mpp_frame_get_fmt(mppframe));
            decoder->mpi->control(decoder->ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
            mpp_frame_deinit(&mppframe);

            // here decoder is fully initialized, we need to feed it again with data
            rk_context->fill_decoder = 1;
            return AVERROR(EAGAIN);
        }
        else {
            rk_context->framecount++;
            fps = ffrkmpp_compute_framerate(avctx);
            profile_log(avctx, "Received frame, (%.2f fps)", fps);
        }
    }

    if (mppframe) {
        // setup general frame fields
        frame->format = AV_PIX_FMT_RKMPP;
        frame->width  = mpp_frame_get_width(mppframe);
        frame->height = mpp_frame_get_height(mppframe);
        frame->pts    = mpp_frame_get_pts(mppframe);

        // now setup the frame buffer info
        buffer = mpp_frame_get_buffer(mppframe);
        if (buffer) {
            ret = mpp_buffer_info_get(buffer, &bufferinfo);
            if (ret != MPP_OK) {
                av_log(avctx, AV_LOG_ERROR, "Failed to get info from MPP buffer (code = %d)\n", ret);
                goto fail;
            }

            primedata = av_mallocz(sizeof(av_drmprime));
            if (!primedata) {
                av_log(avctx, AV_LOG_ERROR, "Failed to allocated drm prime data.\n");
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            // now fill up the DRMPRIME data
            primedata->strides[0]   = mpp_frame_get_hor_stride(mppframe);
            primedata->strides[1]   = primedata->strides[0];
            primedata->offsets[0]   = 0;
            primedata->offsets[1]   = primedata->strides[0] * mpp_frame_get_ver_stride(mppframe);
            primedata->fd           = mpp_buffer_get_fd(buffer);
            primedata->format       = ffrkmpp_get_frameformat(mpp_frame_get_fmt(mppframe));

            frame->data[3] = (uint8_t*)primedata;
            frame->buf[0]  = av_buffer_create((uint8_t*)primedata, sizeof(*primedata), ffrkmpp_release_frame,
                                          mppframe, AV_BUFFER_FLAG_READONLY);

            if (!frame->buf[0]) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            // add a ref to decoder for each frame as we need to release it
            // only once all frames have been released
            frame->buf[1] = av_buffer_ref(decoder->ref);

            return 0;
        }

    }
    else {
	if (decoder->eos_reached)
            return AVERROR_EOF;
    }

    return AVERROR(EAGAIN);

fail:
    if (primedata)
        av_free(primedata);

    if (mppframe)
        mpp_frame_deinit(&mppframe);

    return ret;
}

static int ffrkmpp_decode(AVCodecContext *avctx, void *frame, int *got_frame, AVPacket *avpkt)
{
    int buffer_full = 0;
    int decoder_timeout = 0;
    int retries = 0;
    int ret;

    *got_frame = 0;

    do {
        if (avpkt) {
            if (avpkt->data == NULL && avpkt->size == 0)
                ret = ffrkmpp_send_packet(avctx, NULL);
            else
                ret = ffrkmpp_send_packet(avctx, avpkt);
            if (ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
                av_log(avctx, AV_LOG_ERROR, "Failed to send packet (code = %d)\n", ret);
                return ret;
            }
            buffer_full = ret == AVERROR(EAGAIN);
        }

        ret = ffrkmpp_receive_frame(avctx, frame);
        if (ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
            av_log(avctx, AV_LOG_ERROR, "Failed to receive frame (code = %d)\n", ret);
            return ret;
        }
        if (ret >= 0)
            *got_frame = 1;

        decoder_timeout = ret == AVERROR(EAGAIN);
    }
    while (buffer_full && decoder_timeout && retries++ < 5);

    if (buffer_full) {
        ret = ffrkmpp_send_packet(avctx, avpkt);
        if (ret < 0)
            av_log(avctx, AV_LOG_ERROR, "Failed to send packet (code = %d)\n", ret);
    }

    return avpkt ? avpkt->size : 0;
}

static void ffrkmpp_flush(AVCodecContext *avctx)
{
    MPP_RET ret = MPP_OK;
    RKMPPDecodeContext *rk_context = avctx->priv_data;
    RKMPPDecoder *decoder = rk_context->decoder;

    ret = decoder->mpi->reset(decoder->ctx);
    if (ret == MPP_OK)
        decoder->first_packet = 1;
    else
        av_log(avctx, AV_LOG_ERROR, "Failed to reset MPI (code = %d)\n", ret);
}

#define FFRKMPP_DEC_HWACCEL(NAME, ID) \
  AVHWAccel ff_##NAME##_rkmpp_hwaccel = { \
      .name       = #NAME "_rkmpp", \
      .type       = AVMEDIA_TYPE_VIDEO,\
      .id         = ID, \
      .pix_fmt    = AV_PIX_FMT_RKMPP,\
  };

#define FFRKMPP_DEC_CLASS(NAME) \
    static const AVClass ffrkmpp_##NAME##_dec_class = { \
        .class_name = "rkmpp_" #NAME "_dec", \
        .version    = LIBAVUTIL_VERSION_INT, \
    };

#define FFRKMPP_DEC(NAME, ID) \
    FFRKMPP_DEC_CLASS(NAME) \
    FFRKMPP_DEC_HWACCEL(NAME, ID) \
    AVCodec ff_##NAME##_rkmpp_decoder = { \
        .name           = #NAME "_rkmpp", \
        .long_name      = NULL_IF_CONFIG_SMALL(#NAME " (rkmpp)"), \
        .type           = AVMEDIA_TYPE_VIDEO, \
        .id             = ID, \
        .priv_data_size = sizeof(RKMPPDecodeContext), \
        .init           = ffrkmpp_init_decoder, \
        .close          = ffrkmpp_close_decoder, \
        .decode         = ffrkmpp_decode, \
        .send_packet    = ffrkmpp_send_packet, \
        .receive_frame  = ffrkmpp_receive_frame, \
        .flush          = ffrkmpp_flush, \
        .priv_class     = &ffrkmpp_##NAME##_dec_class, \
        .capabilities   = AV_CODEC_CAP_DELAY, \
        .caps_internal  = FF_CODEC_CAP_SETS_PKT_DTS, \
        .pix_fmts       = (const enum AVPixelFormat[]) { AV_PIX_FMT_RKMPP, \
                                                         AV_PIX_FMT_NONE}, \
    };

FFRKMPP_DEC(h264, AV_CODEC_ID_H264)
FFRKMPP_DEC(hevc, AV_CODEC_ID_HEVC)
FFRKMPP_DEC(vp8,  AV_CODEC_ID_VP8)
