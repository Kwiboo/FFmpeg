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

#include "hwaccel_internal.h"
#include "hwconfig.h"
#include "v4l2_request.h"
#include "vp9dec.h"

typedef struct V4L2RequestControlsVP9 {
    V4L2RequestPictureContext pic;
    struct v4l2_ctrl_vp9_frame frame;
    struct v4l2_ctrl_vp9_compressed_hdr chp;
} V4L2RequestControlsVP9;

static void v4l2_request_vp9_set_frame_ctx(AVCodecContext *avctx)
{
    VP9Context *s = avctx->priv_data;
    const VP9Frame *f = &s->s.frames[CUR_FRAME];
    V4L2RequestControlsVP9 *controls = f->hwaccel_picture_private;
    struct v4l2_ctrl_vp9_compressed_hdr *chp = &controls->chp;

    memset(chp, 0, sizeof(&chp));

    chp->tx_mode = s->s.h.txfmmode;
    memcpy(chp->tx8, s->prob_raw.p.tx8p, sizeof(s->prob_raw.p.tx8p));
    memcpy(chp->tx16, s->prob_raw.p.tx16p, sizeof(s->prob_raw.p.tx16p));
    memcpy(chp->tx32, s->prob_raw.p.tx32p, sizeof(s->prob_raw.p.tx32p));
    for (unsigned i = 0; i < 4; i++) {
        for (unsigned j = 0; j < 2; j++) {
            for (unsigned k = 0; k < 2; k++) {
                for (unsigned l = 0; l < 6; l++) {
                    for (unsigned m = 0; m < 6; m++) {
                        memcpy(chp->coef[i][j][k][l][m], s->prob_raw.coef[i][j][k][l][m], sizeof(chp->coef[0][0][0][0][0]));
                    }
                }
            }
        }
    }
    memcpy(chp->skip, s->prob_raw.p.skip, sizeof(s->prob_raw.p.skip));
    memcpy(chp->inter_mode, s->prob_raw.p.mv_mode, sizeof(s->prob_raw.p.mv_mode));
    memcpy(chp->interp_filter, s->prob_raw.p.filter, sizeof(s->prob_raw.p.filter));
    memcpy(chp->is_inter, s->prob_raw.p.intra, sizeof(s->prob_raw.p.intra));
    memcpy(chp->comp_mode, s->prob_raw.p.comp, sizeof(s->prob_raw.p.comp));
    memcpy(chp->single_ref, s->prob_raw.p.single_ref, sizeof(s->prob_raw.p.single_ref));
    memcpy(chp->comp_ref, s->prob_raw.p.comp_ref, sizeof(s->prob_raw.p.comp_ref));
    memcpy(chp->y_mode, s->prob_raw.p.y_mode, sizeof(s->prob_raw.p.y_mode));
    for (unsigned i = 0; i < 10; i++)
        memcpy(chp->uv_mode[i], s->prob.p.uv_mode[i], sizeof(s->prob.p.uv_mode[0]));
    for (unsigned i = 0; i < 4; i++)
        memcpy(chp->partition[i * 4], s->prob_raw.p.partition[i], sizeof(s->prob_raw.p.partition[0]));
    memcpy(chp->mv.joint, s->prob_raw.p.mv_joint, sizeof(s->prob_raw.p.mv_joint));
    for (unsigned i = 0; i < 2; i++) {
         chp->mv.sign[i] = s->prob_raw.p.mv_comp[i].sign;
         memcpy(chp->mv.classes[i], s->prob_raw.p.mv_comp[i].classes, sizeof(s->prob_raw.p.mv_comp[0].classes));
         chp->mv.class0_bit[i] = s->prob_raw.p.mv_comp[i].class0;
         memcpy(chp->mv.bits[i], s->prob_raw.p.mv_comp[i].bits, sizeof(s->prob_raw.p.mv_comp[0].bits));
         memcpy(chp->mv.class0_fr[i], s->prob_raw.p.mv_comp[i].class0_fp, sizeof(s->prob_raw.p.mv_comp[0].class0_fp));
         memcpy(chp->mv.fr[i], s->prob_raw.p.mv_comp[i].fp, sizeof(s->prob_raw.p.mv_comp[0].fp));
         chp->mv.class0_hp[i] = s->prob_raw.p.mv_comp[i].class0_hp;
         chp->mv.hp[i] = s->prob_raw.p.mv_comp[i].hp;
    }
}

static void fill_frame(struct v4l2_ctrl_vp9_frame *dec_params, AVCodecContext *avctx)
{
    const VP9Context *s = avctx->priv_data;
    AVFrame *ref;

    memset(dec_params, 0, sizeof(*dec_params));

    if (s->s.h.keyframe)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_KEY_FRAME;
    if (!s->s.h.invisible)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_SHOW_FRAME;
    if (s->s.h.errorres)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT;
    if (s->s.h.intraonly)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_INTRA_ONLY;
    if (!s->s.h.keyframe && s->s.h.highprecisionmvs)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV;
    if (s->s.h.refreshctx)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX;
    if (s->s.h.parallelmode)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE;
    if (s->ss_h)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_X_SUBSAMPLING;
    if (s->ss_v)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_Y_SUBSAMPLING;
    if (avctx->color_range == AVCOL_RANGE_JPEG)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_COLOR_RANGE_FULL_SWING;

    dec_params->compressed_header_size = s->s.h.compressed_header_size;
    dec_params->uncompressed_header_size = s->s.h.uncompressed_header_size;
    dec_params->profile = s->s.h.profile;
    dec_params->reset_frame_context = s->s.h.resetctx > 0 ? s->s.h.resetctx - 1 : 0;
    dec_params->frame_context_idx = s->s.h.framectxid;
    dec_params->bit_depth = s->s.h.bpp;

    dec_params->interpolation_filter = s->s.h.filtermode ^ (s->s.h.filtermode <= 1);
    dec_params->tile_cols_log2 = s->s.h.tiling.log2_tile_cols;
    dec_params->tile_rows_log2 = s->s.h.tiling.log2_tile_rows;
    dec_params->reference_mode = s->s.h.comppredmode;
    dec_params->frame_width_minus_1 = s->w - 1;
    dec_params->frame_height_minus_1 = s->h - 1;
    //dec_params->render_width_minus_1 = avctx->width - 1;
    //dec_params->render_height_minus_1 = avctx->height - 1;

    ref = s->s.refs[s->s.h.refidx[0]].f;
    if (ref && ref->buf[0])
        dec_params->last_frame_ts = ff_v4l2_request_get_capture_timestamp(ref);
    ref = s->s.refs[s->s.h.refidx[1]].f;
    if (ref && ref->buf[0])
        dec_params->golden_frame_ts = ff_v4l2_request_get_capture_timestamp(ref);
    ref = s->s.refs[s->s.h.refidx[2]].f;
    if (ref && ref->buf[0])
        dec_params->alt_frame_ts = ff_v4l2_request_get_capture_timestamp(ref);

    if (s->s.h.signbias[0])
        dec_params->ref_frame_sign_bias |= V4L2_VP9_SIGN_BIAS_LAST;
    if (s->s.h.signbias[1])
        dec_params->ref_frame_sign_bias |= V4L2_VP9_SIGN_BIAS_GOLDEN;
    if (s->s.h.signbias[2])
        dec_params->ref_frame_sign_bias |= V4L2_VP9_SIGN_BIAS_ALT;

    if (s->s.h.lf_delta.enabled)
        dec_params->lf.flags |= V4L2_VP9_LOOP_FILTER_FLAG_DELTA_ENABLED;
    if (s->s.h.lf_delta.updated)
        dec_params->lf.flags |= V4L2_VP9_LOOP_FILTER_FLAG_DELTA_UPDATE;

    dec_params->lf.level = s->s.h.filter.level;
    dec_params->lf.sharpness = s->s.h.filter.sharpness;
    for (unsigned i = 0; i < 4; i++)
        dec_params->lf.ref_deltas[i] = s->s.h.lf_delta.ref[i];
    for (unsigned i = 0; i < 2; i++)
        dec_params->lf.mode_deltas[i] = s->s.h.lf_delta.mode[i];

    dec_params->quant.base_q_idx = s->s.h.yac_qi;
    dec_params->quant.delta_q_y_dc = s->s.h.ydc_qdelta;
    dec_params->quant.delta_q_uv_dc = s->s.h.uvdc_qdelta;
    dec_params->quant.delta_q_uv_ac = s->s.h.uvac_qdelta;

    if (s->s.h.segmentation.enabled)
        dec_params->seg.flags |= V4L2_VP9_SEGMENTATION_FLAG_ENABLED;
    if (s->s.h.segmentation.update_map)
        dec_params->seg.flags |= V4L2_VP9_SEGMENTATION_FLAG_UPDATE_MAP;
    if (s->s.h.segmentation.temporal)
        dec_params->seg.flags |= V4L2_VP9_SEGMENTATION_FLAG_TEMPORAL_UPDATE;
    if (s->s.h.segmentation.update_data)
        dec_params->seg.flags |= V4L2_VP9_SEGMENTATION_FLAG_UPDATE_DATA;
    if (s->s.h.segmentation.absolute_vals)
        dec_params->seg.flags |= V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE;

    for (unsigned i = 0; i < 7; i++)
        dec_params->seg.tree_probs[i] = s->s.h.segmentation.prob[i];

    if (s->s.h.segmentation.temporal) {
        for (unsigned i = 0; i < 3; i++)
            dec_params->seg.pred_probs[i] = s->s.h.segmentation.pred_prob[i];
    } else {
        memset(dec_params->seg.pred_probs, 255, sizeof(dec_params->seg.pred_probs));
    }

    for (unsigned i = 0; i < 8; i++) {
        if (s->s.h.segmentation.feat[i].q_enabled) {
            dec_params->seg.feature_enabled[i] |= 1 << V4L2_VP9_SEG_LVL_ALT_Q;
            dec_params->seg.feature_data[i][V4L2_VP9_SEG_LVL_ALT_Q] = s->s.h.segmentation.feat[i].q_val;
        }

        if (s->s.h.segmentation.feat[i].lf_enabled) {
            dec_params->seg.feature_enabled[i] |= 1 << V4L2_VP9_SEG_LVL_ALT_L;
            dec_params->seg.feature_data[i][V4L2_VP9_SEG_LVL_ALT_L] = s->s.h.segmentation.feat[i].lf_val;
        }

        if (s->s.h.segmentation.feat[i].ref_enabled) {
            dec_params->seg.feature_enabled[i] |= 1 << V4L2_VP9_SEG_LVL_REF_FRAME;
            dec_params->seg.feature_data[i][V4L2_VP9_SEG_LVL_REF_FRAME] = s->s.h.segmentation.feat[i].ref_val;
        }

        if (s->s.h.segmentation.feat[i].skip_enabled)
            dec_params->seg.feature_enabled[i] |= 1 << V4L2_VP9_SEG_LVL_SKIP;
    }
}

static int v4l2_request_vp9_start_frame(AVCodecContext *avctx,
                                        av_unused const uint8_t *buffer,
                                        av_unused uint32_t size)
{
    const VP9SharedContext *h = avctx->priv_data;
    const VP9Frame *f = &h->frames[CUR_FRAME];
    V4L2RequestControlsVP9 *controls = f->hwaccel_picture_private;
    int ret;

    ret = ff_v4l2_request_start_frame(avctx, &controls->pic, f->tf.f);
    if (ret)
        return ret;

    v4l2_request_vp9_set_frame_ctx(avctx);

    fill_frame(&controls->frame, avctx);

    return 0;
}

static int v4l2_request_vp9_decode_slice(AVCodecContext *avctx,
                                         const uint8_t *buffer, uint32_t size)
{
    const VP9SharedContext *h = avctx->priv_data;
    V4L2RequestControlsVP9 *controls = h->frames[CUR_FRAME].hwaccel_picture_private;

    return ff_v4l2_request_append_output(avctx, &controls->pic, buffer, size);
}

static int v4l2_request_vp9_end_frame(AVCodecContext *avctx)
{
    const VP9SharedContext *h = avctx->priv_data;
    V4L2RequestControlsVP9 *controls = h->frames[CUR_FRAME].hwaccel_picture_private;

    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_STATELESS_VP9_FRAME,
            .ptr = &controls->frame,
            .size = sizeof(controls->frame),
        },
        {
            .id = V4L2_CID_STATELESS_VP9_COMPRESSED_HDR,
            .ptr = &controls->chp,
            .size = sizeof(controls->chp),
        },
    };

    return ff_v4l2_request_decode_frame(avctx, &controls->pic,
                                        control, FF_ARRAY_ELEMS(control));
}

static int v4l2_request_vp9_init(AVCodecContext *avctx)
{
    struct v4l2_ctrl_vp9_frame frame;

    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_STATELESS_VP9_FRAME,
            .ptr = &frame,
            .size = sizeof(frame),
        },
    };

    fill_frame(&frame, avctx);

    // TODO: check V4L2_CID_MPEG_VIDEO_VP9_PROFILE
    // TODO: check V4L2_CID_MPEG_VIDEO_VP9_LEVEL

    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_VP9_FRAME,
                                3 * 1024 * 1024,
                                control, FF_ARRAY_ELEMS(control));
}

const FFHWAccel ff_vp9_v4l2request_hwaccel = {
    .p.name             = "vp9_v4l2request",
    .p.type             = AVMEDIA_TYPE_VIDEO,
    .p.id               = AV_CODEC_ID_VP9,
    .p.pix_fmt          = AV_PIX_FMT_DRM_PRIME,
    .start_frame        = v4l2_request_vp9_start_frame,
    .decode_slice       = v4l2_request_vp9_decode_slice,
    .end_frame          = v4l2_request_vp9_end_frame,
    .flush              = ff_v4l2_request_flush,
    .frame_priv_data_size = sizeof(V4L2RequestControlsVP9),
    .init               = v4l2_request_vp9_init,
    .uninit             = ff_v4l2_request_uninit,
    .priv_data_size     = sizeof(V4L2RequestContext),
    .frame_params       = ff_v4l2_request_frame_params,
};
