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

#include "hwaccel_internal.h"
#include "hwconfig.h"
#include "v4l2_request.h"
#include "vp8.h"

typedef struct V4L2RequestControlsVP8 {
    struct v4l2_ctrl_vp8_frame ctrl;
} V4L2RequestControlsVP8;

static int v4l2_request_vp8_start_frame(AVCodecContext          *avctx,
                                        av_unused const uint8_t *buffer,
                                        av_unused uint32_t       size)
{
    const VP8Context *s = avctx->priv_data;
    V4L2RequestControlsVP8 *controls = s->framep[VP8_FRAME_CURRENT]->hwaccel_picture_private;

    memset(&controls->ctrl, 0, sizeof(controls->ctrl));
    return ff_v4l2_request_reset_frame(avctx, s->framep[VP8_FRAME_CURRENT]->tf.f);
}

static int v4l2_request_vp8_end_frame(AVCodecContext *avctx)
{
    const VP8Context *s = avctx->priv_data;
    V4L2RequestControlsVP8 *controls = s->framep[VP8_FRAME_CURRENT]->hwaccel_picture_private;
    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_STATELESS_VP8_FRAME,
            .ptr = &controls->ctrl,
            .size = sizeof(controls->ctrl),
        },
    };

    return ff_v4l2_request_decode_frame(avctx, s->framep[VP8_FRAME_CURRENT]->tf.f,
                                        control, FF_ARRAY_ELEMS(control));
}

static int v4l2_request_vp8_decode_slice(AVCodecContext *avctx,
                                         const uint8_t *buffer,
                                         uint32_t size)
{
    const VP8Context *s = avctx->priv_data;
    V4L2RequestControlsVP8 *controls = s->framep[VP8_FRAME_CURRENT]->hwaccel_picture_private;
    struct v4l2_ctrl_vp8_frame *frame = &controls->ctrl;
    const uint8_t *data = buffer + 3 + 7 * s->keyframe;
    unsigned int i, j, k;

    frame->version = s->profile & 0x3;
    frame->width = avctx->width;
    frame->height = avctx->height;
    /* FIXME: set ->xx_scale */
    frame->prob_skip_false = s->prob->mbskip;
    frame->prob_intra = s->prob->intra;
    frame->prob_gf = s->prob->golden;
    frame->prob_last = s->prob->last;
    frame->first_part_size = s->header_partition_size;
    frame->first_part_header_bits = (8 * (s->coder_state_at_header_end.input - data) -
                                    s->coder_state_at_header_end.bit_count - 8);
    frame->num_dct_parts = s->num_coeff_partitions;
    for (i = 0; i < 8; i++)
        frame->dct_part_sizes[i] = s->coeff_partition_size[i];

    frame->coder_state.range = s->coder_state_at_header_end.range;
    frame->coder_state.value = s->coder_state_at_header_end.value;
    frame->coder_state.bit_count = s->coder_state_at_header_end.bit_count;
    if (s->framep[VP8_FRAME_PREVIOUS])
        frame->last_frame_ts = ff_v4l2_request_get_capture_timestamp(s->framep[VP8_FRAME_PREVIOUS]->tf.f);
    if (s->framep[VP8_FRAME_GOLDEN])
        frame->golden_frame_ts = ff_v4l2_request_get_capture_timestamp(s->framep[VP8_FRAME_GOLDEN]->tf.f);
    if (s->framep[VP8_FRAME_ALTREF])
        frame->alt_frame_ts = ff_v4l2_request_get_capture_timestamp(s->framep[VP8_FRAME_ALTREF]->tf.f);
    frame->flags |= s->invisible ? 0 : V4L2_VP8_FRAME_FLAG_SHOW_FRAME;
    frame->flags |= s->mbskip_enabled ? V4L2_VP8_FRAME_FLAG_MB_NO_SKIP_COEFF : 0;
    frame->flags |= (s->profile & 0x4) ? V4L2_VP8_FRAME_FLAG_EXPERIMENTAL : 0;
    frame->flags |= s->keyframe ? V4L2_VP8_FRAME_FLAG_KEY_FRAME : 0;
    frame->flags |= s->sign_bias[VP8_FRAME_GOLDEN] ? V4L2_VP8_FRAME_FLAG_SIGN_BIAS_GOLDEN : 0;
    frame->flags |= s->sign_bias[VP8_FRAME_ALTREF] ? V4L2_VP8_FRAME_FLAG_SIGN_BIAS_ALT : 0;
    frame->segment.flags |= s->segmentation.enabled ? V4L2_VP8_SEGMENT_FLAG_ENABLED : 0;
    frame->segment.flags |= s->segmentation.update_map ? V4L2_VP8_SEGMENT_FLAG_UPDATE_MAP : 0;
    frame->segment.flags |= s->segmentation.update_feature_data ? V4L2_VP8_SEGMENT_FLAG_UPDATE_FEATURE_DATA : 0;
    frame->segment.flags |= s->segmentation.absolute_vals ? 0 : V4L2_VP8_SEGMENT_FLAG_DELTA_VALUE_MODE;
    for (i = 0; i < 4; i++) {
        frame->segment.quant_update[i] = s->segmentation.base_quant[i];
        frame->segment.lf_update[i] = s->segmentation.filter_level[i];
    }

    for (i = 0; i < 3; i++)
        frame->segment.segment_probs[i] = s->prob->segmentid[i];

    frame->lf.level = s->filter.level;
    frame->lf.sharpness_level = s->filter.sharpness;
    frame->lf.flags |= s->lf_delta.enabled ? V4L2_VP8_LF_ADJ_ENABLE : 0;
    frame->lf.flags |= s->lf_delta.update ? V4L2_VP8_LF_DELTA_UPDATE : 0;
    frame->lf.flags |= s->filter.simple ? V4L2_VP8_LF_FILTER_TYPE_SIMPLE : 0;
    for (i = 0; i < 4; i++) {
        frame->lf.ref_frm_delta[i] = s->lf_delta.ref[i];
        frame->lf.mb_mode_delta[i] = s->lf_delta.mode[i + MODE_I4x4];
    }

    // Probabilites
    if (s->keyframe) {
        static const uint8_t keyframe_y_mode_probs[4] = {
            145, 156, 163, 128
        };
        static const uint8_t keyframe_uv_mode_probs[3] = {
            142, 114, 183
        };

        memcpy(frame->entropy.y_mode_probs, keyframe_y_mode_probs,  4);
        memcpy(frame->entropy.uv_mode_probs, keyframe_uv_mode_probs, 3);
    } else {
        for (i = 0; i < 4; i++)
            frame->entropy.y_mode_probs[i] = s->prob->pred16x16[i];
        for (i = 0; i < 3; i++)
            frame->entropy.uv_mode_probs[i] = s->prob->pred8x8c[i];
    }
    for (i = 0; i < 2; i++)
        for (j = 0; j < 19; j++)
            frame->entropy.mv_probs[i][j] = s->prob->mvc[i][j];

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 8; j++) {
            static const int coeff_bands_inverse[8] = {
                0, 1, 2, 3, 5, 6, 4, 15
            };
            int coeff_pos = coeff_bands_inverse[j];

            for (k = 0; k < 3; k++) {
                memcpy(frame->entropy.coeff_probs[i][j][k],
                       s->prob->token[i][coeff_pos][k], 11);
            }
        }
    }

    frame->quant.y_ac_qi = s->quant.yac_qi;
    frame->quant.y_dc_delta = s->quant.ydc_delta;
    frame->quant.y2_dc_delta = s->quant.y2dc_delta;
    frame->quant.y2_ac_delta = s->quant.y2ac_delta;
    frame->quant.uv_dc_delta = s->quant.uvdc_delta;
    frame->quant.uv_ac_delta = s->quant.uvac_delta;

    return ff_v4l2_request_append_output_buffer(avctx, s->framep[VP8_FRAME_CURRENT]->tf.f, buffer, size);
}

static int v4l2_request_vp8_init(AVCodecContext *avctx)
{
    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_VP8_FRAME, 2 * 1024 * 1024, NULL, 0);
}

const FFHWAccel ff_vp8_v4l2request_hwaccel = {
    .p.name         = "vp8_v4l2request",
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_VP8,
    .p.pix_fmt      = AV_PIX_FMT_DRM_PRIME,
    .start_frame    = v4l2_request_vp8_start_frame,
    .decode_slice   = v4l2_request_vp8_decode_slice,
    .end_frame      = v4l2_request_vp8_end_frame,
    .frame_priv_data_size = sizeof(V4L2RequestControlsVP8),
    .init           = v4l2_request_vp8_init,
    .uninit         = ff_v4l2_request_uninit,
    .priv_data_size = sizeof(V4L2RequestContext),
    .frame_params   = ff_v4l2_request_frame_params,
    .caps_internal  = HWACCEL_CAP_ASYNC_SAFE,
};
