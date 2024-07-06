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
#include "av1dec.h"
#include "v4l2_request.h"

typedef struct V4L2RequestControlsAV1 {
    V4L2RequestPictureContext pic;
    struct v4l2_ctrl_av1_sequence sequence;
    struct v4l2_ctrl_av1_frame frame;
    struct v4l2_ctrl_av1_film_grain film_grain;
    struct v4l2_ctrl_av1_tile_group_entry *tile_group_entry;
} V4L2RequestControlsAV1;

static int get_bit_depth_from_seq(const AV1RawSequenceHeader *seq)
{
    if (seq->seq_profile == AV_PROFILE_AV1_PROFESSIONAL &&
        seq->color_config.high_bitdepth)
        return seq->color_config.twelve_bit ? 12 : 10;
    else
        return seq->color_config.high_bitdepth ? 10 : 8;
}

static void fill_sequence(struct v4l2_ctrl_av1_sequence *ctrl, const AV1DecContext *s)
{
    const AV1RawSequenceHeader *seq = s->raw_seq;

    *ctrl = (struct v4l2_ctrl_av1_sequence) {
        .seq_profile = seq->seq_profile,
        .order_hint_bits = seq->enable_order_hint ?
                           seq->order_hint_bits_minus_1 + 1 : 0,
        .bit_depth = get_bit_depth_from_seq(seq),
        .max_frame_width_minus_1 = seq->max_frame_width_minus_1,
        .max_frame_height_minus_1 = seq->max_frame_height_minus_1,
    };

    if (seq->still_picture)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_STILL_PICTURE;

    if (seq->use_128x128_superblock)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_USE_128X128_SUPERBLOCK;

    if (seq->enable_filter_intra)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_FILTER_INTRA;

    if (seq->enable_intra_edge_filter)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_INTRA_EDGE_FILTER;

    if (seq->enable_interintra_compound)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_INTERINTRA_COMPOUND;

    if (seq->enable_masked_compound)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_MASKED_COMPOUND;

    if (seq->enable_warped_motion)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_WARPED_MOTION;

    if (seq->enable_dual_filter)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_DUAL_FILTER;

    if (seq->enable_order_hint)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_ORDER_HINT;

    if (seq->enable_jnt_comp)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_JNT_COMP;

    if (seq->enable_ref_frame_mvs)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_REF_FRAME_MVS;

    if (seq->enable_superres)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_SUPERRES;

    if (seq->enable_cdef)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_CDEF;

    if (seq->enable_restoration)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_RESTORATION;

    if (seq->color_config.mono_chrome)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_MONO_CHROME;

    if (seq->color_config.color_range)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_COLOR_RANGE;

    if (seq->color_config.subsampling_x)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_X;

    if (seq->color_config.subsampling_y)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_Y;

    if (seq->film_grain_params_present)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_FILM_GRAIN_PARAMS_PRESENT;

    if (seq->color_config.separate_uv_delta_q)
        ctrl->flags |= V4L2_AV1_SEQUENCE_FLAG_SEPARATE_UV_DELTA_Q;
}

static void fill_frame(struct v4l2_ctrl_av1_frame *ctrl, const AV1DecContext *s)
{
    const AV1RawFrameHeader *frame_header = s->raw_frame_header;

    *ctrl = (struct v4l2_ctrl_av1_frame) {
        .superres_denom = frame_header->use_superres ?
                          frame_header->coded_denom + AV1_SUPERRES_DENOM_MIN :
                          AV1_SUPERRES_NUM,
        .primary_ref_frame = frame_header->primary_ref_frame,
        .frame_type = frame_header->frame_type,
        .order_hint = frame_header->order_hint,
        //.upscaled_width = frame_header->upscaled_width,
        .interpolation_filter = frame_header->interpolation_filter,
        .tx_mode = frame_header->tx_mode,
        .frame_width_minus_1 = frame_header->frame_width_minus_1,
        .frame_height_minus_1 = frame_header->frame_height_minus_1,
        .render_width_minus_1 = frame_header->render_width_minus_1,
        .render_height_minus_1 = frame_header->render_height_minus_1,
        .current_frame_id = frame_header->current_frame_id,
        .refresh_frame_flags = frame_header->refresh_frame_flags,
    };

    if (frame_header->show_frame)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_SHOW_FRAME;

    if (frame_header->showable_frame)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_SHOWABLE_FRAME;

    if (frame_header->error_resilient_mode)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_ERROR_RESILIENT_MODE;

    if (frame_header->disable_cdf_update)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_DISABLE_CDF_UPDATE;

    if (frame_header->allow_screen_content_tools)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_ALLOW_SCREEN_CONTENT_TOOLS;

    if (frame_header->force_integer_mv)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_FORCE_INTEGER_MV;

    if (frame_header->allow_intrabc)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_ALLOW_INTRABC;

    if (frame_header->use_superres)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_USE_SUPERRES;

    if (frame_header->allow_high_precision_mv)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_ALLOW_HIGH_PRECISION_MV;

    if (frame_header->is_motion_mode_switchable)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_IS_MOTION_MODE_SWITCHABLE;

    if (frame_header->use_ref_frame_mvs)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_USE_REF_FRAME_MVS;

    if (frame_header->disable_frame_end_update_cdf)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_DISABLE_FRAME_END_UPDATE_CDF;

    if (frame_header->allow_warped_motion)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_ALLOW_WARPED_MOTION;

    if (frame_header->reference_select)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_REFERENCE_SELECT;

    if (frame_header->reduced_tx_set)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_REDUCED_TX_SET;

    if (frame_header->skip_mode_present && s->cur_frame.skip_mode_frame_idx[0] > 0)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_SKIP_MODE_ALLOWED;

    if (frame_header->skip_mode_present)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_SKIP_MODE_PRESENT;

    if (frame_header->frame_size_override_flag)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_FRAME_SIZE_OVERRIDE;

    if (frame_header->buffer_removal_time_present_flag)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_BUFFER_REMOVAL_TIME_PRESENT;

    if (frame_header->frame_refs_short_signaling)
        ctrl->flags |= V4L2_AV1_FRAME_FLAG_FRAME_REFS_SHORT_SIGNALING;
}

static int v4l2_request_av1_start_frame(AVCodecContext *avctx,
                                        av_unused const uint8_t *buffer,
                                        av_unused uint32_t size)
{
    const AV1DecContext *s = avctx->priv_data;
    V4L2RequestControlsAV1 *controls = s->cur_frame.hwaccel_picture_private;
    int ret;

    ret = ff_v4l2_request_start_frame(avctx, &controls->pic, s->cur_frame.f);
    if (ret)
        return ret;

    fill_sequence(&controls->sequence, s);
    fill_frame(&controls->frame, s);

    return 0;
}

static int v4l2_request_av1_decode_slice(AVCodecContext *avctx, const uint8_t *buffer, uint32_t size)
{
    const AV1DecContext *s = avctx->priv_data;
    V4L2RequestControlsAV1 *controls = s->cur_frame.hwaccel_picture_private;

    return ff_v4l2_request_append_output(avctx, &controls->pic, buffer, size);
}

static int v4l2_request_av1_end_frame(AVCodecContext *avctx)
{
    const AV1DecContext *s = avctx->priv_data;
    V4L2RequestControlsAV1 *controls = s->cur_frame.hwaccel_picture_private;

    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_STATELESS_AV1_SEQUENCE,
            .ptr = &controls->sequence,
            .size = sizeof(controls->sequence),
        },
        {
            .id = V4L2_CID_STATELESS_AV1_FRAME,
            .ptr = &controls->frame,
            .size = sizeof(controls->frame),
        },
        {
            .id = V4L2_CID_STATELESS_AV1_FILM_GRAIN,
            .ptr = &controls->film_grain,
            .size = sizeof(controls->film_grain),
        },
        // FIXME: tile_group_entry
    };

    return ff_v4l2_request_decode_frame(avctx, &controls->pic,
                                        control, FF_ARRAY_ELEMS(control));
}

static int v4l2_request_av1_init(AVCodecContext *avctx)
{
    const AV1DecContext *s = avctx->priv_data;
    struct v4l2_ctrl_av1_sequence sequence;

    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_STATELESS_AV1_SEQUENCE,
            .ptr = &sequence,
            .size = sizeof(sequence),
        },
    };

    fill_sequence(&sequence, s);

    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_AV1_FRAME,
                                3 * 1024 * 1024,
                                control, FF_ARRAY_ELEMS(control));
}

const FFHWAccel ff_av1_v4l2request_hwaccel = {
    .p.name             = "av1_v4l2request",
    .p.type             = AVMEDIA_TYPE_VIDEO,
    .p.id               = AV_CODEC_ID_AV1,
    .p.pix_fmt          = AV_PIX_FMT_DRM_PRIME,
    .start_frame        = v4l2_request_av1_start_frame,
    .decode_slice       = v4l2_request_av1_decode_slice,
    .end_frame          = v4l2_request_av1_end_frame,
    .flush              = ff_v4l2_request_flush,
    .frame_priv_data_size = sizeof(V4L2RequestControlsAV1),
    .init               = v4l2_request_av1_init,
    .uninit             = ff_v4l2_request_uninit,
    .priv_data_size     = sizeof(V4L2RequestContext),
    .frame_params       = ff_v4l2_request_frame_params,
};
