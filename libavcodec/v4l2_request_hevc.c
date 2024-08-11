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

#include "hevcdec.h"
#include "hwconfig.h"
#include "internal.h"
#include "v4l2_request.h"

#define V4L2_HEVC_CONTROLS_MAX 6

typedef struct V4L2RequestContextHEVC {
    V4L2RequestContext base;
    enum v4l2_stateless_hevc_decode_mode decode_mode;
    enum v4l2_stateless_hevc_start_code start_code;
    unsigned int max_slice_params;
    unsigned int max_entry_point_offsets;
    bool has_scaling_matrix;
} V4L2RequestContextHEVC;

typedef struct V4L2RequestControlsHEVC {
    V4L2RequestPictureContext pic;
    struct v4l2_ctrl_hevc_sps sps;
    struct v4l2_ctrl_hevc_pps pps;
    struct v4l2_ctrl_hevc_decode_params decode_params;
    struct v4l2_ctrl_hevc_scaling_matrix scaling_matrix;
    struct v4l2_ctrl_hevc_slice_params slice_params;
    struct v4l2_ctrl_hevc_slice_params *frame_slice_params;
    unsigned int allocated_slice_params;
    unsigned int num_slice_params;
    uint32_t *entry_point_offsets;
    unsigned int allocated_entry_point_offsets;
    unsigned int num_entry_point_offsets;
    bool first_slice;
} V4L2RequestControlsHEVC;

static uint8_t nalu_slice_start_code[] = { 0x00, 0x00, 0x01 };

static void fill_pred_weight_table(struct v4l2_hevc_pred_weight_table *table,
                                   const HEVCContext *h)
{
    int32_t luma_weight_denom, chroma_weight_denom;
    const SliceHeader *sh = &h->sh;

    if (sh->slice_type == HEVC_SLICE_I ||
        (sh->slice_type == HEVC_SLICE_P && !h->ps.pps->weighted_pred_flag) ||
        (sh->slice_type == HEVC_SLICE_B && !h->ps.pps->weighted_bipred_flag))
        return;

    table->luma_log2_weight_denom = sh->luma_log2_weight_denom;

    if (h->ps.sps->chroma_format_idc)
        table->delta_chroma_log2_weight_denom = sh->chroma_log2_weight_denom -
                                                sh->luma_log2_weight_denom;

    luma_weight_denom = (1 << sh->luma_log2_weight_denom);
    chroma_weight_denom = (1 << sh->chroma_log2_weight_denom);

    for (int i = 0; i < 15 && i < sh->nb_refs[L0]; i++) {
        table->delta_luma_weight_l0[i] = sh->luma_weight_l0[i] - luma_weight_denom;
        table->luma_offset_l0[i] = sh->luma_offset_l0[i];
        table->delta_chroma_weight_l0[i][0] = sh->chroma_weight_l0[i][0] - chroma_weight_denom;
        table->delta_chroma_weight_l0[i][1] = sh->chroma_weight_l0[i][1] - chroma_weight_denom;
        table->chroma_offset_l0[i][0] = sh->chroma_offset_l0[i][0];
        table->chroma_offset_l0[i][1] = sh->chroma_offset_l0[i][1];
    }

    if (sh->slice_type != HEVC_SLICE_B)
        return;

    for (int i = 0; i < 15 && i < sh->nb_refs[L1]; i++) {
        table->delta_luma_weight_l1[i] = sh->luma_weight_l1[i] - luma_weight_denom;
        table->luma_offset_l1[i] = sh->luma_offset_l1[i];
        table->delta_chroma_weight_l1[i][0] = sh->chroma_weight_l1[i][0] - chroma_weight_denom;
        table->delta_chroma_weight_l1[i][1] = sh->chroma_weight_l1[i][1] - chroma_weight_denom;
        table->chroma_offset_l1[i][0] = sh->chroma_offset_l1[i][0];
        table->chroma_offset_l1[i][1] = sh->chroma_offset_l1[i][1];
    }
}

static uint8_t get_ref_pic_index(const HEVCContext *h, const HEVCFrame *frame,
                                 struct v4l2_ctrl_hevc_decode_params *decode_params)
{
    uint64_t timestamp;

    if (!frame || !frame->frame)
        return 0;

    timestamp = ff_v4l2_request_get_capture_timestamp(frame->frame);

    for (uint8_t i = 0; i < decode_params->num_active_dpb_entries; i++) {
        struct v4l2_hevc_dpb_entry *entry = &decode_params->dpb[i];
        if (entry->timestamp == timestamp)
            return i;
    }

    return 0;
}

static void fill_decode_params(struct v4l2_ctrl_hevc_decode_params *decode_params,
                               const HEVCContext *h)
{
    const HEVCFrame *pic = h->ref;
    const SliceHeader *sh = &h->sh;
    int i, entries = 0;

    *decode_params = (struct v4l2_ctrl_hevc_decode_params) {
        .pic_order_cnt_val = pic->poc, /* FIXME: is this same as slice_params->slice_pic_order_cnt ? */
        .short_term_ref_pic_set_size = sh->short_term_ref_pic_set_size,
        .long_term_ref_pic_set_size = sh->long_term_ref_pic_set_size,
        .num_poc_st_curr_before = h->rps[ST_CURR_BEF].nb_refs,
        .num_poc_st_curr_after = h->rps[ST_CURR_AFT].nb_refs,
        .num_poc_lt_curr = h->rps[LT_CURR].nb_refs,
    };

#if HAVE_STRUCT_V4L2_CTRL_HEVC_DECODE_PARAMS_NUM_DELTA_POCS_OF_REF_RPS_IDX
    if (h->sh.short_term_ref_pic_set_sps_flag == 0 && h->sh.short_term_rps)
        decode_params->num_delta_pocs_of_ref_rps_idx =
                                h->sh.short_term_rps->rps_idx_num_delta_pocs;
#endif

    for (i = 0; i < FF_ARRAY_ELEMS(h->DPB); i++) {
        const HEVCFrame *frame = &h->DPB[i];
        if (frame != pic &&
            (frame->flags & (HEVC_FRAME_FLAG_LONG_REF | HEVC_FRAME_FLAG_SHORT_REF))) {
            struct v4l2_hevc_dpb_entry *entry = &decode_params->dpb[entries++];

            entry->timestamp = ff_v4l2_request_get_capture_timestamp(frame->frame);
            entry->field_pic = frame->frame->interlaced_frame;
            entry->flags = 0;
            if (frame->flags & HEVC_FRAME_FLAG_LONG_REF)
                entry->flags |= V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE;

            entry->pic_order_cnt_val = frame->poc;
        }
    }

    decode_params->num_active_dpb_entries = entries;

    if (IS_IRAP(h))
        decode_params->flags |= V4L2_HEVC_DECODE_PARAM_FLAG_IRAP_PIC;

    if (IS_IDR(h))
        decode_params->flags |= V4L2_HEVC_DECODE_PARAM_FLAG_IDR_PIC;

    if (sh->no_output_of_prior_pics_flag)
        decode_params->flags |= V4L2_HEVC_DECODE_PARAM_FLAG_NO_OUTPUT_OF_PRIOR;

    for (i = 0; i < V4L2_HEVC_DPB_ENTRIES_NUM_MAX; i++) {
        decode_params->poc_st_curr_before[i] =
            get_ref_pic_index(h, h->rps[ST_CURR_BEF].ref[i], decode_params);
        decode_params->poc_st_curr_after[i] =
            get_ref_pic_index(h, h->rps[ST_CURR_AFT].ref[i], decode_params);
        decode_params->poc_lt_curr[i] =
            get_ref_pic_index(h, h->rps[LT_CURR].ref[i], decode_params);
    }
}

static int fill_slice_params(V4L2RequestControlsHEVC *controls, int slice,
                             bool max_entry_point_offsets, const HEVCContext *h)
{
    struct v4l2_ctrl_hevc_slice_params *slice_params = &controls->frame_slice_params[slice];
    struct v4l2_ctrl_hevc_decode_params *decode_params = &controls->decode_params;
    const HEVCFrame *pic = h->ref;
    const SliceHeader *sh = &h->sh;
    RefPicList *rpl;
    int i, offsets;

    *slice_params = (struct v4l2_ctrl_hevc_slice_params) {
        .bit_size = 0,
        .data_byte_offset = (get_bits_count(&h->HEVClc->gb) + 1 + 7) / 8,
        .num_entry_point_offsets = sh->num_entry_point_offsets,

        /* ISO/IEC 23008-2, ITU-T Rec. H.265: NAL unit header */
        .nal_unit_type = h->nal_unit_type,
        .nuh_temporal_id_plus1 = h->temporal_id + 1,

        /* ISO/IEC 23008-2, ITU-T Rec. H.265: General slice segment header */
        .slice_type = sh->slice_type,
        .colour_plane_id = sh->colour_plane_id,
        .slice_pic_order_cnt = pic->poc,
        .num_ref_idx_l0_active_minus1 = sh->nb_refs[L0] ? sh->nb_refs[L0] - 1 : 0,
        .num_ref_idx_l1_active_minus1 = sh->nb_refs[L1] ? sh->nb_refs[L1] - 1 : 0,
        .collocated_ref_idx = sh->slice_temporal_mvp_enabled_flag ?
                              sh->collocated_ref_idx : 0,
        .five_minus_max_num_merge_cand = sh->slice_type == HEVC_SLICE_I ?
                                         0 : 5 - sh->max_num_merge_cand,
        .slice_qp_delta = sh->slice_qp_delta,
        .slice_cb_qp_offset = sh->slice_cb_qp_offset,
        .slice_cr_qp_offset = sh->slice_cr_qp_offset,
        .slice_act_y_qp_offset = 0,
        .slice_act_cb_qp_offset = 0,
        .slice_act_cr_qp_offset = 0,
        .slice_beta_offset_div2 = sh->beta_offset / 2,
        .slice_tc_offset_div2 = sh->tc_offset / 2,

        /* ISO/IEC 23008-2, ITU-T Rec. H.265: Picture timing SEI message */
        .pic_struct = h->sei.picture_timing.picture_struct,

        /* ISO/IEC 23008-2, ITU-T Rec. H.265: General slice segment header */
        .slice_segment_addr = sh->slice_segment_addr,
        .short_term_ref_pic_set_size = sh->short_term_ref_pic_set_size,
        .long_term_ref_pic_set_size = sh->long_term_ref_pic_set_size,
    };

    if (sh->slice_sample_adaptive_offset_flag[0])
        slice_params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA;

    if (sh->slice_sample_adaptive_offset_flag[1])
        slice_params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA;

    if (sh->slice_temporal_mvp_enabled_flag)
        slice_params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED;

    if (sh->mvd_l1_zero_flag)
        slice_params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_MVD_L1_ZERO;

    if (sh->cabac_init_flag)
        slice_params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_CABAC_INIT;

    if (sh->collocated_list == L0)
        slice_params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0;

    /* TODO: V4L2_HEVC_SLICE_PARAMS_FLAG_USE_INTEGER_MV */

    if (sh->disable_deblocking_filter_flag)
        slice_params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED;

    if (sh->slice_loop_filter_across_slices_enabled_flag)
        slice_params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED;

    if (sh->dependent_slice_segment_flag)
        slice_params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT;

    if (sh->slice_type != HEVC_SLICE_I) {
        rpl = &h->ref->refPicList[0];
        for (i = 0; i < rpl->nb_refs; i++)
            slice_params->ref_idx_l0[i] = get_ref_pic_index(h, rpl->ref[i], decode_params);
    }

    if (sh->slice_type == HEVC_SLICE_B) {
        rpl = &h->ref->refPicList[1];
        for (i = 0; i < rpl->nb_refs; i++)
            slice_params->ref_idx_l1[i] = get_ref_pic_index(h, rpl->ref[i], decode_params);
    }

    fill_pred_weight_table(&slice_params->pred_weight_table, h);

    if (!max_entry_point_offsets)
        return 0;

    if (controls->allocated_entry_point_offsets < controls->num_entry_point_offsets + sh->num_entry_point_offsets) {
        void *entry_point_offsets = controls->entry_point_offsets;
        offsets = controls->allocated_entry_point_offsets == 0 ? 128 : controls->allocated_entry_point_offsets * 2;
        while (controls->num_entry_point_offsets + sh->num_entry_point_offsets > offsets)
            offsets *= 2;
        entry_point_offsets = av_realloc_array(entry_point_offsets, offsets, sizeof(*controls->entry_point_offsets));
        if (!entry_point_offsets)
            return AVERROR(ENOMEM);
        controls->entry_point_offsets = entry_point_offsets;
        controls->allocated_entry_point_offsets = offsets;
    }

    for (i = 0, offsets = controls->num_entry_point_offsets; i < sh->num_entry_point_offsets; i++)
        controls->entry_point_offsets[offsets + i] = sh->entry_point_offset[i];
    controls->num_entry_point_offsets += sh->num_entry_point_offsets;

    return 0;
}

static void fill_sps(struct v4l2_ctrl_hevc_sps *ctrl, const HEVCContext *h)
{
    const HEVCPPS *pps = h->ps.pps;
    const HEVCSPS *sps = h->ps.sps;

    /* ISO/IEC 23008-2, ITU-T Rec. H.265: Sequence parameter set */
    *ctrl = (struct v4l2_ctrl_hevc_sps) {
        .video_parameter_set_id = sps->vps_id,
        .seq_parameter_set_id = pps->sps_id,
        .pic_width_in_luma_samples = sps->width,
        .pic_height_in_luma_samples = sps->height,
        .bit_depth_luma_minus8 = sps->bit_depth - 8,
        .bit_depth_chroma_minus8 = sps->bit_depth_chroma - 8,
        .log2_max_pic_order_cnt_lsb_minus4 = sps->log2_max_poc_lsb - 4,
        .sps_max_dec_pic_buffering_minus1 =
            sps->temporal_layer[sps->max_sub_layers - 1].max_dec_pic_buffering - 1,
        .sps_max_num_reorder_pics =
            sps->temporal_layer[sps->max_sub_layers - 1].num_reorder_pics,
        .sps_max_latency_increase_plus1 =
            sps->temporal_layer[sps->max_sub_layers - 1].max_latency_increase + 1,
        .log2_min_luma_coding_block_size_minus3 = sps->log2_min_cb_size - 3,
        .log2_diff_max_min_luma_coding_block_size =
            sps->log2_diff_max_min_coding_block_size,
        .log2_min_luma_transform_block_size_minus2 = sps->log2_min_tb_size - 2,
        .log2_diff_max_min_luma_transform_block_size =
            sps->log2_max_trafo_size - sps->log2_min_tb_size,
        .max_transform_hierarchy_depth_inter = sps->max_transform_hierarchy_depth_inter,
        .max_transform_hierarchy_depth_intra = sps->max_transform_hierarchy_depth_intra,
        .pcm_sample_bit_depth_luma_minus1 = sps->pcm.bit_depth - 1,
        .pcm_sample_bit_depth_chroma_minus1 = sps->pcm.bit_depth_chroma - 1,
        .log2_min_pcm_luma_coding_block_size_minus3 = sps->pcm.log2_min_pcm_cb_size - 3,
        .log2_diff_max_min_pcm_luma_coding_block_size =
            sps->pcm.log2_max_pcm_cb_size - sps->pcm.log2_min_pcm_cb_size,
        .num_short_term_ref_pic_sets = sps->nb_st_rps,
        .num_long_term_ref_pics_sps = sps->num_long_term_ref_pics_sps,
        .chroma_format_idc = sps->chroma_format_idc,
        .sps_max_sub_layers_minus1 = sps->max_sub_layers - 1,
    };

    if (sps->separate_colour_plane_flag)
        ctrl->flags |= V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE;

    if (sps->scaling_list_enable_flag)
        ctrl->flags |= V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED;

    if (sps->amp_enabled_flag)
        ctrl->flags |= V4L2_HEVC_SPS_FLAG_AMP_ENABLED;

    if (sps->sao_enabled)
        ctrl->flags |= V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET;

    if (sps->pcm_enabled_flag)
        ctrl->flags |= V4L2_HEVC_SPS_FLAG_PCM_ENABLED;

    if (sps->pcm.loop_filter_disable_flag)
        ctrl->flags |= V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED;

    if (sps->long_term_ref_pics_present_flag)
        ctrl->flags |= V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT;

    if (sps->sps_temporal_mvp_enabled_flag)
        ctrl->flags |= V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED;

    if (sps->sps_strong_intra_smoothing_enable_flag)
        ctrl->flags |= V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED;
}

static int v4l2_request_hevc_start_frame(AVCodecContext *avctx,
                                         av_unused const uint8_t *buffer,
                                         av_unused uint32_t size)
{
    const HEVCContext *h = avctx->priv_data;
    const HEVCPPS *pps = h->ps.pps;
    const HEVCSPS *sps = h->ps.sps;
    V4L2RequestContextHEVC *ctx = avctx->internal->hwaccel_priv_data;
    V4L2RequestControlsHEVC *controls = h->ref->hwaccel_picture_private;
    const SliceHeader *sh = &h->sh;
    int ret;

    ret = ff_v4l2_request_start_frame(avctx, &controls->pic, h->ref->frame);
    if (ret)
        return ret;

    fill_sps(&controls->sps, h);
    fill_decode_params(&controls->decode_params, h);

    if (ctx->has_scaling_matrix) {
        const ScalingList *sl = pps->scaling_list_data_present_flag ?
                                &pps->scaling_list :
                                sps->scaling_list_enable_flag ?
                                &sps->scaling_list : NULL;
        if (sl) {
            for (int i = 0; i < 6; i++) {
                for (int j = 0; j < 16; j++)
                    controls->scaling_matrix.scaling_list_4x4[i][j] = sl->sl[0][i][j];
                for (int j = 0; j < 64; j++) {
                    controls->scaling_matrix.scaling_list_8x8[i][j]   = sl->sl[1][i][j];
                    controls->scaling_matrix.scaling_list_16x16[i][j] = sl->sl[2][i][j];
                    if (i < 2)
                        controls->scaling_matrix.scaling_list_32x32[i][j] = sl->sl[3][i * 3][j];
                }
                controls->scaling_matrix.scaling_list_dc_coef_16x16[i] = sl->sl_dc[0][i];
                if (i < 2)
                    controls->scaling_matrix.scaling_list_dc_coef_32x32[i] = sl->sl_dc[1][i * 3];
            }
        }
    }

    /* ISO/IEC 23008-2, ITU-T Rec. H.265: Picture parameter set */
    controls->pps = (struct v4l2_ctrl_hevc_pps) {
        .pic_parameter_set_id = sh->pps_id,
        .num_extra_slice_header_bits = pps->num_extra_slice_header_bits,
        .num_ref_idx_l0_default_active_minus1 = pps->num_ref_idx_l0_default_active - 1,
        .num_ref_idx_l1_default_active_minus1 = pps->num_ref_idx_l1_default_active - 1,
        .init_qp_minus26 = pps->pic_init_qp_minus26,
        .diff_cu_qp_delta_depth = pps->diff_cu_qp_delta_depth,
        .pps_cb_qp_offset = pps->cb_qp_offset,
        .pps_cr_qp_offset = pps->cr_qp_offset,
        .pps_beta_offset_div2 = pps->beta_offset / 2,
        .pps_tc_offset_div2 = pps->tc_offset / 2,
        .log2_parallel_merge_level_minus2 = pps->log2_parallel_merge_level - 2,
    };

    if (pps->dependent_slice_segments_enabled_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED;

    if (pps->output_flag_present_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT;

    if (pps->sign_data_hiding_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED;

    if (pps->cabac_init_present_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT;

    if (pps->constrained_intra_pred_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED;

    if (pps->transform_skip_enabled_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED;

    if (pps->cu_qp_delta_enabled_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED;

    if (pps->pic_slice_level_chroma_qp_offsets_present_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT;

    if (pps->weighted_pred_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED;

    if (pps->weighted_bipred_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED;

    if (pps->transquant_bypass_enable_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED;

    if (pps->tiles_enabled_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_TILES_ENABLED;

    if (pps->entropy_coding_sync_enabled_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED;

    if (pps->loop_filter_across_tiles_enabled_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED;

    if (pps->seq_loop_filter_across_slices_enabled_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED;

    if (pps->deblocking_filter_override_enabled_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED;

    if (pps->disable_dbf)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER;

    if (pps->lists_modification_present_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT;

    if (pps->slice_header_extension_present_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT;

    if (pps->deblocking_filter_control_present_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT;

    if (pps->uniform_spacing_flag)
        controls->pps.flags |= V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING;

    if (pps->tiles_enabled_flag) {
        controls->pps.num_tile_columns_minus1 = pps->num_tile_columns - 1;
        controls->pps.num_tile_rows_minus1 = pps->num_tile_rows - 1;

        for (int i = 0; i < pps->num_tile_columns; i++)
            controls->pps.column_width_minus1[i] = pps->column_width[i] - 1;

        for (int i = 0; i < pps->num_tile_rows; i++)
            controls->pps.row_height_minus1[i] = pps->row_height[i] - 1;
    }

    controls->first_slice = true;
    controls->frame_slice_params = &controls->slice_params;
    controls->allocated_slice_params = 0;
    controls->num_slice_params = 0;
    controls->allocated_entry_point_offsets = 0;
    controls->num_entry_point_offsets = 0;

    return 0;
}

static int v4l2_request_hevc_queue_decode(AVCodecContext *avctx, bool last_slice)
{
    const HEVCContext *h = avctx->priv_data;
    V4L2RequestContextHEVC *ctx = avctx->internal->hwaccel_priv_data;
    V4L2RequestControlsHEVC *controls = h->ref->hwaccel_picture_private;
    int count = 0;

    struct v4l2_ext_control control[V4L2_HEVC_CONTROLS_MAX] = {};

    control[count++] = (struct v4l2_ext_control) {
        .id = V4L2_CID_STATELESS_HEVC_SPS,
        .ptr = &controls->sps,
        .size = sizeof(controls->sps),
    };

    control[count++] = (struct v4l2_ext_control) {
        .id = V4L2_CID_STATELESS_HEVC_PPS,
        .ptr = &controls->pps,
        .size = sizeof(controls->pps),
    };

    control[count++] = (struct v4l2_ext_control) {
        .id = V4L2_CID_STATELESS_HEVC_DECODE_PARAMS,
        .ptr = &controls->decode_params,
        .size = sizeof(controls->decode_params),
    };

    if (ctx->has_scaling_matrix) {
        control[count++] = (struct v4l2_ext_control) {
            .id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX,
            .ptr = &controls->scaling_matrix,
            .size = sizeof(controls->scaling_matrix),
        };
    }

    if (ctx->max_slice_params && controls->num_slice_params) {
        control[count++] = (struct v4l2_ext_control) {
            .id = V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
            .ptr = controls->frame_slice_params,
            .size = sizeof(*controls->frame_slice_params) *
                    FFMIN(controls->num_slice_params, ctx->max_slice_params),
        };
    }

    if (ctx->max_entry_point_offsets && controls->num_entry_point_offsets) {
        control[count++] = (struct v4l2_ext_control) {
            .id = V4L2_CID_STATELESS_HEVC_ENTRY_POINT_OFFSETS,
            .ptr = controls->entry_point_offsets,
            .size = sizeof(*controls->entry_point_offsets) *
                    FFMIN(controls->num_entry_point_offsets,
                          ctx->max_entry_point_offsets),
        };
    }

    if (ctx->decode_mode == V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED)
        return ff_v4l2_request_decode_slice(avctx, &controls->pic, control, count,
                                            controls->first_slice, last_slice);

    return ff_v4l2_request_decode_frame(avctx, &controls->pic, control, count);
}

static int v4l2_request_hevc_decode_slice(AVCodecContext *avctx,
                                          const uint8_t *buffer, uint32_t size)
{
    const HEVCContext *h = avctx->priv_data;
    V4L2RequestContextHEVC *ctx = avctx->internal->hwaccel_priv_data;
    V4L2RequestControlsHEVC *controls = h->ref->hwaccel_picture_private;
    const SliceHeader *sh = &h->sh;
    int ret, slice = controls->num_slice_params;
    uint32_t extra_size = 0;

    if (ctx->decode_mode == V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED &&
        (slice >= ctx->max_slice_params || (ctx->max_entry_point_offsets &&
         (controls->num_entry_point_offsets + sh->num_entry_point_offsets > ctx->max_entry_point_offsets)))) {
        ret = v4l2_request_hevc_queue_decode(avctx, false);
        if (ret)
            return ret;

        ff_v4l2_request_reset_picture(avctx, &controls->pic);
        slice = controls->num_slice_params = 0;
        controls->num_entry_point_offsets = 0;
        controls->first_slice = false;
    }

    if (ctx->max_slice_params) {
        if (slice && controls->allocated_slice_params < slice + 1) {
            void *slice_params = controls->allocated_slice_params == 0 ? NULL : controls->frame_slice_params;
            int slices = controls->allocated_slice_params == 0 ? 8 : controls->allocated_slice_params * 2;
            slice_params = av_realloc_array(slice_params, slices, sizeof(*controls->frame_slice_params));
            if (!slice_params)
                return AVERROR(ENOMEM);
            if (controls->allocated_slice_params == 0)
                memcpy(slice_params, controls->frame_slice_params, sizeof(*controls->frame_slice_params));
            controls->frame_slice_params = slice_params;
            controls->allocated_slice_params = slices;
        }

        ret = fill_slice_params(controls, slice, !!ctx->max_entry_point_offsets, h);
        if (ret)
            return ret;
    }

    if (ctx->start_code == V4L2_STATELESS_HEVC_START_CODE_ANNEX_B) {
        ret = ff_v4l2_request_append_output(avctx, &controls->pic,
                                            nalu_slice_start_code, 3);
        if (ret)
            return ret;
        extra_size = 3;
    }

    ret = ff_v4l2_request_append_output(avctx, &controls->pic, buffer, size);
    if (ret)
        return ret;

    if (ctx->max_slice_params)
        controls->frame_slice_params[slice].bit_size = (size + extra_size) * 8;

    controls->num_slice_params++;
    return 0;
}

static int v4l2_request_hevc_end_frame(AVCodecContext *avctx)
{
    const HEVCContext *h = avctx->priv_data;
    V4L2RequestControlsHEVC *controls = h->ref->hwaccel_picture_private;
    int ret;

    ret = v4l2_request_hevc_queue_decode(avctx, true);

    if (controls->allocated_slice_params)
        av_freep(&controls->frame_slice_params);

    av_freep(&controls->entry_point_offsets);

    return ret;
}

static int v4l2_request_hevc_post_probe(AVCodecContext *avctx)
{
    V4L2RequestContextHEVC *ctx = avctx->internal->hwaccel_priv_data;
    int ret;

    struct v4l2_ext_control control[] = {
        { .id = V4L2_CID_STATELESS_HEVC_DECODE_MODE, },
        { .id = V4L2_CID_STATELESS_HEVC_START_CODE, },
    };
    struct v4l2_query_ext_ctrl scaling_matrix = {
        .id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX,
    };
    struct v4l2_query_ext_ctrl entry_point_offsets = {
        .id = V4L2_CID_STATELESS_HEVC_ENTRY_POINT_OFFSETS,
    };
    struct v4l2_query_ext_ctrl slice_params = {
        .id = V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
    };

    ctx->decode_mode = ff_v4l2_request_query_control_default_value(avctx,
                                        V4L2_CID_STATELESS_HEVC_DECODE_MODE);
    if (ctx->decode_mode != V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED &&
        ctx->decode_mode != V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED) {
        av_log(ctx, AV_LOG_VERBOSE, "Unsupported decode mode: %d\n",
               ctx->decode_mode);
        return AVERROR(EINVAL);
    }

    ctx->start_code = ff_v4l2_request_query_control_default_value(avctx,
                                        V4L2_CID_STATELESS_HEVC_START_CODE);
    if (ctx->start_code != V4L2_STATELESS_HEVC_START_CODE_NONE &&
        ctx->start_code != V4L2_STATELESS_HEVC_START_CODE_ANNEX_B) {
        av_log(ctx, AV_LOG_VERBOSE, "Unsupported start code: %d\n",
               ctx->start_code);
        return AVERROR(EINVAL);
    }

    // TODO: check V4L2_CID_MPEG_VIDEO_HEVC_PROFILE control
    // TODO: check V4L2_CID_MPEG_VIDEO_HEVC_LEVEL control

    ret = ff_v4l2_request_query_control(avctx, &scaling_matrix);
    if (!ret)
        ctx->has_scaling_matrix = true;
    else
        ctx->has_scaling_matrix = false;

    ret = ff_v4l2_request_query_control(avctx, &entry_point_offsets);
    if (!ret)
        ctx->max_entry_point_offsets = FFMAX(entry_point_offsets.dims[0], 1);
    else
        ctx->max_entry_point_offsets = 0;

    ret = ff_v4l2_request_query_control(avctx, &slice_params);
    if (!ret) {
        ctx->max_slice_params = FFMAX(slice_params.dims[0], 1);

        /*
         * There is some uncertainty on what value to use for data_byte_offset
         * when a driver support multiple slice params. Documentation state:
         *
         *   Offset (in bytes) to the video data in the current slice data.
         *
         * This hwaccel implementation uses the value of sh->data_offset for
         * each slice, e.g. the offset in the specific slice data.
         *
         * However, downstream rpivid driver, first driver to use multiple
         * slice params for a slice-based decoder, instead expects that the
         * value matches the accumulated offset in the output buffer.
         *
         * Limit to only support a single slice when using slice-based decoding
         * until this uncertainty has settled.
         */
        if (ctx->max_slice_params > 1 &&
            ctx->decode_mode == V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED)
            ctx->max_slice_params = 1;
    } else {
        ctx->max_slice_params = 0;

        /*
         * NOTE: Is next two checks something we need or should care about?
         *
         * The code here adapts and only sets slice params control based on
         * value of max_slice_params, regardless of what decode mode is used.
         *
         * For a slice-based decoder having support for the slice params
         * control is more than likely a requirement, however not something we
         * need to enforce here.
         */
        if (ctx->decode_mode == V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED) {
            av_log(ctx, AV_LOG_ERROR,
                   "Slice-based decoder, "
                   "required SLICE_PARAMS control is missing\n");
            return AVERROR(EINVAL);
        }

        if (ctx->max_entry_point_offsets) {
            av_log(ctx, AV_LOG_ERROR,
                   "Decoder with ENTRY_POINT_OFFSETS control, "
                   "required SLICE_PARAMS control is missing\n");
            return AVERROR(EINVAL);
        }
    }

    av_log(ctx, AV_LOG_VERBOSE, "%s-based decoder with SLICE_PARAMS=%u, "
           "ENTRY_POINT_OFFSETS=%u and SCALING_MATRIX=%d controls\n",
          ctx->decode_mode == V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED ? "slice" : "frame",
          ctx->max_slice_params, ctx->max_entry_point_offsets, ctx->has_scaling_matrix);

    control[0].value = ctx->decode_mode;
    control[1].value = ctx->start_code;

    return ff_v4l2_request_set_controls(avctx, control, FF_ARRAY_ELEMS(control));
}

static int v4l2_request_hevc_init(AVCodecContext *avctx)
{
    V4L2RequestContextHEVC *ctx = avctx->internal->hwaccel_priv_data;
    const HEVCContext *h = avctx->priv_data;
    struct v4l2_ctrl_hevc_sps sps;

    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_STATELESS_HEVC_SPS,
            .ptr = &sps,
            .size = sizeof(sps),
        },
    };

    fill_sps(&sps, h);

    ctx->base.post_probe = v4l2_request_hevc_post_probe;
    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_HEVC_SLICE,
                                4 * 1024 * 1024,
                                control, FF_ARRAY_ELEMS(control));
}

const AVHWAccel ff_hevc_v4l2request_hwaccel = {
    .name           = "hevc_v4l2request",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_HEVC,
    .pix_fmt        = AV_PIX_FMT_DRM_PRIME,
    .start_frame    = v4l2_request_hevc_start_frame,
    .decode_slice   = v4l2_request_hevc_decode_slice,
    .end_frame      = v4l2_request_hevc_end_frame,
    .frame_priv_data_size = sizeof(V4L2RequestControlsHEVC),
    .init           = v4l2_request_hevc_init,
    .uninit         = ff_v4l2_request_uninit,
    .priv_data_size = sizeof(V4L2RequestContextHEVC),
    .frame_params   = ff_v4l2_request_frame_params,
};
