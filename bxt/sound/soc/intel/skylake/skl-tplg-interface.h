/*
 * skl-tplg-interface.h - Intel DSP FW private data interface
 *
 * Copyright (C) 2015 Intel Corp
 * Author: Jeeja KP <jeeja.kp@intel.com>
 *	    Nilofer, Samreen <samreen.nilofer@intel.com>
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __HDA_TPLG_INTERFACE_H__
#define __HDA_TPLG_INTERFACE_H__

/*
 * Default types range from 0~12. type can range from 0 to 0xff
 * SST types start at higher to avoid any overlapping in future
 */
#define SKL_CONTROL_TYPE_BYTE_TLV	0x100
#define SKL_CONTROL_TYPE_VOLUME         0x102
#define SKL_CONTROL_TYPE_RAMP_DURATION  0x103
#define SKL_CONTROL_TYPE_RAMP_TYPE      0x104

#define HDA_SST_CFG_MAX	900 /* size of copier cfg*/
#define MAX_IN_QUEUE 8
#define MAX_OUT_QUEUE 8
#define STEREO 2

#define SKL_UUID_STR_SZ 40
/* Event types goes here */
/* Reserve event type 0 for no event handlers */
enum skl_event_types {
	SKL_EVENT_NONE = 0,
	SKL_MIXER_EVENT,
	SKL_MUX_EVENT,
	SKL_VMIXER_EVENT,
	SKL_PGA_EVENT
};

/**
 * enum skl_ch_cfg - channel configuration
 *
 * @SKL_CH_CFG_MONO:	One channel only
 * @SKL_CH_CFG_STEREO:	L & R
 * @SKL_CH_CFG_2_1:	L, R & LFE
 * @SKL_CH_CFG_3_0:	L, C & R
 * @SKL_CH_CFG_3_1:	L, C, R & LFE
 * @SKL_CH_CFG_QUATRO:	L, R, Ls & Rs
 * @SKL_CH_CFG_4_0:	L, C, R & Cs
 * @SKL_CH_CFG_5_0:	L, C, R, Ls & Rs
 * @SKL_CH_CFG_5_1:	L, C, R, Ls, Rs & LFE
 * @SKL_CH_CFG_DUAL_MONO: One channel replicated in two
 * @SKL_CH_CFG_I2S_DUAL_STEREO_0: Stereo(L,R) in 4 slots, 1st stream:[ L, R, -, - ]
 * @SKL_CH_CFG_I2S_DUAL_STEREO_1: Stereo(L,R) in 4 slots, 2nd stream:[ -, -, L, R ]
 * @SKL_CH_CFG_INVALID:	Invalid
 */
enum skl_ch_cfg {
	SKL_CH_CFG_MONO = 0,
	SKL_CH_CFG_STEREO = 1,
	SKL_CH_CFG_2_1 = 2,
	SKL_CH_CFG_3_0 = 3,
	SKL_CH_CFG_3_1 = 4,
	SKL_CH_CFG_QUATRO = 5,
	SKL_CH_CFG_4_0 = 6,
	SKL_CH_CFG_5_0 = 7,
	SKL_CH_CFG_5_1 = 8,
	SKL_CH_CFG_DUAL_MONO = 9,
	SKL_CH_CFG_I2S_DUAL_STEREO_0 = 10,
	SKL_CH_CFG_I2S_DUAL_STEREO_1 = 11,
	SKL_CH_CFG_4_CHANNEL = 12,
	SKL_CH_CFG_INVALID
};

enum skl_module_type {
	SKL_MODULE_TYPE_MIXER = 0,
	SKL_MODULE_TYPE_COPIER,
	SKL_MODULE_TYPE_UPDWMIX,
	SKL_MODULE_TYPE_SRCINT,
	SKL_MODULE_TYPE_ALGO,
	SKL_MODULE_TYPE_BASE_OUTFMT,
	SKL_MODULE_TYPE_KPB,
	SKL_MODULE_TYPE_GAIN
};

enum skl_core_affinity {
	SKL_AFFINITY_CORE_0 = 0,
	SKL_AFFINITY_CORE_1,
	SKL_AFFINITY_CORE_MAX
};

enum skl_pipe_conn_type {
	SKL_PIPE_CONN_TYPE_NONE = 0,
	SKL_PIPE_CONN_TYPE_FE,
	SKL_PIPE_CONN_TYPE_BE
};

enum skl_hw_conn_type {
	SKL_CONN_NONE = 0,
	SKL_CONN_SOURCE = 1,
	SKL_CONN_SINK = 2
};

enum skl_dev_type {
	SKL_DEVICE_BT = 0x0,
	SKL_DEVICE_DMIC = 0x1,
	SKL_DEVICE_I2S = 0x2,
	SKL_DEVICE_SLIMBUS = 0x3,
	SKL_DEVICE_HDALINK = 0x4,
	SKL_DEVICE_HDAHOST = 0x5,
	SKL_DEVICE_NONE
};

/**
 * enum skl_interleaving - interleaving style
 *
 * @SKL_INTERLEAVING_PER_CHANNEL: [s1_ch1...s1_chN,...,sM_ch1...sM_chN]
 * @SKL_INTERLEAVING_PER_SAMPLE: [s1_ch1...sM_ch1,...,s1_chN...sM_chN]
 */
enum skl_interleaving {
	SKL_INTERLEAVING_PER_CHANNEL = 0,
	SKL_INTERLEAVING_PER_SAMPLE = 1,
};

enum skl_sample_type {
	SKL_SAMPLE_TYPE_INT_MSB = 0,
	SKL_SAMPLE_TYPE_INT_LSB = 1,
	SKL_SAMPLE_TYPE_INT_SIGNED = 2,
	SKL_SAMPLE_TYPE_INT_UNSIGNED = 3,
	SKL_SAMPLE_TYPE_FLOAT = 4
};

enum module_pin_type {
	/* All pins of the module takes same PCM inputs or outputs
	* e.g. mixout
	*/
	SKL_PIN_TYPE_HOMOGENEOUS,
	/* All pins of the module takes different PCM inputs or outputs
	* e.g mux
	*/
	SKL_PIN_TYPE_HETEROGENEOUS,
};

enum skl_module_param_type {
	SKL_PARAM_DEFAULT = 0,
	SKL_PARAM_INIT,
	SKL_PARAM_SET,
	SKL_PARAM_BIND
};

struct skl_dfw_module_pin {
	u16 module_id;
	u16 instance_id;
} __packed;

struct skl_dfw_module_fmt {
	u32 channels;
	u32 freq;
	u32 bit_depth;
	u32 valid_bit_depth;
	u32 ch_cfg;
	u32 interleaving_style;
	u32 sample_type;
	u32 ch_map;
} __packed;

struct skl_dfw_module_caps {
	u32 set_params:2;
	u32 rsvd:30;
	u32 param_id;
	u32 caps_size;
	u32 caps[HDA_SST_CFG_MAX];
};

struct skl_dfw_pipe {
	u8 pipe_id;
	u8 pipe_priority;
	u16 conn_type:4;
	u16 rsvd:4;
	u16 memory_pages:8;
} __packed;

struct skl_dfw_module {
	u8 uuid[16];

	u16 module_id;
	u16 instance_id;
	u32 max_mcps;
	u32 mem_pages;
	u32 obs;
	u32 ibs;
	u32 vbus_id;
	u32 dma_buffer_size;

	u32 max_in_queue:8;
	u32 max_out_queue:8;
	u32 time_slot:8;
	u32 core_id:4;
	u32 rsvd1:4;

	u32 module_type:8;
	u32 conn_type:4;
	u32 dev_type:4;
	u32 hw_conn_type:4;
	u32 rsvd2:12;

	u32 params_fixup:8;
	u32 converter:8;
	u32 input_pin_type:1;
	u32 output_pin_type:1;
	u32 is_dynamic_in_pin:1;
	u32 is_dynamic_out_pin:1;
	u32 is_loadable:1;
	u32 proc_domain:1;
	u32 rsvd3:10;

	struct skl_dfw_pipe pipe;
	struct skl_dfw_module_fmt in_fmt[MAX_IN_QUEUE];
	struct skl_dfw_module_fmt out_fmt[MAX_OUT_QUEUE];
	struct skl_dfw_module_pin in_pin[MAX_IN_QUEUE];
	struct skl_dfw_module_pin out_pin[MAX_OUT_QUEUE];
	struct skl_dfw_module_caps caps;
} __packed;

struct skl_gain_config {
	u32 channel_id;
	u32 target_volume;
	u32 ramp_type;
	u64 ramp_duration;
} __packed;

struct skl_dfw_algo_data {
	u32 set_params:2;
	u32 rsvd:30;
	u32 param_id;
	u32 max;
	char params[0];
} __packed;

struct skl_dfw_dma_ctrl_hdr {
	u32 vbus_id;
	u32 direction;
	u32 tdm_slot;
	struct skl_dfw_module_fmt fmt;
	u32 data_size;
	u8 *data[0];
} __packed;

struct skl_dfw_sclk_fs_cfg {
	u32 vbus_id;
	u32 direction;
	u32 tdm_slot;
	struct skl_dfw_module_fmt fmt;
	u32 data_size;

	/* DMA SClk&FS  TLV params */
	u32 type;
	u32 size;

	u32 sampling_frequency;
	u32 bit_depth;
	u32 channel_map;
	u32 channel_config;
	u32 interleaving_style;
	u32 number_of_channels : 8;
	u32 valid_bit_depth : 8;
	u32 sample_type : 8;
	u32 reserved : 8;
} __packed;

struct skl_dfw_clk_cfg {
	u32 vbus_id;
	u32 direction;
	u32 tdm_slot;
	struct skl_dfw_module_fmt fmt;
	u32 data_size;

	/* DMA Clk TLV params */
	u32 type;
	u32 len;

	u32 clk_warm_up:16;
	u32 mclk:1;
	u32 warm_up_over:1;
	u32 rsvd0:14;

	u32 clk_stop_delay:16;
	u32 keep_running:1;
	u32 clk_stop_over:1;
	u32 rsvd1:14;
} __packed;

#define LIB_NAME_LENGTH	128
#define HDA_MAX_LIB	16
#define MAX_SSP		6
#define MAX_DMA_CFG    24

struct lib_info {
	char name[LIB_NAME_LENGTH];
} __packed;

struct skl_dfw_dma_ctrl_info {
	u32 type;
	u32 size;
	struct skl_dfw_clk_cfg clk_cfg;
	struct skl_dfw_sclk_fs_cfg sclk_fs_cfg[MAX_SSP];
} __packed;

struct skl_dma_config {
	u32 min_size;
	u32 max_size;
} __packed;

struct skl_mem_status {
	u32 type;
	u32 size;
	u32 mem_status;
} __packed;

struct skl_dsp_freq {
	u32 type;
	u32 size;
	u32 frq;
} __packed;

struct skl_dma_buff_cfg {
	u32 type;
	u32 size;
	struct skl_dma_config dma_cfg[MAX_DMA_CFG];
} __packed;

struct fw_cfg_info {
	struct skl_mem_status mem_sts;
	struct skl_dsp_freq slw_frq;
	struct skl_dsp_freq fst_frq;
	struct skl_dma_buff_cfg dmacfg;
} __packed;

struct skl_dfw_manifest {
	u32 lib_count;
	struct lib_info lib[HDA_MAX_LIB];
	struct fw_cfg_info cfg;
	struct skl_dfw_dma_ctrl_info dma_ctrl;
} __packed;

#endif
