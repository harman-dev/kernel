/*
 * Copyright (C) 2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
** =============================================================================
**
**				MODULE DESCRIPTION
**
** =============================================================================
*/
/* This file contains state machine of the FM Radio core driver */

/*
** =============================================================================
**
**				INCLUDE STATEMENTS
**
** =============================================================================
*/
#include "fmtrx_sys.h"		/* System related */
#include "fmtrx_common.h"
#include "fmtrx_hw_rx.h"

/*
** =============================================================================
**
**				DEFINES
**
** =============================================================================
*/
#define MAX_VOLUME 112
#define OFFSETS_ARRAY_SIZE (sizeof(u16) * MAX_OFFSETS)

/*
** =============================================================================
**
**				STRUCT DECLARATIONS
**
** =============================================================================
*/
#ifdef FMR_HOST_TEST
static enum fmrx_state host_state = FMRX_HW_STATE_INVALID;
static u32 host_frequency;
#endif

/*
** =============================================================================
**
**				LOCAL DATA DEFINITIONS
**
** =============================================================================
*/

/*
** =============================================================================
**
**				LOCAL FUNCTION DECLARATIONS
**
** =============================================================================
*/
/* Set offsets based on the antenna type
 * @antenna Type of antenna
 */
int fmrx_set_offsets(
		struct fmrx_priv *data,
		enum antenna_type antenna);

/*
** =============================================================================
**
**				EXPORTED FUNCTION DEFINITIONS
**
** =============================================================================
*/

int fmtrx_init(
		struct fmrx_priv **priv_data,
		enum fmtrx_init_mode mode,
		enum fmtrx_type type)
{
	int err = 0;
	struct fmrx_priv *data;

	if (FMTRX_INIT_MODE_ON == mode) {
		*priv_data = kzalloc(sizeof(struct fmrx_priv), GFP_KERNEL);
		if (0 == *priv_data) {
			err = -ENOMEM;
			fmtrx_sys_log(CRIT,
				"%s: %s %d, fmrx_priv Mem Alloc failed %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_init_exit;
		}

		data = *priv_data;

		err = fmtrx_sys_init(data);
		if (0 != err) {
			err = -EIO;
			fmtrx_sys_log(CRIT,
				"%s:%s %d, Sys init failed! %d\n", FILE,
				__func__, __LINE__, err);
			goto fmtrx_init_exit1;
		}
		goto fmtrx_init_exit;
	} else {
		data = *priv_data;
		err = fmtrx_sys_deinit(data->fmrx_plat);
		if (0 != err) {
			err = -EIO;
			fmtrx_sys_log(CRIT,
				"%s:%s %d, Sys deinit failed! %d\n", FILE,
				__func__, __LINE__, err);
		}
		kfree(data->fmrx_cfg);
		data->fmrx_cfg = NULL;
	}

fmtrx_init_exit1:
	kfree(data);
	data = NULL;
fmtrx_init_exit:
	return err;
}

int fmrx_get_hw_state(
		struct fmrx_priv *priv_data,
		enum fmrx_state *data)
{
	int err = 0;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log(CRIT,
			"%s: %s, Invalid arguments!\n",
			FILE,
			__func__);
		goto fmrx_get_hw_state_exit;
	}

#ifdef FMR_HOST_TEST
	*data = host_state;
#else
	err = fmrx_hw_get_fw_state(priv_data->fmrx_plat, data);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
			"%s:%s %d, Get FW state failed! %d\n",
			FILE, __func__,
			__LINE__, err);
	}
#endif

fmrx_get_hw_state_exit:
	return err;
}

int fmrx_power_on(
		struct fmrx_priv *priv_data)
{
	int err = 0;
	u32 size = 0;
	const u8 *data = 0;

	err = fmtrx_sys_power_enable(priv_data->fmrx_plat, true);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
			"%s: %s %d,Power enable failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit;
	}

	/* Halt miniDSP */
	err = fmtrx_hw_stop_minidsp(priv_data->fmrx_plat);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
			"%s: %s %d,Stop DSP failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	/* Fetch firmware image */
	err = fmtrx_sys_fetch_fw(priv_data->fmrx_plat, FMTRX_RX, &data, &size);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
			"%s: %s %d,Firmware fetch failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	/* Copy the FW to FMR IP memory */
	err = fmtrx_sys_mem_write(priv_data->fmrx_plat, 0, data, size);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
			"%s: %s %d,FW copy failed! failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit2;
	}

	/* Release fetched firmware */
	err = fmtrx_sys_release_fw(priv_data->fmrx_plat);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
			"%s: %s %d,Firmware release failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	/* Wait to fw get uncompressed */
	fmtrx_sys_idle_wait(2);

	err = fmtrx_hw_start_minidsp(priv_data->fmrx_plat);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
			"%s: %s %d,MiniDSP start failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmtrx_hw_rf_poweron(priv_data->fmrx_plat,
				priv_data->fmrx_cfg->antenna,
				priv_data->fmrx_cfg->other_cfg.lna_type);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
			"%s: %s %d, RF poweron failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmrx_set_offsets(priv_data, priv_data->fmrx_cfg->antenna);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d, Set offsets failed! %d\n",
				FILE, __func__,
				__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmtrx_hw_set_band(priv_data->fmrx_plat,
			priv_data->fmrx_cfg->band_cfg.min,
			priv_data->fmrx_cfg->band_cfg.max);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set band config failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmrx_hw_set_routing(priv_data->fmrx_plat,
			priv_data->fmrx_cfg->routing);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set Routing failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmrx_hw_set_volume(priv_data->fmrx_plat,
			priv_data->fmrx_cfg->vol_cfg.left,
			priv_data->fmrx_cfg->vol_cfg.right);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set Volume config failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmrx_hw_set_audio_deemp(priv_data->fmrx_plat,
			priv_data->fmrx_cfg->band_cfg.deemp);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set Deemphasis failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmrx_hw_set_audio_volumeramp(priv_data->fmrx_plat,
				priv_data->fmrx_cfg->other_cfg.volume_ramp);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set Volume ramp failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmrx_hw_set_audio_forcemono(priv_data->fmrx_plat,
			priv_data->fmrx_cfg->force_mono);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set Force mono failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmrx_hw_set_snc(priv_data->fmrx_plat,
			&priv_data->fmrx_cfg->snc_cfg);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set SNC config failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmrx_hw_set_sm(priv_data->fmrx_plat,
		&priv_data->fmrx_cfg->sm_cfg);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set SM configuration failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmrx_hw_set_agc(priv_data->fmrx_plat,
		&priv_data->fmrx_cfg->agc_cfg);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set AGC config failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmrx_hw_set_rds_cfg(priv_data->fmrx_plat, 4, 7, 4);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set RDS config failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmrx_hw_set_rds_onmode(priv_data->fmrx_plat,
		priv_data->fmrx_cfg->rds_cfg.mode);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set RDS power mode failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

#ifdef FMR_104MHZ_WORKAROUND
	err = fmtrx_sys_reg_write16(priv_data->fmrx_plat, 0x4D84, 0xE000);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Write 16-bit register failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}

	err = fmtrx_sys_reg_write16(priv_data->fmrx_plat, 0x4D82, 0xE000);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Write 16-bit register failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_on_exit1;
	}
#endif

#ifdef FMR_AUDIO_ENABLE
	if (!priv_data->fmrx_cfg->mute) {
		err = fmtrx_sys_audio_enable(priv_data->fmrx_plat, true);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Set Host audio failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_power_on_exit1;
		}
	}
#endif

#ifdef FMR_HOST_TEST
	host_state = FMRX_HW_STATE_IDLE;
#endif

	goto fmrx_power_on_exit;

fmrx_power_on_exit2: {
		int err1 = fmtrx_sys_release_fw(priv_data->fmrx_plat);
		if (0 != err1) {
			fmtrx_sys_log
				(CRIT,
				"%s: %s %d,FM RX fw release failed! %d\n",
				FILE, __func__,
				__LINE__, err1);
		}
	}
fmrx_power_on_exit1: {
		int err1 = fmtrx_sys_power_enable(priv_data->fmrx_plat, false);
		if (0 != err1) {
			fmtrx_sys_log
				(CRIT,
				"%s: %s %d,FM RX power disable failed! %d\n",
				FILE, __func__,
				__LINE__, err1);
		}
	}
fmrx_power_on_exit:
	fmtrx_sys_log_traffic();
	return err;
}

int fmrx_power_off(
		struct fmrx_priv *priv_data)
{
	int err = 0;

#ifdef FMR_AUDIO_ENABLE
	if (!priv_data->fmrx_cfg->mute) {
		err = fmtrx_sys_audio_enable(priv_data->fmrx_plat, false);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Set Host audio failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_power_off_exit;
		}
	}
#endif

	err = fmtrx_sys_power_enable(priv_data->fmrx_plat, false);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,FM RX power disable failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_power_off_exit;
	}

#ifdef FMR_HOST_TEST
	host_state = FMRX_HW_STATE_INVALID;
#endif

fmrx_power_off_exit:
	fmtrx_sys_log_traffic();
	return err;
}

int fmrx_set_band(
		struct fmrx_priv *priv_data,
		struct band *data)
{
	int err = 0;

	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmrx_set_band_exit;
	}

	if (!((priv_data->fmrx_cfg->band_cfg.max == data->max) &&
		(priv_data->fmrx_cfg->band_cfg.min == data->min))) {
		err = fmtrx_hw_set_band(priv_data->fmrx_plat,
				data->min, data->max);
		if (0 != err) {
			fmtrx_sys_log(CRIT,
				"%s: %s %d,Set band config failed! %d\n",
				FILE, __func__, __LINE__, err);
			goto fmrx_set_band_exit;
		}
		priv_data->fmrx_cfg->band_cfg.max = data->max;
		priv_data->fmrx_cfg->band_cfg.min = data->min;
	}
	priv_data->fmrx_cfg->band_cfg.step = data->step;

	err = fmrx_hw_set_audio_deemp(priv_data->fmrx_plat, data->deemp);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
			"%s: %s %d,Set deemphasis failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_band_exit;
	}
	priv_data->fmrx_cfg->band_cfg.deemp = data->deemp;

fmrx_set_band_exit:
	return err;
}

int fmrx_get_band(
		struct fmrx_priv *priv_data,
		struct band *data)
{
	int err = 0;

	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmrx_get_band_exit;
	}
	memcpy((u8 *)data, &priv_data->fmrx_cfg->band_cfg, sizeof(struct band));

fmrx_get_band_exit:
	return err;
}

int fmrx_get_channel_info(
		struct fmrx_priv *priv_data,
		struct channel_info *data)
{
	int err = 0;

	err = fmrx_hw_get_channel_info(priv_data->fmrx_plat, data);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
			"%s: %s %d,Get Channel information failed! %d\n",
			FILE, __func__, __LINE__, err);
		goto fmrx_get_channel_info_exit;
	}

#ifdef FMR_HOST_TEST
	data->state = host_state;
	data->frequency = host_frequency;
#endif

fmrx_get_channel_info_exit:
	return err;
}

int fmrx_get_channel_freq(
		struct fmrx_priv *priv_data,
		u32 *data)
{
	int err = 0;

	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmrx_get_channel_freq_exit;
	}

	err = fmrx_hw_get_channel_freq(priv_data->fmrx_plat, data);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Get channel frequency failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_get_channel_freq_exit;
	}

#ifdef FMR_HOST_TEST
	*data = host_frequency;
#endif

fmrx_get_channel_freq_exit:
	return err;
}

int fmrx_set_force_mono(
		struct fmrx_priv *priv_data,
		bool force_mono)
{
	int err = 0;

	if (priv_data->fmrx_cfg->force_mono != force_mono) {
		err = fmrx_hw_set_audio_forcemono(priv_data->fmrx_plat,
				force_mono);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Set force mono failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_set_force_mono_exit;
		}
		priv_data->fmrx_cfg->force_mono = force_mono;
	}

fmrx_set_force_mono_exit:
	return err;
}

s16 fmrx_calculate_rssi_other_offset(
	struct fmrx_config *fmrx_cfg,
	u32 frequency)
{
	s16 offset = 0;
	struct rssi_offsets *data = 0;
	bool is_external = ((ANTENNA_HS_SINGLEEND == fmrx_cfg->antenna) ||
			(ANTENNA_EBD_SINGLEEND == fmrx_cfg->antenna));

	data = is_external ? &fmrx_cfg->ext_ext_lna_offsets :
				&fmrx_cfg->int_ext_lna_offsets;

	if (frequency <= data->frequency1)
		offset = data->offset1;
	else if (frequency <= data->frequency2)
		offset = data->offset2;
	else if (frequency <= data->frequency3)
		offset = data->offset3;
	else if (frequency <= data->frequency4)
		offset = data->offset4;
	else if (frequency <= data->frequency5)
		offset = data->offset5;
	else
		offset = data->offset6;

	/* Add the RSSI other offset */
	offset += (is_external ? fmrx_cfg->other_cfg.ext_rssi_other_offset :
				fmrx_cfg->other_cfg.int_rssi_other_offset);

	return offset;
}

int fmrx_station_tuning(
		struct fmrx_priv *priv_data,
		u32 frequency)
{
	int err = 0;

	/* Validate if the frequency is within the band
	     configuration */
	if (frequency < priv_data->fmrx_cfg->band_cfg.min ||
		frequency > priv_data->fmrx_cfg->band_cfg.max) {
		err = -ECHRNG;
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Out of band frequency! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_station_tuning_exit;
	}

	/* Update RSSI Other Offset (adds Ext LNA Offset as well) */
	err = fmrx_hw_set_rssi_other_offset(
			priv_data->fmrx_plat,
			fmrx_calculate_rssi_other_offset(priv_data->fmrx_cfg,
						frequency));
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set RSSI Other offset failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_station_tuning_exit;
	}

	err = fmrx_hw_channel_tune(priv_data->fmrx_plat, frequency,
			priv_data->fmrx_cfg->side, 0,
			priv_data->fmrx_cfg->other_cfg.clk_switch_range_104);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Channel tune failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_station_tuning_exit;
	}

	err = fmrx_hw_rds_reset(priv_data->fmrx_plat);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,RDS reset failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_station_tuning_exit;
	}

#ifdef FMR_HOST_TEST
	host_frequency = frequency;
	host_state = FMRX_HW_STATE_RX_ACTIVE;
#endif

fmrx_station_tuning_exit:
	fmtrx_sys_log_traffic();
	return err;
}

int fmrx_set_volume_level(
		struct fmrx_priv *priv_data,
		enum audio_channel type,
		u8 level)
{
	int err = 0;
	u8 left = 0, right = 0;

	if ((MAX_VOLUME < level) || (FMRX_AUD_CHN_INVALID <= type)) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmrx_set_volume_level_exit;
	}

	left = ((FMRX_AUD_CHN_ALL == type) ||
			(FMRX_AUD_CHN_LEFT == type)) ?
			level : priv_data->fmrx_cfg->vol_cfg.left;
	right = ((FMRX_AUD_CHN_ALL == type) ||
			(FMRX_AUD_CHN_RIGHT == type)) ?
			level : priv_data->fmrx_cfg->vol_cfg.right;

	err = fmrx_hw_set_volume(priv_data->fmrx_plat, left, right);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
			"%s: %s %d,Set Volume configuration failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_volume_level_exit;
	}
	priv_data->fmrx_cfg->vol_cfg.left = left;
	priv_data->fmrx_cfg->vol_cfg.right = right;

fmrx_set_volume_level_exit:
	return err;
}

int fmrx_set_mute(
		struct fmrx_priv *priv_data,
		enum audio_channel type,
		bool enable)
{
	int err = 0;
	u8 left = 0, right = 0;

	if (FMRX_AUD_CHN_ALL == type) {
		if (priv_data->fmrx_cfg->mute != enable) {
#ifdef FMR_AUDIO_ENABLE
			err = fmtrx_sys_audio_enable(priv_data->fmrx_plat,
					!enable);
			if (0 != err) {
				fmtrx_sys_log(CRIT,
					"%s: %s %d,Set Host audio fail! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmrx_set_mute_exit;
			}
#endif
			err = fmrx_hw_set_mute(priv_data->fmrx_plat,
					enable, false);
			if (0 != err) {
				fmtrx_sys_log
				(CRIT, "%s: %s %d,Set mute config failed! %d\n",
				FILE, __func__,
				__LINE__, err);
				goto fmrx_set_mute_exit;
			}
			priv_data->fmrx_cfg->mute = enable;
		}
	} else {
		left = ((FMRX_AUD_CHN_LEFT == type) &&
			(enable)) ? 0 : priv_data->fmrx_cfg->vol_cfg.left;
		right = ((FMRX_AUD_CHN_RIGHT == type) &&
			(enable)) ? 0 : priv_data->fmrx_cfg->vol_cfg.right;

		err = fmrx_hw_set_volume(priv_data->fmrx_plat, left, right);
		if (0 != err) {
			fmtrx_sys_log(CRIT,
				"%s: %s %d,Set Volume config failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_set_mute_exit;
		}
	}

fmrx_set_mute_exit:
	return err;
}

int fmrx_station_seeking(
		struct fmrx_priv *priv_data,
		const struct v4l2_hw_freq_seek *data)
{
	int err = 0, step = 0;
	u32 start_frequency = 0, stop_frequency = 0;
	u32 tuned_frequency = 0, found_frequency = 0, cur_frequency = 0;
	enum seek_mode mode = FMRX_SEEK_MODE_INVALID;

	/* Check seek up/down */
	if (data->seek_upward) {
		mode = (data->wrap_around) ? FMRX_SEEK_MODE_UP :
				FMRX_SEEK_MODE_UP_TO_LIMIT;
	} else {
		mode = (data->wrap_around) ? FMRX_SEEK_MODE_DOWN :
				FMRX_SEEK_MODE_DOWN_TO_LIMIT;
	}

	/* Validate seek mode */
	if (FMRX_SEEK_MODE_INVALID <= mode) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmrx_station_seeking_exit;
	}

	/* Store the original frequency */
	err = fmrx_get_channel_freq(priv_data, &tuned_frequency);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Get Channel information failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_station_seeking_exit;
	}

#ifdef FMR_HOST_TEST
	host_state = FMRX_HW_STATE_RX_SEEKING;
#endif

	/* Auto seek mode is not supported */
	if (FMRX_SEEK_MODE_AUTO == mode) {
		/* TODO: */
		goto fmrx_station_seeking_exit;
	}

	step = ((FMRX_SEEK_MODE_UP == mode) ||
			(FMRX_SEEK_MODE_UP_TO_LIMIT == mode)) ?
		priv_data->fmrx_cfg->band_cfg.step :
		-priv_data->fmrx_cfg->band_cfg.step;
	cur_frequency = ((0 == tuned_frequency) ?
			priv_data->fmrx_cfg->band_cfg.min : tuned_frequency);
	start_frequency = cur_frequency + step;

	if ((FMRX_SEEK_MODE_UP == mode) ||
			(FMRX_SEEK_MODE_DOWN == mode)) {
		/* Correction for crossing band limits */
		start_frequency = (start_frequency >
				priv_data->fmrx_cfg->band_cfg.max) ?
			priv_data->fmrx_cfg->band_cfg.min : start_frequency;
		start_frequency = (start_frequency <
				priv_data->fmrx_cfg->band_cfg.min) ?
			priv_data->fmrx_cfg->band_cfg.max : start_frequency;

		stop_frequency = cur_frequency - step;
		/* Correction for crossing band limits */
		stop_frequency = (stop_frequency <
				priv_data->fmrx_cfg->band_cfg.min) ?
			priv_data->fmrx_cfg->band_cfg.max :
			stop_frequency;
		stop_frequency = (stop_frequency >
				priv_data->fmrx_cfg->band_cfg.max) ?
			priv_data->fmrx_cfg->band_cfg.min :
			stop_frequency;
	} else if ((FMRX_SEEK_MODE_UP_TO_LIMIT == mode) &&
			(cur_frequency != priv_data->fmrx_cfg->band_cfg.max)) {
		stop_frequency = priv_data->fmrx_cfg->band_cfg.max;
	} else if ((FMRX_SEEK_MODE_DOWN_TO_LIMIT == mode) &&
			(cur_frequency != priv_data->fmrx_cfg->band_cfg.min)) {
		stop_frequency = priv_data->fmrx_cfg->band_cfg.min;
	} else {
		err = -EINVAL;
		goto fmrx_station_seeking_exit;
	}

	err = fmrx_hw_channel_search(priv_data->fmrx_plat,
			start_frequency, stop_frequency,
			step, priv_data->fmrx_cfg->side,
			priv_data->fmrx_cfg->other_cfg.seek_thr,
			priv_data->fmrx_cfg->other_cfg.pn_thr);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
			"%s: %s %d,Channel search configuration failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_station_seeking_exit1;
	}

	/* Fetch the found channel */
	err = fmrx_get_channel_freq(priv_data, &found_frequency);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Get Channel information failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_station_seeking_exit1;
	}
	tuned_frequency = found_frequency;

fmrx_station_seeking_exit1:
	/* Don't tune to the original channel if EALREADY is returned as
      error - it happens only when SEEK is cancelled in
			the middle, by a new tune request */
	/* Don't tune to the original channel, if original frequency is
		 already zero - it happens when SEEK is called
		 before TUNE (first launch) */
	if ((-EALREADY != err) && (0 != tuned_frequency)) {
		int err1 = 0;
		/* Tune to the original/found channel */
		err1 = fmrx_station_tuning(priv_data, tuned_frequency);
		if (0 != err1) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Station tune failed! %d\n",
				FILE, __func__,
				__LINE__, err1);
		}
	}
fmrx_station_seeking_exit:
#ifdef FMR_HOST_TEST
	host_state = FMRX_HW_STATE_RX_ACTIVE;
#endif
	fmtrx_sys_log_traffic();
	return err;
}

int fmrx_set_route(
		struct fmrx_priv *priv_data,
		enum routing_mode routing)
{
	int err = 0;

	/* Validate routing mode */
	if (ROUTING_INVALID <= routing) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmrx_set_route_exit;
	}

	/* Skip, if same routing is requested */
	if (priv_data->fmrx_cfg->routing == routing) {
		fmtrx_sys_log
			(INFO, "%s: %s %d,Same routing requested!\n",
			FILE, __func__,
			__LINE__);
		goto fmrx_set_route_exit;
	}

	err = fmrx_hw_set_mute(priv_data->fmrx_plat, true, false);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set mute failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_route_exit;
	}

	err = fmrx_hw_set_routing(priv_data->fmrx_plat, routing);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set routing failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_route_exit;
	}

	err = fmrx_hw_set_audio_deemp(priv_data->fmrx_plat,
			priv_data->fmrx_cfg->band_cfg.deemp);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set Deemphasis failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_route_exit1;
	}
	priv_data->fmrx_cfg->routing = routing;

	err = fmrx_hw_set_mute(priv_data->fmrx_plat,
			priv_data->fmrx_cfg->mute, false);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set mute failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_route_exit;
	}

fmrx_set_route_exit1: {
		int err1 = 0;
		err1 = fmrx_hw_set_routing(priv_data->fmrx_plat,
				priv_data->fmrx_cfg->routing);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Set routing failed! %d\n",
				FILE, __func__,
				__LINE__, err1);
		}
	}
fmrx_set_route_exit:
	return err;
}

int fmrx_set_antenna(
		struct fmrx_priv *priv_data,
		enum antenna_type antenna)
{
	int err = 0;
	u32 tuned_frequency = 0;
	enum fmrx_state state = FMRX_HW_STATE_INVALID;

	/* Validate antenna type */
	if (ANTENNA_INVALID <= antenna) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmrx_set_antenna_exit;
	}

	/* Skip, if same antenna is requested */
	if (priv_data->fmrx_cfg->antenna == antenna) {
		fmtrx_sys_log
			(INFO, "%s: %s %d,Same antenna requested!\n",
			FILE, __func__,
			__LINE__);
		goto fmrx_set_antenna_exit;
	}

	/* Get state & validate it */
	err = fmrx_get_hw_state(priv_data, &state);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s:%s %d, Get FW state failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_antenna_exit;
	}

	if (FMRX_HW_STATE_RX_ACTIVE == state) {
		err = fmrx_get_channel_freq(priv_data, &tuned_frequency);
		if (0 != err) {
			fmtrx_sys_log(CRIT,
				"%s: %s %d,Get Channel inform failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_set_antenna_exit;
		}

		err = fmrx_hw_set_mute(priv_data->fmrx_plat, true, false);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Set mute failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_set_antenna_exit;
		}
	}

	/* Switch to requested antenna */
	err = fmtrx_hw_rf_poweron(priv_data->fmrx_plat, antenna,
			priv_data->fmrx_cfg->other_cfg.lna_type);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,RF poweron failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_antenna_exit;
	}
	priv_data->fmrx_cfg->antenna = antenna;

	/* Configure offsets for requested antenna */
	err = fmrx_set_offsets(priv_data, antenna);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set offsets failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_antenna_exit;
	}

	if (FMRX_HW_STATE_RX_ACTIVE == state) {
		err = fmrx_hw_channel_tune(priv_data->fmrx_plat,
			tuned_frequency,
			priv_data->fmrx_cfg->side, 0,
			priv_data->fmrx_cfg->other_cfg.clk_switch_range_104);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Channel tune failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_set_antenna_exit;
		}

		err = fmrx_hw_set_mute(priv_data->fmrx_plat,
				priv_data->fmrx_cfg->mute, false);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Set mute failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_set_antenna_exit;
		}
	}

fmrx_set_antenna_exit:
	return err;
}

int fmrx_set_sideband(
		struct fmrx_priv *priv_data,
		enum injection_side side,
		bool force)
{
	int err = 0;
	u32 tuned_frequency = 0;
	enum fmrx_state state = FMRX_HW_STATE_INVALID;

	/* Skip, if same injection side is requested */
	if (priv_data->fmrx_cfg->side == side) {
		fmtrx_sys_log
			(INFO, "%s: %s %d,Same Injection side requested!\n",
			FILE, __func__,
			__LINE__);
		goto fmrx_set_sideband_exit;
	}

	/* Get state & validate it */
	err = fmrx_get_hw_state(priv_data, &state);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s:%s %d, Get FW state failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_sideband_exit;
	}

	/* Tune to selected side only when it's in active state */
	if (FMRX_HW_STATE_RX_ACTIVE == state) {
		err = fmrx_get_channel_freq(priv_data, &tuned_frequency);
		if (0 != err) {
			fmtrx_sys_log(CRIT,
				"%s: %s %d,Get Channel info failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_set_sideband_exit;
		}

		err = fmrx_hw_channel_tune(priv_data->fmrx_plat,
			tuned_frequency, side, 0,
			priv_data->fmrx_cfg->other_cfg.clk_switch_range_104);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Channel tune failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_set_sideband_exit;
		}
	}
	if (force)
		priv_data->fmrx_cfg->side = side;

fmrx_set_sideband_exit:
	return err;
}

int fmrx_set_snc(
		struct fmrx_priv *priv_data,
		struct snc *data)
{
	int err = 0;

	err = fmrx_hw_set_snc(priv_data->fmrx_plat, data);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set SNC failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_snc_exit;
	}
	memcpy((u8 *)&priv_data->fmrx_cfg->snc_cfg,
			(u8 *)data, sizeof(struct snc));

fmrx_set_snc_exit:
	return err;
}

int fmrx_set_sm(
		struct fmrx_priv *priv_data,
		struct sm *data)
{
	int err = 0;

	err = fmrx_hw_set_sm(priv_data->fmrx_plat, data);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set SM failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_sm_exit;
	}

	memcpy((u8 *)&priv_data->fmrx_cfg->sm_cfg,
			(u8 *)data, sizeof(struct sm));

fmrx_set_sm_exit:
	return err;
}

int fmrx_set_agc(
		struct fmrx_priv *priv_data,
		struct agc *data)
{
	int err = 0;

	err = fmrx_hw_set_agc(priv_data->fmrx_plat, data);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set AGC failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_agc_exit;
	}
	memcpy((u8 *)&priv_data->fmrx_cfg->agc_cfg,
			(u8 *)data, sizeof(struct agc));

fmrx_set_agc_exit:
	return err;
}

int fmrx_set_rssi_notification(
		struct fmrx_priv *priv_data,
		struct rssi_notify *data)
{
	int err = 0;

	err = fmrx_hw_set_rssi_notification(priv_data->fmrx_plat, data);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set RSSI notification failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_rssi_notification_exit;
	}
	memcpy((u8 *)&priv_data->fmrx_cfg->rssi_cfg,
				(u8 *)data, sizeof(struct rssi_notify));

fmrx_set_rssi_notification_exit:
	return err;
}

int fmrx_set_other_params(
		struct fmrx_priv *priv_data,
		struct other_params *data)
{
	int err = 0;
	bool is_external =
		((ANTENNA_HS_SINGLEEND == priv_data->fmrx_cfg->antenna) ||
		 (ANTENNA_EBD_SINGLEEND == priv_data->fmrx_cfg->antenna));

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmrx_set_other_params_exit;
	}

	/* Validate input arguments */
	if (GAIN_INVALID <= data->lna_type) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmrx_set_other_params_exit;
	}

	/* Copy values that don't need any change in register settings */
	priv_data->fmrx_cfg->other_cfg.pn_thr = data->pn_thr;
	priv_data->fmrx_cfg->other_cfg.clk_switch_range_104 =
						data->clk_switch_range_104;
	priv_data->fmrx_cfg->other_cfg.seek_thr = data->seek_thr;

	/* Set RSSI other offset configuration */
	if ((priv_data->fmrx_cfg->other_cfg.ext_rssi_other_offset !=
				data->ext_rssi_other_offset) ||
			(priv_data->fmrx_cfg->other_cfg.int_rssi_other_offset !=
			data->int_rssi_other_offset)) {
		err = fmrx_hw_set_rssi_other_offset(priv_data->fmrx_plat,
				is_external ? data->ext_rssi_other_offset :
				data->int_rssi_other_offset);
		if (0 != err) {
			fmtrx_sys_log(CRIT,
				"%s: %s %d,Set RSSI Other offset failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_set_other_params_exit;
		}
		priv_data->fmrx_cfg->other_cfg.ext_rssi_other_offset =
					data->ext_rssi_other_offset;
		priv_data->fmrx_cfg->other_cfg.int_rssi_other_offset =
					data->int_rssi_other_offset;
	}

	/* Set volume ramp configuration */
	if (priv_data->fmrx_cfg->other_cfg.volume_ramp != data->volume_ramp) {
		err = fmrx_hw_set_audio_volumeramp(priv_data->fmrx_plat,
				data->volume_ramp);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Set Volume ramp failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_set_other_params_exit;
		}
		priv_data->fmrx_cfg->other_cfg.volume_ramp = data->volume_ramp;
	}

	/* Set LNA type configuration */
	if (priv_data->fmrx_cfg->other_cfg.lna_type != data->lna_type) {
		enum lna_out_gain temp_type =
					priv_data->fmrx_cfg->other_cfg.lna_type;

		priv_data->fmrx_cfg->other_cfg.lna_type = data->lna_type;
		/* Setting antenna function will update the lna type as well */
		err = fmrx_set_antenna(priv_data, priv_data->fmrx_cfg->antenna);
		if (0 != err) {
			priv_data->fmrx_cfg->other_cfg.lna_type = temp_type;
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Set Antenna failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_set_other_params_exit;
		}
	}

fmrx_set_other_params_exit:
	return err;
}

int fmtrx_set_gain_rssi_offsets(
		struct fmrx_priv *priv_data,
		struct rssi_offsets_pkt *data)
{
	int err = 0;
	struct rssi_offsets *cfg = 0;
	bool is_external = false;
	u32 tuned_frequency = 0;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log(CRIT,
			"%s: %s %d, Invalid arguments!\n", FILE, __func__,
			__LINE__);
		goto fmtrx_set_gain_rssi_offsets_exit;
	}

	/* Validate antenna type */
	if (ANTENNA_INVALID <= data->antenna) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s %d Invalid arguments!\n", FILE, __func__,
			__LINE__);
		goto fmtrx_set_gain_rssi_offsets_exit;
	}

	/* Validate frequency range */
	if (((0 != data->offsets.frequency1) &&
				(data->offsets.frequency1 <
				 priv_data->fmrx_cfg->band_cfg.min ||
				 data->offsets.frequency1 >
				 priv_data->fmrx_cfg->band_cfg.max)) ||
			((0 != data->offsets.frequency2) &&
			 (data->offsets.frequency2 <
			  priv_data->fmrx_cfg->band_cfg.min ||
			  data->offsets.frequency2 >
			  priv_data->fmrx_cfg->band_cfg.max)) ||
			((0 != data->offsets.frequency3) &&
			 (data->offsets.frequency3 <
			  priv_data->fmrx_cfg->band_cfg.min ||
			  data->offsets.frequency3 >
			  priv_data->fmrx_cfg->band_cfg.max)) ||
			((0 != data->offsets.frequency4) &&
			 (data->offsets.frequency4 <
			  priv_data->fmrx_cfg->band_cfg.min ||
			  data->offsets.frequency4 >
			  priv_data->fmrx_cfg->band_cfg.max)) ||
			((0 != data->offsets.frequency5) &&
			 (data->offsets.frequency5 <
			  priv_data->fmrx_cfg->band_cfg.min ||
			  data->offsets.frequency5 >
			  priv_data->fmrx_cfg->band_cfg.max))) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s %d Invalid arguments!\n", FILE, __func__,
			__LINE__);
		goto fmtrx_set_gain_rssi_offsets_exit;
	}

	/* Get current tuned frequency */
	err = fmrx_get_channel_freq(priv_data, &tuned_frequency);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Get Channel inform failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_set_gain_rssi_offsets_exit;
	}

	is_external = ((ANTENNA_HS_SINGLEEND == data->antenna) ||
			(ANTENNA_EBD_SINGLEEND == data->antenna));

	switch (data->type) {
	case GAIN_OFFSET_RSSI:
		if (priv_data->fmrx_cfg->antenna == data->antenna) {
			err = fmtrx_hw_set_gain_offsets(priv_data->fmrx_plat,
					GAIN_OFFSET_RSSI,
					(u8 *)&data->offsets,
					sizeof(struct rssi_offsets));
			if (0 != err) {
				fmtrx_sys_log(CRIT,
					"%s: %s %d,Set RSSI Gain off fail!%d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_set_gain_rssi_offsets_exit;
			}
		}
		cfg = is_external ? &priv_data->fmrx_cfg->ext_rssi_offsets :
					&priv_data->fmrx_cfg->int_rssi_offsets;
		memcpy((u8 *)cfg, (u8 *)&data->offsets,
					sizeof(struct rssi_offsets));
		break;
	case GAIN_OFFSET_EXT_LNA:
		cfg = is_external ? &priv_data->fmrx_cfg->ext_ext_lna_offsets :
				&priv_data->fmrx_cfg->int_ext_lna_offsets;
		memcpy((u8 *)cfg, (u8 *)&data->offsets,
					sizeof(struct rssi_offsets));
		if (priv_data->fmrx_cfg->antenna == data->antenna) {
			/* Update RSSI Other Offset
				(adds Ext LNA Offset as well) */
			err = fmrx_hw_set_rssi_other_offset(
				priv_data->fmrx_plat,
				fmrx_calculate_rssi_other_offset(
					priv_data->fmrx_cfg, tuned_frequency));
			if (0 != err) {
				fmtrx_sys_log(CRIT,
					"%s: %s %d,Set RSSI Other offset failed! %d\n",
					FILE, __func__,
					__LINE__, err);
				goto fmtrx_set_gain_rssi_offsets_exit;
			}
		}
		break;

	default:
		err = -EINVAL;
		break;
	}

	/* Tune back to current channel to update RSSI offsets */
	if (0 != tuned_frequency) {
		/* do the channel tune to get update of new rssi values. */
		err = fmrx_hw_channel_tune(priv_data->fmrx_plat,
			tuned_frequency, priv_data->fmrx_cfg->side, 0,
			priv_data->fmrx_cfg->other_cfg.clk_switch_range_104);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Channel tune failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_set_gain_rssi_offsets_exit;
		}
	}

fmtrx_set_gain_rssi_offsets_exit:
	return err;
}

int fmtrx_get_gain_rssi_offsets(
		struct fmrx_priv *priv_data,
		struct rssi_offsets_pkt *data)
{
	int err = 0;
	struct rssi_offsets *cfg = 0;
	bool is_external = false;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_get_gain_rssi_offsets_exit;
	}

	/* Validate antenna type */
	if (ANTENNA_INVALID <= data->antenna) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_get_gain_rssi_offsets_exit;
	}

	is_external = ((ANTENNA_HS_SINGLEEND == data->antenna) ||
			(ANTENNA_EBD_SINGLEEND == data->antenna));

	switch (data->type) {
	case GAIN_OFFSET_RSSI:
		cfg = is_external ? &priv_data->fmrx_cfg->ext_rssi_offsets :
					&priv_data->fmrx_cfg->int_rssi_offsets;
		memcpy((u8 *)&data->offsets, (u8 *)cfg,
					sizeof(struct rssi_offsets));
		break;
	case GAIN_OFFSET_EXT_LNA:
		cfg = is_external ? &priv_data->fmrx_cfg->ext_ext_lna_offsets :
			&priv_data->fmrx_cfg->int_ext_lna_offsets;
		memcpy((u8 *)&data->offsets, (u8 *)cfg,
					sizeof(struct rssi_offsets));
		break;
	default:
		err = -EINVAL;
		break;
	}

fmtrx_get_gain_rssi_offsets_exit:
	return err;
}

int fmtrx_set_gain_offsets(
		struct fmrx_priv *priv_data,
		struct gain_offsets_pkt *data)
{
	int err = 0;
	u16 *cfg = 0;
	bool is_external = false;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_set_gain_offsets_exit;
	}

	/* Validate antenna type */
	if (ANTENNA_INVALID <= data->antenna) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_set_gain_offsets_exit;
	}

	is_external = ((ANTENNA_HS_SINGLEEND == data->antenna) ||
			(ANTENNA_EBD_SINGLEEND == data->antenna));

	switch (data->type) {
	case GAIN_OFFSET_LNA:
		cfg = is_external ? priv_data->fmrx_cfg->ext_lna_offsets :
					priv_data->fmrx_cfg->int_lna_offsets;
		break;
	case GAIN_OFFSET_PPF:
		cfg = is_external ? priv_data->fmrx_cfg->ext_ppf_offsets :
					priv_data->fmrx_cfg->int_ppf_offsets;
		break;
	case GAIN_OFFSET_CP_INIT:
		cfg = priv_data->fmrx_cfg->cp_init_offsets;
		break;
	default:
		err = -EINVAL;
		goto fmtrx_set_gain_offsets_exit;
	}

	if (priv_data->fmrx_cfg->antenna == data->antenna) {
		err = fmtrx_hw_set_gain_offsets(priv_data->fmrx_plat,
				data->type,
				(u8 *)data->offsets, OFFSETS_ARRAY_SIZE);
		if (0 != err) {
			fmtrx_sys_log(CRIT,
				"%s: %s %d,Set Gain off (type %d) fail! %d\n",
				FILE, __func__,
				__LINE__, data->type, err);
			goto fmtrx_set_gain_offsets_exit;
		}
	}
	memcpy((u8 *)cfg, (u8 *)&data->offsets, OFFSETS_ARRAY_SIZE);

fmtrx_set_gain_offsets_exit:
	return err;
}

int fmtrx_get_gain_offsets(
		struct fmrx_priv *priv_data,
		struct gain_offsets_pkt *data)
{
	int err = 0;
	u16 *cfg = 0;
	bool is_external = false;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_get_gain_offsets_exit;
	}

	/* Validate antenna type */
	if (ANTENNA_INVALID <= data->antenna) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmtrx_get_gain_offsets_exit;
	}

	is_external = ((ANTENNA_HS_SINGLEEND == data->antenna) ||
				(ANTENNA_EBD_SINGLEEND == data->antenna));

	switch (data->type) {
	case GAIN_OFFSET_LNA:
		cfg = is_external ? priv_data->fmrx_cfg->ext_lna_offsets :
					priv_data->fmrx_cfg->int_lna_offsets;
		break;
	case GAIN_OFFSET_PPF:
		cfg = is_external ? priv_data->fmrx_cfg->ext_ppf_offsets :
					priv_data->fmrx_cfg->int_ppf_offsets;
		break;
	case GAIN_OFFSET_CP_INIT:
		cfg = priv_data->fmrx_cfg->cp_init_offsets;
		break;
	default:
		err = -EINVAL;
		goto fmtrx_get_gain_offsets_exit;
	}
	memcpy((u8 *)&data->offsets, (u8 *)cfg, OFFSETS_ARRAY_SIZE);

fmtrx_get_gain_offsets_exit:
	return err;
}

int fmrx_get_rds_groups(
		struct fmrx_priv *priv_data,
		struct rds_group_pkt *data)
{
	int err = 0;
	enum fmrx_state state = FMRX_HW_STATE_INVALID;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmrx_get_rds_groups_exit;
	}

	/* Get state & validate it */
	err = fmrx_get_hw_state(priv_data, &state);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s:%s %d, Get FW state failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_get_rds_groups_exit;
	}


	/* Check if RDS is subscribed */
	/* Allow RDS fetch in active state only */
	if ((RDS_ONMODE_ON == priv_data->fmrx_cfg->rds_cfg.mode) &&
			(FMRX_HW_STATE_RX_ACTIVE == state)) {
		err = fmrx_hw_get_rds_groups(priv_data->fmrx_plat,
			data->requested_groups,
			data->data, true, &data->received_groups);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Get RDS groups failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_get_rds_groups_exit;
		}
	} else {
		/* Return bad state since RDS is not subscribed */
		err = -EBADFD;
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Get RDS called on invalid state!%d\n",
			FILE, __func__,
			__LINE__, err);
	}

fmrx_get_rds_groups_exit:
	fmtrx_sys_log_traffic();
	return err;
}

int fmrx_set_rds(
		struct fmrx_priv *priv_data,
		struct rds *data)
{
	int err = 0;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmrx_set_rds_exit;
	}

	/* Skip, if same RDS configuration is requested */
	if (data->mode == priv_data->fmrx_cfg->rds_cfg.mode) {
		fmtrx_sys_log
			(INFO, "%s: %s %d,Same RDS configuration requested!\n",
			FILE, __func__,
			__LINE__);
		goto fmrx_set_rds_exit;
	}

	err = fmrx_hw_set_rds_cfg(priv_data->fmrx_plat, 4, 7, 4);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set RDS configuration failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_rds_exit;
	}

	err = fmrx_hw_set_rds_onmode(priv_data->fmrx_plat, data->mode);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set RDS power mode failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_rds_exit;
	}
	priv_data->fmrx_cfg->rds_cfg.mode = data->mode;

	err = fmrx_hw_rds_reset(priv_data->fmrx_plat);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,RDS reset failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_rds_exit;
	}

fmrx_set_rds_exit:
	return err;
}

int fmtrx_get_id(
		struct fmrx_priv *priv_data,
		struct fmr_id *data)
{
	int err = 0;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmrx_get_id_exit;
	}

	err = fmrx_hw_get_id(priv_data->fmrx_plat, data);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Get RX Core version failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_get_id_exit;
	}

fmrx_get_id_exit:
	return err;
}

int fmrx_af_switch(
		struct fmrx_priv *priv_data,
		struct af_info *data)
{
	int err = 0;
	s16 idx = 0, idx1 = 0, *rssi = 0, t_rssi = 0;
	u32 tuned_frequency = 0, t_frequency = 0;
	s16 pi_code = 0, tuned_rssi = 0;
	enum fmrx_state state = FMRX_HW_STATE_INVALID;

	/* Validate input arguments */
	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			(CRIT, "%s: %s, Invalid arguments!\n", FILE, __func__);
		goto fmrx_af_switch_exit;
	}

	/* Get state & validate it */
	err = fmrx_get_hw_state(priv_data, &state);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s:%s %d, Get FW state failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_af_switch_exit;
	}

	/* Allow AF switch only in active state */
	if (FMRX_HW_STATE_RX_ACTIVE != state) {
		err = -EBADFD;
		fmtrx_sys_log(CRIT,
			"%s: %s %d,FMR in wrong state! Current state: %d,%d\n",
			FILE, __func__,
			__LINE__, state, err);
		goto fmrx_af_switch_exit;
	}

	rssi = kzalloc(sizeof(s16) * data->count, GFP_KERNEL);
	if (0 == rssi) {
		err = ENOMEM;
		fmtrx_sys_log
			(CRIT, "%s: %s %d,RSSI list allocation failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_af_switch_exit;
	}

	/* Store the current tuned frequency */
	err = fmrx_hw_get_channel_freq(priv_data->fmrx_plat, &tuned_frequency);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Get Channel frequency failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_af_switch_exit;
	}

	/* Store the current RSSI */
	err = fmrx_hw_get_channel_rssi(priv_data->fmrx_plat, &tuned_rssi);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Get Channel RSSI failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_af_switch_exit;
	}

	fmtrx_sys_log
	(INFO, "%s: %s %d,Current Frequency: %d, RSSI: %d!\n",
	FILE, __func__,
	__LINE__, tuned_frequency, tuned_rssi);

	/* Mute before we do AF switch to avoid distortions */
	err = fmrx_hw_set_mute(priv_data->fmrx_plat, true, false);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set mute failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_af_switch_exit;
	}

	/* Loop around to find out the best AF to switch
			based on the signal strength */
	for (idx = 0; idx < data->count; idx++) {
		err = fmrx_station_tuning(priv_data,
					data->freq_list[idx]);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Channel tune failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_af_switch_exit1;
		}

		err = fmrx_hw_get_channel_rssi(priv_data->fmrx_plat,
				&rssi[idx]);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Get RSSI failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_af_switch_exit1;
		}
		fmtrx_sys_log
		(INFO, "%s: %s %d,AF frequency: %d, RSSI: %d!\n",
		FILE, __func__,
		__LINE__, data->freq_list[idx], rssi[idx]);
	}

	/* Insertion sort is better for smaller array of integers */
	for (idx = 1; idx < data->count; idx++) {
		t_rssi = rssi[idx];
		t_frequency = data->freq_list[idx];
		idx1 = idx;
		while ((idx1 > 0) && (rssi[idx1 - 1] < t_rssi)) {
			rssi[idx1] = rssi[idx1 - 1];
			data->freq_list[idx1] = data->freq_list[idx1 - 1];
			idx1 = idx1 - 1;
		}
		rssi[idx1] = t_rssi;
		data->freq_list[idx1] = t_frequency;
	}

	/* Switch on RDS */
	err = fmrx_hw_set_rds_onmode(priv_data->fmrx_plat, RDS_ONMODE_ON);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set RDS power mode failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_af_switch_exit1;
	}

	/* Set RDS to Fast PI mode */
	err = fmrx_hw_set_rds_pimode(priv_data->fmrx_plat, true);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set RDS fast pi mode failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_af_switch_exit2;
	}

	/* Find a suitable AF channel */
	for (idx = 0; idx < data->count; idx++) {
		fmtrx_sys_log
		(INFO, "%s: %s %d,Evaluating AF frequency: %d!\n",
		FILE, __func__,
		__LINE__, data->freq_list[idx]);
		pi_code = 0;

		/* Skip, if signal strength of the AF channel is
			lesser than the original frequency */
		if (rssi[idx] < tuned_rssi) {
			fmtrx_sys_log(INFO,
				"%s: %s %d,Skip AF chn.strength low than orig freq!\n",
				FILE, __func__,
				__LINE__);
			err = -EAGAIN;
			break;
		}

		/* Tune to the AF channel */
		err = fmrx_hw_channel_tune(priv_data->fmrx_plat,
			data->freq_list[idx], priv_data->fmrx_cfg->side, 0,
			priv_data->fmrx_cfg->other_cfg.clk_switch_range_104);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Channel tune failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_af_switch_exit3;
		}

		/* Wait & fetch the PI code */
		err = fmrx_hw_get_rds_pi(priv_data->fmrx_plat, &pi_code);
		if (0 != err) {
			fmtrx_sys_log(CRIT,
				"%s: %s %d,Fetch RDS PI code failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			continue;
		}

		fmtrx_sys_log
		(INFO, "%s: %s %d,Eval AF freq: %d,PI code detected: %d!\n",
		FILE, __func__,
		__LINE__, data->freq_list[idx], pi_code);

		/* Match with the original channel PI code */
		if (pi_code == data->pi_code) {
			fmtrx_sys_log
			(INFO, "%s: %s %d,PI code match Success; AF chn: %d!\n",
				FILE, __func__,
				__LINE__, data->freq_list[idx]);
			break;
		}
	}

fmrx_af_switch_exit3: {
		int err1 = 0;
		/* Set RDS to Normal PI mode */
		err1 = fmrx_hw_set_rds_pimode(priv_data->fmrx_plat, false);
		if (0 != err1) {
			fmtrx_sys_log(CRIT,
				"%s: %s %d,Set RDS fast pi mode failed! %d\n",
				FILE, __func__,
				__LINE__, err1);
		}
	}
fmrx_af_switch_exit2: {
		int err1 = 0;
		/* Restore RDS power state */
		err1 = fmrx_hw_set_rds_onmode(priv_data->fmrx_plat,
				priv_data->fmrx_cfg->rds_cfg.mode);
		if (0 != err1) {
			fmtrx_sys_log(CRIT,
				"%s: %s %d,Set RDS power mode failed! %d\n",
				FILE, __func__,
				__LINE__, err1);
		}
	}
fmrx_af_switch_exit1: {
		int err1 = 0;

		/* Tune to original frequency if there is a failure */
		if (0 != err) {
			/* Tune to the original channel */
			err1 = fmrx_station_tuning(priv_data,
						tuned_frequency);
			if (0 != err1) {
				fmtrx_sys_log(CRIT,
					"%s: %s %d,Channel tune failed! %d\n",
					FILE, __func__,
					__LINE__, err1);
			}
		}
		/* Restore to requested mute state */
		err1 = fmrx_hw_set_mute(priv_data->fmrx_plat,
				priv_data->fmrx_cfg->mute, false);
		if (0 != err1) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Set mute failed! %d\n",
				FILE, __func__,
				__LINE__, err1);
		}
	}
fmrx_af_switch_exit:
	kfree(rssi);
	return err;
}

int fmrx_stop(
		struct fmrx_priv *priv_data)
{
	int err = 0;
	enum fmrx_state state = FMRX_HW_STATE_INVALID;

	err = fmrx_get_hw_state(priv_data, &state);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s:%s %d, Get FW state failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_stop_exit;
	}

	if (FMRX_HW_STATE_RX_SEEKING == state) {
		/* Unblock the seek IOCTL by waking up the event */
		err = fmtrx_sys_wakeup_event(priv_data->fmrx_plat);
		if (0 != err) {
			fmtrx_sys_log
				(CRIT, "%s: %s %d,Wake up event failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_stop_exit;
		}

		/* Cancel seek by switching to idle state */
		err = fmrx_hw_set_fw_state(priv_data->fmrx_plat,
				FMRX_HW_STATE_IDLE);
		if (0 != err) {
			fmtrx_sys_log
			(CRIT, "%s: %s %d,Set firmware state failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_stop_exit;
		}
	}
fmrx_stop_exit:
	return err;
}

int fmrx_get_debug_info(
		struct fmrx_priv *priv_data,
		struct debug_info *data)
{
	int err = 0;
	err = fmrx_hw_get_debug_info(priv_data->fmrx_plat, data);
	if (0 != err) {
		fmtrx_sys_log(CRIT,
		"%s: %s %d,Get Debug information failed! %d\n",
		FILE, __func__, __LINE__, err);
	}

	return err;
}

/*
** =============================================================================
**
**				LOCAL FUNCTION DEFINITIONS
**
** =============================================================================
*/
int fmrx_set_offsets(
		struct fmrx_priv *priv_data,
		enum antenna_type antenna)
{
	int err = 0;
	struct rssi_offsets *rssi_offs = 0;
	u16 *lna_offsets = 0, *ppf_offsets = 0;
	bool is_external = ((ANTENNA_HS_SINGLEEND == antenna) ||
				(ANTENNA_EBD_SINGLEEND == antenna));

	lna_offsets = is_external ? priv_data->fmrx_cfg->ext_lna_offsets :
				priv_data->fmrx_cfg->int_lna_offsets;
	err = fmtrx_hw_set_gain_offsets(priv_data->fmrx_plat, GAIN_OFFSET_LNA,
				(u8 *)lna_offsets, OFFSETS_ARRAY_SIZE);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set LNA Gain offsets failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_offsets_exit;
	}

	ppf_offsets = is_external ? priv_data->fmrx_cfg->ext_ppf_offsets :
					priv_data->fmrx_cfg->int_ppf_offsets;
	err = fmtrx_hw_set_gain_offsets(priv_data->fmrx_plat, GAIN_OFFSET_PPF,
				(u8 *)ppf_offsets, OFFSETS_ARRAY_SIZE);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set PPF Gain offsets failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_offsets_exit;
	}

	rssi_offs = is_external ? &priv_data->fmrx_cfg->ext_rssi_offsets :
				&priv_data->fmrx_cfg->int_rssi_offsets;
	err = fmtrx_hw_set_gain_offsets(priv_data->fmrx_plat, GAIN_OFFSET_RSSI,
			(u8 *)rssi_offs, sizeof(struct rssi_offsets));
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set RSSI Gain offsets failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_offsets_exit;
	}

	if ((ANTENNA_EBD_SINGLEEND == antenna) ||
			(ANTENNA_EBD_DIFFERENTIAL == antenna)) {
		err = fmtrx_hw_set_gain_offsets(priv_data->fmrx_plat,
				GAIN_OFFSET_CP_INIT,
				(u8 *)priv_data->fmrx_cfg->cp_init_offsets,
				OFFSETS_ARRAY_SIZE);
		if (0 != err) {
			fmtrx_sys_log(CRIT,
				"%s: %s %d,Set CP init offsets failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmrx_set_offsets_exit;
		}
	}

	err = fmrx_hw_set_rssi_other_offset(priv_data->fmrx_plat, is_external ?
			priv_data->fmrx_cfg->other_cfg.ext_rssi_other_offset :
			priv_data->fmrx_cfg->other_cfg.int_rssi_other_offset);
	if (0 != err) {
		fmtrx_sys_log
			(CRIT, "%s: %s %d,Set RSSI Other offset failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmrx_set_offsets_exit;
	}

fmrx_set_offsets_exit:
	return err;
}

/* end of file */
