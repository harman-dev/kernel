/*
 * tdf8532.c  --  driver for NXP Semiconductors TDF8532
 *
 * Copyright (C) 2016 Intel Corp.
 * Author: Steffen Wagner <steffen.wagner@intel.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>

struct tdf8532_priv {
	struct i2c_client *i2c;
	u8 rom_version;
};

static u8 packet_id;

static int tdf8532_dai_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return 0;
}

static int tdf8532_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	return 0;
}

static int tdf8532_dai_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	int ret;
	struct snd_soc_codec *codec = dai->codec;
	struct tdf8532_priv *tdf8532 = snd_soc_codec_get_drvdata(codec);

	unsigned char data[6] = {0x02, packet_id++, 0x03, 0x80, 0x1A, 0x01};

	dev_dbg(codec->dev, "%s: cmd = %d\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		data[5] = 0x01;
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		data[5] = 0x00;
		break;
	}
	ret = i2c_master_send(tdf8532->i2c, data, 6);

	dev_dbg(codec->dev, "%s:i2c master send returned :%d\n", __func__, ret);

	return ret;
}

static int tdf8532_dai_prepare(struct snd_pcm_substream *substream,
						struct snd_soc_dai *dai)
{
	return 0;
}


#define MUTE 0x42
#define UNMUTE 0x43
#define VERSION_7 0x07
#define VERSION_6 0x06
#define ROM_VERSION_OFFSET 0x8

static int tdf8532_mute(struct snd_soc_dai *dai, int mute)
{
	int ret;
	struct tdf8532_priv *tdf8532 = snd_soc_codec_get_drvdata(dai->codec);
	unsigned char data[] = {0x02, packet_id++, 0x03, 0x80, MUTE, 0x1F};
	unsigned char data_gain_rom6_frmt[] = {0x02, packet_id++, 0x3A, 0x80, 0x24, 0x05, 0x01,
		0x00, 0x00, 0x00, 0x00, 0x1c, 0x9a, 0x02, 0x02,
		0x24, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x1c,
		0x9a, 0x02, 0x02, 0x24, 0x00, 0x03, 0x00, 0x00,
		0x00, 0x00, 0x1c, 0x9a, 0x02, 0x02, 0x24, 0x00,
		0x04, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x9a, 0x02,
		0x02, 0x24, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
		0xB4, 0x7B, 0x02, 0x02, 0x24, 0x00};
	unsigned char data_gain_rom7_frmt[] = {0x02, packet_id++, 0x2f, 0x80, 0x24, 0x04,
		0x01, 0x50, 0x03, 0x00, 0x00, 0x1a, 0x45, 0x02, 0x00, 0x24, 0x00,
		0x02, 0x50, 0x03, 0x01, 0x00, 0x1a, 0x45, 0x02, 0x00, 0x24, 0x00,
		0x03, 0x50, 0x03, 0x02, 0x00, 0x1a, 0x45, 0x02, 0x00, 0x24, 0x00,
		0x04, 0x50, 0x03, 0x03, 0x00, 0x1a, 0x45, 0x02, 0x00, 0x24, 0x00};

	if (!mute)
		data[4] = UNMUTE;
	else
		data[4] = MUTE;

	if (!mute) {
		if (tdf8532->rom_version == VERSION_6) {
			ret = i2c_master_send(tdf8532->i2c, data_gain_rom6_frmt, ARRAY_SIZE(data_gain_rom6_frmt));
			dev_dbg(dai->codec->dev,
				"%s: gain : i2c master send returned :%d\n", __func__,
				ret);

		} else if (tdf8532->rom_version == VERSION_7) {
			ret = i2c_master_send(tdf8532->i2c, data_gain_rom7_frmt, ARRAY_SIZE(data_gain_rom7_frmt));
			dev_dbg(dai->codec->dev,
				"%s: gain : i2c master send returned :%d\n", __func__,
				ret);
		} else {
			dev_dbg(dai->codec->dev, "Failed to set gain");
		}
	}

	ret = i2c_master_send(tdf8532->i2c, data, ARRAY_SIZE(data));
	dev_dbg(dai->codec->dev,
			"%s:i2c master send returned :%d\n", __func__,
				ret);

	return 0;
}
static int tdf8532_request_rom_vesion(struct tdf8532_priv *tdf8532)
{
	int ret;
	unsigned char req_info[] = {0x02, packet_id++, 0x02, 0x80, 0xE0};
	unsigned char get_first_reply[] = {0x2, 0x0, 0x0};
	unsigned char identification_msg[] = {0x2, 0x0, 0x8, 0x80, 0xE0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

	/* Send Identificaton Request: Write to address 0xD8 offset E0 */
	ret = i2c_master_send(tdf8532->i2c, req_info, ARRAY_SIZE(req_info));
	if (ret != ARRAY_SIZE(req_info)) {
		dev_err(&tdf8532->i2c->dev,
		"%s: GetIdentification_req Failed: i2c master send returned :%d\n", __func__,
		ret);
		return -EIO;
	}

	msleep(10);
	ret = i2c_master_recv(tdf8532->i2c, get_first_reply, ARRAY_SIZE(get_first_reply));
	if (ret != ARRAY_SIZE(get_first_reply))	{
		dev_err(&tdf8532->i2c->dev,
			"%s: GetIdentification_repl Failed: i2c master receive returned : %d\n", __func__,
			 ret);
		return -EIO;
	}

	msleep(10);
	ret = i2c_master_recv(tdf8532->i2c, identification_msg, ARRAY_SIZE(identification_msg));
	if (ret != ARRAY_SIZE(identification_msg)) {
		dev_err(&tdf8532->i2c->dev,
			"%s: GetIdentification Message Failed : i2c master receive returned : %d\n", __func__,
			ret);
		return -EIO;
	}

	tdf8532->rom_version = identification_msg[ROM_VERSION_OFFSET];

	return 0;
}
static int tdf8532_set_fast_mute(struct i2c_client *i2c)
{
	int ret;
	unsigned char data[] = {
		0x02, packet_id++, 0x06, 0x80, 0x18, 0x03, 0x01, 0x02, 0x00};

	ret = i2c_master_send(i2c, data, ARRAY_SIZE(data));

	dev_dbg(&i2c->dev, "%s:i2c master send returned :%d\n", __func__, ret);

	return ret;
}

static const struct snd_soc_dai_ops tdf8532_dai_ops = {
	.startup  = tdf8532_dai_startup,
	.hw_params  = tdf8532_dai_hw_params,
	.trigger  = tdf8532_dai_trigger,
	.prepare  = tdf8532_dai_prepare,
	.digital_mute = tdf8532_mute,
};

static struct snd_soc_codec_driver soc_codec_tdf8532;

static struct snd_soc_dai_driver tdf8532_dai[] = {
	{
		.name = "tdf8532-hifi",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &tdf8532_dai_ops,
	}
};


static int tdf8532_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{

	int ret;
	struct tdf8532_priv *tdf8532;

	dev_dbg(&i2c->dev, "%s\n", __func__);

	tdf8532 = devm_kzalloc(&i2c->dev, sizeof(*tdf8532),
			GFP_KERNEL);

	if (NULL == tdf8532)
		return -ENOMEM;

	tdf8532->i2c = i2c;
	i2c_set_clientdata(i2c, tdf8532);

	ret = tdf8532_request_rom_vesion(tdf8532);
	if (ret < 0)
		dev_err(&i2c->dev, "Failed to read rom version : %d\n", ret);

	ret = tdf8532_set_fast_mute(i2c);
	if (ret < 0)
		dev_err(&i2c->dev, "Failed to set fast mute option: %d\n", ret);

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_tdf8532,
			tdf8532_dai, ARRAY_SIZE(tdf8532_dai));
	if (ret != 0)
		dev_err(&i2c->dev, "%s failed!!\n", __func__);

	return ret;

}

static int tdf8532_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

static const struct i2c_device_id tdf8532_i2c_id[] = {
	{ "tdf8532", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tdf8532_i2c_id);

#if CONFIG_ACPI
static const struct acpi_device_id tdf8532_acpi_match[] = {
	{"INT34C3", 0},
	{},
};

MODULE_DEVICE_TABLE(acpi, tdf8532_acpi_match);
#endif

static struct i2c_driver tdf8532_i2c_driver = {
	.driver = {
		.name = "tdf8532-codec",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(tdf8532_acpi_match),
	},
	.probe =    tdf8532_i2c_probe,
	.remove =   tdf8532_i2c_remove,
	.id_table = tdf8532_i2c_id,
};

module_i2c_driver(tdf8532_i2c_driver);

MODULE_DESCRIPTION("ASoC NXP Semiconductors TDF8532 driver");
MODULE_AUTHOR("Steffen Wagner <steffen.wagner@intel.com>");
MODULE_LICENSE("GPL");
