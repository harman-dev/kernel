/*
 * thermal iio interface driver
 * Copyright (c) 2015, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/thermal.h>

struct thermal_iio_data {
	struct thermal_zone_device *tz;
	struct iio_trigger *dready_trig;
	s16 buffer[8];
	bool enable;
	long temp_thres;
	bool ev_enable_state;
	struct mutex mutex;

};

static const struct iio_event_spec thermal_event = {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE)
};

#define THERMAL_TEMP_CHANNELS {					\
	{								\
		.type = IIO_TEMP,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.scan_index = 0,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 32,				\
			.storagebits = 32,				\
			.shift = 0,					\
			.endianness = IIO_CPU,				\
		},							\
		.event_spec = &thermal_event,				\
		.num_event_specs = 1					\
	},								\
}

static const struct iio_chan_spec thermal_iio_channels[] =
							THERMAL_TEMP_CHANNELS;

static int thermal_iio_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct thermal_iio_data *iio_data = iio_priv(indio_dev);
	long temp;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = thermal_zone_get_temp(iio_data->tz, &temp);
		if (ret)
			return ret;
		*val = (int) temp;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t thermal_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct thermal_iio_data *iio_data = iio_priv(indio_dev);
	long temp;
	int ret;

	ret = thermal_zone_get_temp(iio_data->tz, &temp);
	if (ret)
		goto err_read;

	*(s32 *)iio_data->buffer = (s32)temp;
	iio_push_to_buffers(indio_dev, iio_data->buffer);
err_read:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int thermal_data_rdy_trigger_set_state(struct iio_trigger *trig,
					      bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct thermal_iio_data *iio_data = iio_priv(indio_dev);

	mutex_lock(&iio_data->mutex);
	iio_data->enable = state;
	mutex_unlock(&iio_data->mutex);

	return 0;
}

static const struct iio_trigger_ops thermal_trigger_ops = {
	.set_trigger_state = thermal_data_rdy_trigger_set_state,
	.owner = THIS_MODULE,
};

static int thermal_iio_read_event(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  enum iio_event_type type,
				  enum iio_event_direction dir,
				  enum iio_event_info info,
				  int *val, int *val2)
{
	struct thermal_iio_data *iio_data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&iio_data->mutex);
	*val2 = 0;
	switch (info) {
	case IIO_EV_INFO_VALUE:
		*val = iio_data->temp_thres;
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&iio_data->mutex);

	return ret;
}

static int thermal_iio_write_event(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info,
				   int val, int val2)
{
	struct thermal_iio_data *iio_data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&iio_data->mutex);
	if (iio_data->ev_enable_state) {
		ret = -EBUSY;
		goto done_write_event;
	}
	switch (info) {
	case IIO_EV_INFO_VALUE:
		iio_data->temp_thres = val;
		break;
	default:
		ret = -EINVAL;
		break;
	}
done_write_event:
	mutex_unlock(&iio_data->mutex);

	return ret;
}

static int thermal_iio_read_event_config(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan,
					 enum iio_event_type type,
					 enum iio_event_direction dir)
{

	struct thermal_iio_data *iio_data = iio_priv(indio_dev);
	bool state;

	mutex_lock(&iio_data->mutex);
	state = iio_data->ev_enable_state;
	mutex_unlock(&iio_data->mutex);

	return state;
}

static int thermal_iio_write_event_config(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir,
					  int state)
{
	struct thermal_iio_data *iio_data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&iio_data->mutex);
	if (state && iio_data->ev_enable_state)
		goto done_write_event;

	if (iio_data->tz->ops->set_threshold_temp)
		ret = iio_data->tz->ops->set_threshold_temp(iio_data->tz, 0,
							iio_data->temp_thres);
	iio_data->ev_enable_state = state;

done_write_event:
	mutex_unlock(&iio_data->mutex);

	return ret;
}

static const struct iio_info thermal_iio_info = {
	.read_raw		= thermal_iio_read_raw,
	.read_event_value	= thermal_iio_read_event,
	.write_event_value	= thermal_iio_write_event,
	.write_event_config	= thermal_iio_write_event_config,
	.read_event_config	= thermal_iio_read_event_config,
	.driver_module		= THIS_MODULE,
};

int thermal_iio_sensor_register(struct thermal_zone_device *tz)
{
	struct thermal_iio_data *iio_data;
	int ret;

	tz->indio_dev = devm_iio_device_alloc(&tz->device, sizeof(*iio_data));
	if (!tz->indio_dev)
		return -ENOMEM;

	iio_data = iio_priv(tz->indio_dev);
	iio_data->tz = tz;
	mutex_init(&iio_data->mutex);

	tz->indio_dev->dev.parent = &tz->device;
	tz->indio_dev->channels = thermal_iio_channels;
	tz->indio_dev->num_channels = ARRAY_SIZE(thermal_iio_channels);
	tz->indio_dev->name = tz->type;
	tz->indio_dev->info = &thermal_iio_info;
	tz->indio_dev->modes = INDIO_DIRECT_MODE;

	iio_data->dready_trig = devm_iio_trigger_alloc(&tz->device, "%s-dev%d",
						       tz->type,
						       tz->indio_dev->id);
	if (iio_data->dready_trig == NULL) {
		dev_err(&tz->device, "Trigger Allocate Failed\n");
		return -ENOMEM;
	}

	iio_data->dready_trig->dev.parent = &tz->device;
	iio_data->dready_trig->ops = &thermal_trigger_ops;
	iio_trigger_set_drvdata(iio_data->dready_trig, tz->indio_dev);
	tz->indio_dev->trig = iio_data->dready_trig;
	iio_trigger_get(tz->indio_dev->trig);
	ret = iio_trigger_register(iio_data->dready_trig);
	if (ret) {
		dev_err(&tz->device, "Trigger Allocate Failed\n");
		return ret;
	}

	ret = iio_triggered_buffer_setup(tz->indio_dev,
					 &iio_pollfunc_store_time,
					 thermal_trigger_handler, NULL);
	if (ret) {
		dev_err(&tz->device, "failed to initialize trigger buffer\n");
		goto err_unreg_trig;
	}

	ret = iio_device_register(tz->indio_dev);
	if (ret < 0) {
		dev_err(&tz->device, "unable to register iio device\n");
		goto err_cleanup_trig;
	}

	return 0;

err_cleanup_trig:
	iio_triggered_buffer_cleanup(tz->indio_dev);
err_unreg_trig:
	iio_device_unregister(tz->indio_dev);

	return ret;
}

int thermal_iio_sensor_unregister(struct thermal_zone_device *tz)
{
	struct thermal_iio_data *iio_data = iio_priv(tz->indio_dev);

	iio_device_unregister(tz->indio_dev);
	iio_triggered_buffer_cleanup(tz->indio_dev);
	iio_trigger_unregister(iio_data->dready_trig);

	return 0;
}

#define IIO_EVENT_CODE_THERMAL_THRES IIO_UNMOD_EVENT_CODE(IIO_TEMP, 0,\
							  IIO_EV_TYPE_THRESH,\
							  IIO_EV_DIR_EITHER)

#define IIO_EVENT_CODE_TRIP_UPDATE IIO_UNMOD_EVENT_CODE(IIO_TEMP, 0,\
							IIO_EV_TYPE_CHANGE,\
							IIO_EV_DIR_NONE)

int thermal_iio_sensor_notify(struct thermal_zone_device *tz,
			      enum thermal_zone_event_type event)
{
	struct thermal_iio_data *iio_data = iio_priv(tz->indio_dev);
	long temp = 0;
	int ret;

	mutex_lock(&iio_data->mutex);
	if (iio_data->ev_enable_state) {
		if (event == THERMAL_TEMP_THRESHOLD)
			iio_push_event(tz->indio_dev,
				       IIO_EVENT_CODE_THERMAL_THRES,
				       iio_get_time_ns());
		else if (event == THERMAL_TRIP_UPDATE)
			iio_push_event(tz->indio_dev,
				       IIO_EVENT_CODE_TRIP_UPDATE,
				       iio_get_time_ns());
		else
			dev_err(&tz->device, "invalid event\n");
	}
	if (iio_data->enable) {
		ret = thermal_zone_get_temp(iio_data->tz, &temp);
		if (ret)
			goto err_read;
		*(s32 *)iio_data->buffer = (s32)temp;
		iio_push_to_buffers(tz->indio_dev, iio_data->buffer);
	}
	mutex_unlock(&iio_data->mutex);

	return 0;
err_read:
	mutex_unlock(&iio_data->mutex);
	return ret;
}
