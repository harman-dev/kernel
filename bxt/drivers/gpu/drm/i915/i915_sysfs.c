/*
 * Copyright © 2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Ben Widawsky <ben@bwidawsk.net>
 *
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/stat.h>
#include <linux/sysfs.h>
#include "intel_drv.h"
#include "i915_drv.h"
#include "../drm_internal.h"

static inline struct drm_i915_private *kdev_minor_to_i915(struct device *kdev)
{
	struct drm_minor *minor = dev_get_drvdata(kdev);
	return to_i915(minor->dev);
}

#ifdef CONFIG_PM
static u32 calc_residency(struct drm_i915_private *dev_priv,
			  i915_reg_t reg)
{
	u64 raw_time; /* 32b value may overflow during fixed point math */
	u64 units = 128ULL, div = 100000ULL;
	u32 ret;

	if (!intel_enable_rc6())
		return 0;

	intel_runtime_pm_get(dev_priv);

	/* On VLV and CHV, residency time is in CZ units rather than 1.28us */
	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) {
		units = 1;
		div = dev_priv->czclk_freq;

		if (I915_READ(VLV_COUNTER_CONTROL) & VLV_COUNT_RANGE_HIGH)
			units <<= 8;
	} else if (IS_GEN9_LP(dev_priv)) {
		units = 1;
		div = 1200;		/* 833.33ns */
	}

	raw_time = I915_READ(reg) * units;
	ret = DIV_ROUND_UP_ULL(raw_time, div);

	intel_runtime_pm_put(dev_priv);
	return ret;
}

static ssize_t
show_forcewake(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	ssize_t ret;
	unsigned count;

	count = dev_priv->uncore.fw_domain[FW_DOMAIN_ID_RENDER].wake_count;
	ret = snprintf(buf, PAGE_SIZE, "[%d]RENDER=0x%x\n",
			(int)FW_DOMAIN_ID_RENDER, count);

	count = dev_priv->uncore.fw_domain[FW_DOMAIN_ID_BLITTER].wake_count;
	ret += snprintf(buf, PAGE_SIZE, "[%d]BLITTER=0x%x\n",
			(int)FW_DOMAIN_ID_BLITTER, count);

	count = dev_priv->uncore.fw_domain[FW_DOMAIN_ID_MEDIA].wake_count;
	ret += snprintf(buf, PAGE_SIZE, "[%d]MEDIA=0x%x\n",
			(int)FW_DOMAIN_ID_MEDIA, count);

	return ret;
}

static ssize_t
forcewake_store(struct device *kdev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	if (val)
		intel_uncore_forcewake_get(dev_priv, FORCEWAKE_ALL);
	else
		intel_uncore_forcewake_put(dev_priv, FORCEWAKE_ALL);

	return count;
}

static ssize_t
show_rc6_mask(struct device *kdev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%x\n", intel_enable_rc6());
}

static ssize_t
show_rc6_ms(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 rc6_residency = calc_residency(dev_priv, GEN6_GT_GFX_RC6);
	return snprintf(buf, PAGE_SIZE, "%u\n", rc6_residency);
}

static ssize_t
show_rc6p_ms(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 rc6p_residency = calc_residency(dev_priv, GEN6_GT_GFX_RC6p);
	return snprintf(buf, PAGE_SIZE, "%u\n", rc6p_residency);
}

static ssize_t
show_rc6pp_ms(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 rc6pp_residency = calc_residency(dev_priv, GEN6_GT_GFX_RC6pp);
	return snprintf(buf, PAGE_SIZE, "%u\n", rc6pp_residency);
}

static ssize_t
show_media_rc6_ms(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 rc6_residency = calc_residency(dev_priv, VLV_GT_MEDIA_RC6);
	return snprintf(buf, PAGE_SIZE, "%u\n", rc6_residency);
}

static DEVICE_ATTR(forcewake, S_IRUSR | S_IWUSR, show_forcewake,
		   forcewake_store);
static DEVICE_ATTR(rc6_enable, S_IRUGO, show_rc6_mask, NULL);
static DEVICE_ATTR(rc6_residency_ms, S_IRUGO, show_rc6_ms, NULL);
static DEVICE_ATTR(rc6p_residency_ms, S_IRUGO, show_rc6p_ms, NULL);
static DEVICE_ATTR(rc6pp_residency_ms, S_IRUGO, show_rc6pp_ms, NULL);
static DEVICE_ATTR(media_rc6_residency_ms, S_IRUGO, show_media_rc6_ms, NULL);

static struct attribute *rc6_attrs[] = {
	&dev_attr_forcewake.attr,
	&dev_attr_rc6_enable.attr,
	&dev_attr_rc6_residency_ms.attr,
	NULL
};

static struct attribute_group rc6_attr_group = {
	.name = power_group_name,
	.attrs =  rc6_attrs
};

static struct attribute *rc6p_attrs[] = {
	&dev_attr_rc6p_residency_ms.attr,
	&dev_attr_rc6pp_residency_ms.attr,
	NULL
};

static struct attribute_group rc6p_attr_group = {
	.name = power_group_name,
	.attrs =  rc6p_attrs
};

static struct attribute *media_rc6_attrs[] = {
	&dev_attr_media_rc6_residency_ms.attr,
	NULL
};

static struct attribute_group media_rc6_attr_group = {
	.name = power_group_name,
	.attrs =  media_rc6_attrs
};
#endif

static int l3_access_valid(struct drm_i915_private *dev_priv, loff_t offset)
{
	if (!HAS_L3_DPF(dev_priv))
		return -EPERM;

	if (offset % 4 != 0)
		return -EINVAL;

	if (offset >= GEN7_L3LOG_SIZE)
		return -ENXIO;

	return 0;
}

static ssize_t
i915_l3_read(struct file *filp, struct kobject *kobj,
	     struct bin_attribute *attr, char *buf,
	     loff_t offset, size_t count)
{
	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct drm_device *dev = &dev_priv->drm;
	int slice = (int)(uintptr_t)attr->private;
	int ret;

	count = round_down(count, 4);

	ret = l3_access_valid(dev_priv, offset);
	if (ret)
		return ret;

	count = min_t(size_t, GEN7_L3LOG_SIZE - offset, count);

	ret = i915_mutex_lock_interruptible(dev);
	if (ret)
		return ret;

	if (dev_priv->l3_parity.remap_info[slice])
		memcpy(buf,
		       dev_priv->l3_parity.remap_info[slice] + (offset/4),
		       count);
	else
		memset(buf, 0, count);

	mutex_unlock(&dev->struct_mutex);

	return count;
}

static ssize_t
i915_l3_write(struct file *filp, struct kobject *kobj,
	      struct bin_attribute *attr, char *buf,
	      loff_t offset, size_t count)
{
	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct drm_device *dev = &dev_priv->drm;
	struct i915_gem_context *ctx;
	u32 *temp = NULL; /* Just here to make handling failures easy */
	int slice = (int)(uintptr_t)attr->private;
	int ret;

	if (!HAS_HW_CONTEXTS(dev_priv))
		return -ENXIO;

	ret = l3_access_valid(dev_priv, offset);
	if (ret)
		return ret;

	ret = i915_mutex_lock_interruptible(dev);
	if (ret)
		return ret;

	if (!dev_priv->l3_parity.remap_info[slice]) {
		temp = kzalloc(GEN7_L3LOG_SIZE, GFP_KERNEL);
		if (!temp) {
			mutex_unlock(&dev->struct_mutex);
			return -ENOMEM;
		}
	}

	/* TODO: Ideally we really want a GPU reset here to make sure errors
	 * aren't propagated. Since I cannot find a stable way to reset the GPU
	 * at this point it is left as a TODO.
	*/
	if (temp)
		dev_priv->l3_parity.remap_info[slice] = temp;

	memcpy(dev_priv->l3_parity.remap_info[slice] + (offset/4), buf, count);

	/* NB: We defer the remapping until we switch to the context */
	list_for_each_entry(ctx, &dev_priv->context_list, link)
		ctx->remap_slice |= (1<<slice);

	mutex_unlock(&dev->struct_mutex);

	return count;
}

static struct bin_attribute dpf_attrs = {
	.attr = {.name = "l3_parity", .mode = (S_IRUSR | S_IWUSR)},
	.size = GEN7_L3LOG_SIZE,
	.read = i915_l3_read,
	.write = i915_l3_write,
	.mmap = NULL,
	.private = (void *)0
};

static struct bin_attribute dpf_attrs_1 = {
	.attr = {.name = "l3_parity_slice_1", .mode = (S_IRUSR | S_IWUSR)},
	.size = GEN7_L3LOG_SIZE,
	.read = i915_l3_read,
	.write = i915_l3_write,
	.mmap = NULL,
	.private = (void *)1
};

static ssize_t gt_act_freq_mhz_show(struct device *kdev,
				    struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	int ret;

	intel_runtime_pm_get(dev_priv);

	mutex_lock(&dev_priv->rps.hw_lock);
	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) {
		u32 freq;
		freq = vlv_punit_read(dev_priv, PUNIT_REG_GPU_FREQ_STS);
		ret = intel_gpu_freq(dev_priv, (freq >> 8) & 0xff);
	} else {
		u32 rpstat = I915_READ(GEN6_RPSTAT1);
		if (IS_GEN9(dev_priv))
			ret = (rpstat & GEN9_CAGF_MASK) >> GEN9_CAGF_SHIFT;
		else if (IS_HASWELL(dev_priv) || IS_BROADWELL(dev_priv))
			ret = (rpstat & HSW_CAGF_MASK) >> HSW_CAGF_SHIFT;
		else
			ret = (rpstat & GEN6_CAGF_MASK) >> GEN6_CAGF_SHIFT;
		ret = intel_gpu_freq(dev_priv, ret);
	}
	mutex_unlock(&dev_priv->rps.hw_lock);

	intel_runtime_pm_put(dev_priv);

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t gt_cur_freq_mhz_show(struct device *kdev,
				    struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			intel_gpu_freq(dev_priv,
				       dev_priv->rps.cur_freq));
}

static ssize_t gt_boost_freq_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			intel_gpu_freq(dev_priv,
				       dev_priv->rps.boost_freq));
}

static ssize_t gt_boost_freq_mhz_store(struct device *kdev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	/* Validate against (static) hardware limits */
	val = intel_freq_opcode(dev_priv, val);
	if (val < dev_priv->rps.min_freq || val > dev_priv->rps.max_freq)
		return -EINVAL;

	mutex_lock(&dev_priv->rps.hw_lock);
	dev_priv->rps.boost_freq = val;
	mutex_unlock(&dev_priv->rps.hw_lock);

	return count;
}

static ssize_t vlv_rpe_freq_mhz_show(struct device *kdev,
				     struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			intel_gpu_freq(dev_priv,
				       dev_priv->rps.efficient_freq));
}

static ssize_t gt_max_freq_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			intel_gpu_freq(dev_priv,
				       dev_priv->rps.max_freq_softlimit));
}

static ssize_t gt_max_freq_mhz_store(struct device *kdev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	intel_runtime_pm_get(dev_priv);

	mutex_lock(&dev_priv->rps.hw_lock);

	val = intel_freq_opcode(dev_priv, val);

	if (val < dev_priv->rps.min_freq ||
	    val > dev_priv->rps.max_freq ||
	    val < dev_priv->rps.min_freq_softlimit) {
		mutex_unlock(&dev_priv->rps.hw_lock);
		intel_runtime_pm_put(dev_priv);
		return -EINVAL;
	}

	if (val > dev_priv->rps.rp0_freq)
		DRM_DEBUG("User requested overclocking to %d\n",
			  intel_gpu_freq(dev_priv, val));

	dev_priv->rps.max_freq_softlimit = val;

	val = clamp_t(int, dev_priv->rps.cur_freq,
		      dev_priv->rps.min_freq_softlimit,
		      dev_priv->rps.max_freq_softlimit);

	/* We still need *_set_rps to process the new max_delay and
	 * update the interrupt limits and PMINTRMSK even though
	 * frequency request may be unchanged. */
	intel_set_rps(dev_priv, val);

	mutex_unlock(&dev_priv->rps.hw_lock);

	intel_runtime_pm_put(dev_priv);

	return count;
}

static ssize_t gt_min_freq_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			intel_gpu_freq(dev_priv,
				       dev_priv->rps.min_freq_softlimit));
}

static ssize_t gt_min_freq_mhz_store(struct device *kdev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	intel_runtime_pm_get(dev_priv);

	mutex_lock(&dev_priv->rps.hw_lock);

	val = intel_freq_opcode(dev_priv, val);

	if (val < dev_priv->rps.min_freq ||
	    val > dev_priv->rps.max_freq ||
	    val > dev_priv->rps.max_freq_softlimit) {
		mutex_unlock(&dev_priv->rps.hw_lock);
		intel_runtime_pm_put(dev_priv);
		return -EINVAL;
	}

	dev_priv->rps.min_freq_softlimit = val;

	val = clamp_t(int, dev_priv->rps.cur_freq,
		      dev_priv->rps.min_freq_softlimit,
		      dev_priv->rps.max_freq_softlimit);

	/* We still need *_set_rps to process the new min_delay and
	 * update the interrupt limits and PMINTRMSK even though
	 * frequency request may be unchanged. */
	intel_set_rps(dev_priv, val);

	mutex_unlock(&dev_priv->rps.hw_lock);

	intel_runtime_pm_put(dev_priv);

	return count;

}

static DEVICE_ATTR(gt_act_freq_mhz, S_IRUGO, gt_act_freq_mhz_show, NULL);
static DEVICE_ATTR(gt_cur_freq_mhz, S_IRUGO, gt_cur_freq_mhz_show, NULL);
static DEVICE_ATTR(gt_boost_freq_mhz, S_IRUGO | S_IWUSR, gt_boost_freq_mhz_show, gt_boost_freq_mhz_store);
static DEVICE_ATTR(gt_max_freq_mhz, S_IRUGO | S_IWUSR, gt_max_freq_mhz_show, gt_max_freq_mhz_store);
static DEVICE_ATTR(gt_min_freq_mhz, S_IRUGO | S_IWUSR, gt_min_freq_mhz_show, gt_min_freq_mhz_store);

static DEVICE_ATTR(vlv_rpe_freq_mhz, S_IRUGO, vlv_rpe_freq_mhz_show, NULL);

static ssize_t gt_rp_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(gt_RP0_freq_mhz, S_IRUGO, gt_rp_mhz_show, NULL);
static DEVICE_ATTR(gt_RP1_freq_mhz, S_IRUGO, gt_rp_mhz_show, NULL);
static DEVICE_ATTR(gt_RPn_freq_mhz, S_IRUGO, gt_rp_mhz_show, NULL);

/* For now we have a static number of RP states */
static ssize_t gt_rp_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 val;

	if (attr == &dev_attr_gt_RP0_freq_mhz)
		val = intel_gpu_freq(dev_priv, dev_priv->rps.rp0_freq);
	else if (attr == &dev_attr_gt_RP1_freq_mhz)
		val = intel_gpu_freq(dev_priv, dev_priv->rps.rp1_freq);
	else if (attr == &dev_attr_gt_RPn_freq_mhz)
		val = intel_gpu_freq(dev_priv, dev_priv->rps.min_freq);
	else
		BUG();

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t thaw_show(struct device *kdev, struct device_attribute *attr,
			 char *buf)
{
	return 0;
}
static DEVICE_ATTR(thaw, S_IRUGO, thaw_show, NULL);


static const struct attribute *gen6_attrs[] = {
	&dev_attr_gt_act_freq_mhz.attr,
	&dev_attr_gt_cur_freq_mhz.attr,
	&dev_attr_gt_boost_freq_mhz.attr,
	&dev_attr_gt_max_freq_mhz.attr,
	&dev_attr_gt_min_freq_mhz.attr,
	&dev_attr_gt_RP0_freq_mhz.attr,
	&dev_attr_gt_RP1_freq_mhz.attr,
	&dev_attr_gt_RPn_freq_mhz.attr,
	&dev_attr_thaw.attr,
	NULL,
};

static const struct attribute *vlv_attrs[] = {
	&dev_attr_gt_act_freq_mhz.attr,
	&dev_attr_gt_cur_freq_mhz.attr,
	&dev_attr_gt_boost_freq_mhz.attr,
	&dev_attr_gt_max_freq_mhz.attr,
	&dev_attr_gt_min_freq_mhz.attr,
	&dev_attr_gt_RP0_freq_mhz.attr,
	&dev_attr_gt_RP1_freq_mhz.attr,
	&dev_attr_gt_RPn_freq_mhz.attr,
	&dev_attr_vlv_rpe_freq_mhz.attr,
	&dev_attr_thaw.attr,
	NULL,
};

#if IS_ENABLED(CONFIG_DRM_I915_CAPTURE_ERROR)

static ssize_t error_state_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *attr, char *buf,
				loff_t off, size_t count)
{

	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct drm_device *dev = &dev_priv->drm;
	struct i915_error_state_file_priv error_priv;
	struct drm_i915_error_state_buf error_str;
	ssize_t ret_count = 0;
	int ret;

	memset(&error_priv, 0, sizeof(error_priv));

	ret = i915_error_state_buf_init(&error_str, to_i915(dev), count, off);
	if (ret)
		return ret;

	error_priv.i915 = dev_priv;
	i915_error_state_get(dev, &error_priv);

	ret = i915_error_state_to_str(&error_str, &error_priv);
	if (ret)
		goto out;

	ret_count = count < error_str.bytes ? count : error_str.bytes;

	memcpy(buf, error_str.buf, ret_count);
out:
	i915_error_state_put(&error_priv);
	i915_error_state_buf_release(&error_str);

	return ret ?: ret_count;
}

static ssize_t error_state_write(struct file *file, struct kobject *kobj,
				 struct bin_attribute *attr, char *buf,
				 loff_t off, size_t count)
{
	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);

	DRM_DEBUG_DRIVER("Resetting error state\n");
	i915_destroy_error_state(dev_priv);

	return count;
}

static ssize_t i915_gem_clients_state_read(struct file *filp,
				struct kobject *memtrack_kobj,
				struct bin_attribute *attr,
				char *buf, loff_t off, size_t count)
{
	struct kobject *kobj = memtrack_kobj->parent;
	struct device *kdev = container_of(kobj, struct device, kobj);
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct drm_device *dev = &dev_priv->drm;
	struct drm_i915_error_state_buf error_str;
	ssize_t ret_count = 0;
	int ret;

	ret = i915_error_state_buf_init(&error_str, dev_priv, count, off);
	if (ret)
		return ret;

	ret = i915_get_drm_clients_info(&error_str, dev);
	if (ret)
		goto out;

	ret_count = count < error_str.bytes ? count : error_str.bytes;

	memcpy(buf, error_str.buf, ret_count);
out:
	i915_error_state_buf_release(&error_str);

	return ret ?: ret_count;
}

#define GEM_OBJ_STAT_BUF_SIZE (4*1024) /* 4KB */
#define GEM_OBJ_STAT_BUF_SIZE_MAX (1024*1024) /* 1MB */

struct i915_gem_file_attr_priv {
	char tgid_str[16];
	struct pid *tgid;
	struct drm_i915_error_state_buf buf;
};

static ssize_t i915_gem_read_objects(struct file *filp,
				struct kobject *memtrack_kobj,
				struct bin_attribute *attr,
				char *buf, loff_t off, size_t count)
{
	struct kobject *kobj = memtrack_kobj->parent;
	struct device *kdev = container_of(kobj, struct device, kobj);
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
        struct drm_device *dev = &dev_priv->drm;
	struct i915_gem_file_attr_priv *attr_priv;
	struct pid *tgid;
	ssize_t ret_count = 0;
	long bytes_available;
	int ret = 0, buf_size = GEM_OBJ_STAT_BUF_SIZE;
	unsigned long timeout = msecs_to_jiffies(500) + 1;

	/*
	 * There may arise a scenario where syfs file entry is being removed,
	 * and may race against sysfs read. Sysfs file remove function would
	 * have taken the drm_global_mutex and would wait for read to finish,
	 * which is again waiting to acquire drm_global_mutex, leading to
	 * deadlock. To avoid this, use mutex_trylock here with a timeout.
	 */
	while (!mutex_trylock(&drm_global_mutex) && --timeout)
		schedule_timeout_killable(1);
	if (timeout == 0) {
		DRM_DEBUG_DRIVER("Unable to acquire drm global mutex.\n");
		return -EBUSY;
	}

	if (!attr || !attr->private) {
		ret = -EINVAL;
		DRM_ERROR("attr | attr->private pointer is NULL\n");
		goto out;
	}

	attr_priv = attr->private;
	tgid = attr_priv->tgid;

	if (off && !attr_priv->buf.buf) {
		ret = -EINVAL;
		DRM_ERROR(
			"Buf not allocated during read with non-zero offset\n");
		goto out;
	}

	if (off == 0) {
retry:
		if (!attr_priv->buf.buf) {
			ret = i915_obj_state_buf_init(&attr_priv->buf,
				buf_size);
			if (ret) {
				DRM_ERROR(
					"obj state buf init failed. buf_size=%d\n",
					buf_size);
				goto out;
			}
		} else {
			/* Reset the buf parameters before filling data */
			attr_priv->buf.pos = 0;
			attr_priv->buf.bytes = 0;
		}

		/* Read the gfx device stats */
		ret = i915_gem_get_obj_info(&attr_priv->buf, dev, tgid);
		if (ret)
			goto out;

		ret = i915_error_ok(&attr_priv->buf);
		if (ret) {
			ret = 0;
			goto copy_data;
		}
		if (buf_size >= GEM_OBJ_STAT_BUF_SIZE_MAX) {
			DRM_DEBUG_DRIVER("obj stat buf size limit reached\n");
			ret = -ENOMEM;
			goto out;
		} else {
			/* Try to reallocate buf of larger size */
			i915_error_state_buf_release(&attr_priv->buf);
			buf_size *= 2;

			ret = i915_obj_state_buf_init(&attr_priv->buf,
						buf_size);
			if (ret) {
				DRM_ERROR(
					"obj stat buf init failed. buf_size=%d\n",
					buf_size);
				goto out;
			}
			goto retry;
		}
	}
copy_data:

	bytes_available = (long)attr_priv->buf.bytes - (long)off;

	if (bytes_available > 0) {
		ret_count = count < bytes_available ? count : bytes_available;
		memcpy(buf, attr_priv->buf.buf + off, ret_count);
	} else
		ret_count = 0;

out:
	mutex_unlock(&drm_global_mutex);

	return ret ?: ret_count;
}

int i915_gem_create_sysfs_file_entry(struct drm_device *dev,
					struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_gem_file_attr_priv *attr_priv;
	struct bin_attribute *obj_attr;
	struct drm_file *file_local;
	int ret;

	if (!i915.memtrack_debug)
		return 0;

	/*
	 * Check for multiple drm files having same tgid. If found, copy the
	 * bin attribute into the new file priv. Otherwise allocate a new
	 * copy of bin attribute, and create its corresponding sysfs file.
	 */
	mutex_lock(&dev->struct_mutex);
	list_for_each_entry(file_local, &dev->filelist, lhead) {
		struct drm_i915_file_private *file_priv_local =
				file_local->driver_priv;

		if (file_priv->tgid == file_priv_local->tgid) {
			file_priv->obj_attr = file_priv_local->obj_attr;
			mutex_unlock(&dev->struct_mutex);
			return 0;
		}
	}
	mutex_unlock(&dev->struct_mutex);

	obj_attr = kzalloc(sizeof(*obj_attr), GFP_KERNEL);
	if (!obj_attr) {
		DRM_ERROR("Alloc failed. Out of memory\n");
		ret = -ENOMEM;
		goto out;
	}

	attr_priv = kzalloc(sizeof(*attr_priv), GFP_KERNEL);
	if (!attr_priv) {
		DRM_ERROR("Alloc failed. Out of memory\n");
		ret = -ENOMEM;
		goto out_obj_attr;
	}

	snprintf(attr_priv->tgid_str, 16, "%d", task_tgid_nr(current));
	obj_attr->attr.name = attr_priv->tgid_str;
	obj_attr->attr.mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
	obj_attr->size = 0;
	obj_attr->read = i915_gem_read_objects;

	attr_priv->tgid = file_priv->tgid;
	obj_attr->private = attr_priv;

	ret = sysfs_create_bin_file(&dev_priv->memtrack_kobj,
				   obj_attr);
	if (ret) {
		DRM_ERROR(
			"sysfs tgid file setup failed. tgid=%d, process:%s, ret:%d\n",
			pid_nr(file_priv->tgid), file_priv->process_name, ret);

		goto out_attr_priv;
	}

	file_priv->obj_attr = obj_attr;
	return 0;

out_attr_priv:
	kfree(attr_priv);
out_obj_attr:
	kfree(obj_attr);
out:
	return ret;
}

void i915_gem_remove_sysfs_file_entry(struct drm_device *dev,
			struct drm_file *file)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct drm_file *file_local;
	int open_count = 1;

	if (!i915.memtrack_debug)
		return;

	/*
	 * The current drm file instance is already removed from filelist at
	 * this point.
	 * Check if this particular drm file being removed is the last one for
	 * that particular tgid, and no other instances for this tgid exist in
	 * the filelist. If so, remove the corresponding sysfs file entry also.
	 */
	list_for_each_entry(file_local, &dev->filelist, lhead) {
		struct drm_i915_file_private *file_priv_local =
				file_local->driver_priv;

		if (pid_nr(file_priv->tgid) == pid_nr(file_priv_local->tgid))
			open_count++;
	}

	if (open_count == 1) {
		struct i915_gem_file_attr_priv *attr_priv;

		if (WARN_ON(file_priv->obj_attr == NULL))
			return;
		attr_priv = file_priv->obj_attr->private;

		sysfs_remove_bin_file(&dev_priv->memtrack_kobj,
				file_priv->obj_attr);

		i915_error_state_buf_release(&attr_priv->buf);
		kfree(file_priv->obj_attr->private);
		kfree(file_priv->obj_attr);
	}
}

static struct bin_attribute error_state_attr = {
	.attr.name = "error",
	.attr.mode = S_IRUSR | S_IWUSR,
	.size = 0,
	.read = error_state_read,
	.write = error_state_write,
};

static void i915_setup_error_capture(struct device *kdev)
{
	if (sysfs_create_bin_file(&kdev->kobj, &error_state_attr))
		DRM_ERROR("error_state sysfs setup failed\n");
}

static void i915_teardown_error_capture(struct device *kdev)
{
	sysfs_remove_bin_file(&kdev->kobj, &error_state_attr);
}
#else
static void i915_setup_error_capture(struct device *kdev) {}
static void i915_teardown_error_capture(struct device *kdev) {}
#endif

static struct bin_attribute i915_gem_client_state_attr = {
	.attr.name = "i915_gem_meminfo",
	.attr.mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
	.size = 0,
	.read = i915_gem_clients_state_read,
};

static struct attribute *memtrack_kobj_attrs[] = {NULL};

static struct kobj_type memtrack_kobj_type = {
	.release = NULL,
	.sysfs_ops = NULL,
	.default_attrs = memtrack_kobj_attrs,
};

void i915_setup_sysfs(struct drm_i915_private *dev_priv)
{
	struct device *kdev = dev_priv->drm.primary->kdev;
	int ret;

#ifdef CONFIG_PM
	if (HAS_RC6(dev_priv)) {
		ret = sysfs_merge_group(&kdev->kobj,
					&rc6_attr_group);
		if (ret)
			DRM_ERROR("RC6 residency sysfs setup failed\n");
	}
	if (HAS_RC6p(dev_priv)) {
		ret = sysfs_merge_group(&kdev->kobj,
					&rc6p_attr_group);
		if (ret)
			DRM_ERROR("RC6p residency sysfs setup failed\n");
	}
	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) {
		ret = sysfs_merge_group(&kdev->kobj,
					&media_rc6_attr_group);
		if (ret)
			DRM_ERROR("Media RC6 residency sysfs setup failed\n");
	}
#endif
	if (HAS_L3_DPF(dev_priv)) {
		ret = device_create_bin_file(kdev, &dpf_attrs);
		if (ret)
			DRM_ERROR("l3 parity sysfs setup failed\n");

		if (NUM_L3_SLICES(dev_priv) > 1) {
			ret = device_create_bin_file(kdev,
						     &dpf_attrs_1);
			if (ret)
				DRM_ERROR("l3 parity slice 1 setup failed\n");
		}
	}

	ret = 0;
	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		ret = sysfs_create_files(&kdev->kobj, vlv_attrs);
	else if (INTEL_GEN(dev_priv) >= 6)
		ret = sysfs_create_files(&kdev->kobj, gen6_attrs);
	if (ret)
		DRM_ERROR("RPS sysfs setup failed\n");

	i915_setup_error_capture(kdev);

	if (i915.memtrack_debug) {
		struct drm_device *dev = &dev_priv->drm;

		/*
		 * Create the gfx_memtrack directory for memtrack sysfs files
		 */
		ret = kobject_init_and_add(
			&dev_priv->memtrack_kobj, &memtrack_kobj_type,
			&dev->primary->kdev->kobj, "gfx_memtrack");
		if (unlikely(ret != 0)) {
			DRM_ERROR(
				"i915 sysfs setup memtrack directory failed\n"
				);
			kobject_put(&dev_priv->memtrack_kobj);
		} else {
			ret = sysfs_create_bin_file(&dev_priv->memtrack_kobj,
					    &i915_gem_client_state_attr);
			if (ret)
				DRM_ERROR(
					  "i915_gem_client_state sysfs setup failed\n"
				);
		}
	}
}

void i915_teardown_sysfs(struct drm_i915_private *dev_priv)
{
	struct device *kdev = dev_priv->drm.primary->kdev;

	i915_teardown_error_capture(kdev);

	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		sysfs_remove_files(&kdev->kobj, vlv_attrs);
	else
		sysfs_remove_files(&kdev->kobj, gen6_attrs);
	device_remove_bin_file(kdev,  &dpf_attrs_1);
	device_remove_bin_file(kdev,  &dpf_attrs);
#ifdef CONFIG_PM
	sysfs_unmerge_group(&kdev->kobj, &rc6_attr_group);
	sysfs_unmerge_group(&kdev->kobj, &rc6p_attr_group);
#endif
	if (i915.memtrack_debug) {
		sysfs_remove_bin_file(&dev_priv->memtrack_kobj,
					&i915_gem_client_state_attr);
		kobject_del(&dev_priv->memtrack_kobj);
		kobject_put(&dev_priv->memtrack_kobj);
	}
}
