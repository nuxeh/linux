/* Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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


#include <linux/nvs.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/export.h>


const char * const nvs_float_significances[] = {
	"micro",
	"nano",
};
EXPORT_SYMBOL(nvs_float_significances);

int nvs_of_dt(const struct device_node *np, struct sensor_cfg *cfg,
	      const char *dev_name)
{
	char str[256];
	const char *charp;
	unsigned int i;
	int lenp;
	int ret;

	if (np == NULL)
		return -EINVAL;

	if (!of_device_is_available(np))
		return -ENODEV;

	if (cfg == NULL)
		return -EINVAL;

	if (dev_name == NULL)
		dev_name = cfg->name;
	ret = sprintf(str, "%s_disable", dev_name);
	if (ret > 0) {
		ret = of_property_read_u32(np, str, (u32 *)&i);
		if (!ret) {
			if (i)
				cfg->snsr_id = -1;
		}
	}
	ret = sprintf(str, "%s_float_significance", dev_name);
	if (ret > 0) {
		if (!(of_property_read_string((struct device_node *)np,
					      str, &charp))) {
			for (i = 0; i < ARRAY_SIZE(nvs_float_significances);
									 i++) {
				if (!strcasecmp(charp,
						nvs_float_significances[i])) {
					cfg->float_significance = i;
					break;
				}
			}
		}
	}
	ret = sprintf(str, "%s_flags", dev_name);
	if (ret > 0) {
		ret = of_property_read_u32(np, str, (u32 *)&i);
		if (!ret) {
			cfg->flags &= SENSOR_FLAG_READONLY_MASK;
			i &= ~SENSOR_FLAG_READONLY_MASK;
			cfg->flags |= i;
		}
	}
	ret = sprintf(str, "%s_kbuffer_size", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->kbuf_sz);
	ret = sprintf(str, "%s_max_range_ival", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->max_range.ival);
	ret = sprintf(str, "%s_max_range_fval", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->max_range.fval);
	ret = sprintf(str, "%s_resolution_ival", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->resolution.ival);
	ret = sprintf(str, "%s_resolution_fval", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->resolution.fval);
	ret = sprintf(str, "%s_milliamp_ival", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->milliamp.ival);
	ret = sprintf(str, "%s_milliamp_fval", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->milliamp.fval);
	ret = sprintf(str, "%s_delay_us_min", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->delay_us_min);
	ret = sprintf(str, "%s_delay_us_max", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->delay_us_max);
	ret = sprintf(str, "%s_fifo_max_event_count", dev_name);
	if (ret > 0)
		of_property_read_u32(np, str, (u32 *)&cfg->fifo_rsrv_evnt_cnt);
	ret = sprintf(str, "%s_fifo_reserved_event_count", dev_name);
	if (ret > 0)
		of_property_read_u32(np, str, (u32 *)&cfg->fifo_max_evnt_cnt);
	ret = sprintf(str, "%s_matrix", dev_name);
	if (ret > 0) {
		charp = of_get_property(np, str, &lenp);
		if (charp && lenp == sizeof(cfg->matrix))
			memcpy(&cfg->matrix, charp, lenp);
	}
	ret = sprintf(str, "%s_uncalibrated_lo", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->uncal_lo);
	ret = sprintf(str, "%s_uncalibrated_hi", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->uncal_hi);
	ret = sprintf(str, "%s_calibrated_lo", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->cal_lo);
	ret = sprintf(str, "%s_calibrated_hi", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->cal_hi);
	ret = sprintf(str, "%s_threshold_lo", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->thresh_lo);
	ret = sprintf(str, "%s_threshold_hi", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->thresh_hi);
	ret = sprintf(str, "%s_report_count", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->report_n);
	ret = sprintf(str, "%s_scale_ival", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->scale.ival);
	ret = sprintf(str, "%s_scale_fval", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->scale.fval);
	ret = sprintf(str, "%s_offset_ival", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->offset.ival);
	ret = sprintf(str, "%s_offset_fval", dev_name);
	if (ret > 0)
		of_property_read_s32(np, str, (s32 *)&cfg->offset.fval);
	for (i = 0; i < NVS_CHANNEL_N_MAX; i++) {
		ret = sprintf(str, "%s_scale_ival_ch%u", dev_name, i);
		if (ret > 0)
			of_property_read_s32(np, str,
					     (s32 *)&cfg->scales[i].ival);
		ret = sprintf(str, "%s_scale_fval_ch%u", dev_name, i);
		if (ret > 0)
			of_property_read_s32(np, str,
					     (s32 *)&cfg->scales[i].fval);
		ret = sprintf(str, "%s_offset_ival_ch%u", dev_name, i);
		if (ret > 0)
			of_property_read_s32(np, str,
					     (s32 *)&cfg->offsets[i].ival);
		ret = sprintf(str, "%s_offset_fval_ch%u", dev_name, i);
		if (ret > 0)
			of_property_read_s32(np, str,
					     (s32 *)&cfg->offsets[i].fval);
	}
	return 0;
}

