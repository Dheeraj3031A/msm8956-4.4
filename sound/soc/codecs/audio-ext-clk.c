/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/clk/msm-clk-provider.h>
#include <linux/clk/msm-clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <sound/q6afe-v2.h>

#ifndef CONFIG_ARCH_MSM8916
#include <dt-bindings/clock/audio-ext-clk.h>
#else
 #ifdef clk_audio_ap_clk
  #undef clk_audio_ap_clk
 #endif

 #ifdef clk_audio_pmi_clk
  #undef clk_audio_pmi_clk
 #endif

#include <dt-bindings/clock/msm-clocks-8976.h>

/* Hacky: those clocks are not present in 8916-class SoC!! */
#define clk_audio_ap_clk2       0x454d1e91
#define clk_audio_lpass_mclk    0xf0f2a284
#define clk_audio_pmi_lnbb_clk   0x57312343
#endif

struct pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *sleep;
	struct pinctrl_state *active;
};

#if defined(CONFIG_WCD9335_CODEC_MCLK_USE_MSM_GPIO)
#include <sound/q6afe-v2.h>
#include <linux/io.h>

#define LPASS_CSR_GP_IO_MUX_MIC_CTL 0x07702000
#define LPASS_CSR_GP_IO_MUX_SPKR_CTL 0x07702004

enum ap_clk_mux {
	AP_CLK1 = 1,
	AP_CLK2,
};
struct pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *sleep;
	struct pinctrl_state *active;
};
#endif

struct audio_ext_ap_clk {
	bool enabled;
	int gpio;
	struct clk c;
};

struct audio_ext_pmi_clk {
	int gpio;
	struct clk c;
};

struct audio_ext_ap_clk2 {
	bool enabled;
	struct pinctrl_info pnctrl_info;
	struct clk c;
};

static struct afe_clk_set clk2_config = {
	Q6AFE_LPASS_CLK_CONFIG_API_VERSION,
	Q6AFE_LPASS_CLK_ID_SPEAKER_I2S_OSR,
	Q6AFE_LPASS_IBIT_CLK_11_P2896_MHZ,
	Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO,
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	0,
};
#if defined(CONFIG_WCD9335_CODEC_MCLK_USE_MSM_GPIO)
struct audio_ext_ap_clk1 {
	bool enabled;
	struct pinctrl_info pinctrl_info;
	u32 mclk_freq;
	struct clk c;
};

struct afe_digital_clk_cfg tasha_digital_cdc_clk;
struct afe_clk_cfg tasha_cdc_clk;
#endif

static inline struct audio_ext_ap_clk *to_audio_ap_clk(struct clk *clk)
{
	return container_of(clk, struct audio_ext_ap_clk, c);
}

static int audio_ext_clk_prepare(struct clk *clk)
{
#if defined(CONFIG_WCD9335_CODEC_MCLK_USE_MSM_GPIO)
	int ret = 0;
	int val = 0;
	void __iomem *vaddr = NULL;

	tasha_digital_cdc_clk.i2s_cfg_minor_version = AFE_API_VERSION_I2S_CONFIG;
	tasha_digital_cdc_clk.clk_val = 9600000;
	tasha_digital_cdc_clk.clk_root = 5;
	tasha_digital_cdc_clk.reserved = 0;

	vaddr = ioremap(LPASS_CSR_GP_IO_MUX_SPKR_CTL, 4);

	val = ioread32(vaddr);
	val = val | 0x00000002;
	iowrite32(val, vaddr);

	vaddr = ioremap(LPASS_CSR_GP_IO_MUX_MIC_CTL, 4);

	val = ioread32(vaddr);
	val = val | 0x00000002;
	iowrite32(val, vaddr);

	ret =  afe_set_digital_codec_core_clock(AFE_PORT_ID_PRIMARY_MI2S_RX, &tasha_digital_cdc_clk);

	pr_err("%s afe_set_digital_codec_core_clock ret %d \n", __func__, ret);

	if (ret < 0) {
		pr_err("%s afe_set_digital_codec_core_clock failed\n",
			__func__);
		goto err;
	}
	return 0;
err:
	return ret;
#else
	struct audio_ext_ap_clk *audio_clk = to_audio_ap_clk(clk);

	pr_debug("%s: gpio: %d\n", __func__, audio_clk->gpio);
	if (gpio_is_valid(audio_clk->gpio))
		return gpio_direction_output(audio_clk->gpio, 1);
	return 0;
}

static void audio_ext_clk_unprepare(struct clk *clk)
{
	struct audio_ext_ap_clk *audio_clk = to_audio_ap_clk(clk);

	pr_debug("%s: gpio: %d\n", __func__, audio_clk->gpio);
	if (gpio_is_valid(audio_clk->gpio))
		gpio_direction_output(audio_clk->gpio, 0);
}

static inline struct audio_ext_ap_clk2 *to_audio_ap_clk2(struct clk *clk)
{
	return container_of(clk, struct audio_ext_ap_clk2, c);
}

static int audio_ext_clk2_prepare(struct clk *clk)
{
	struct audio_ext_ap_clk2 *audio_clk2 = to_audio_ap_clk2(clk);
	struct pinctrl_info *pnctrl_info = &audio_clk2->pnctrl_info;
	int ret;


	if (!pnctrl_info->pinctrl || !pnctrl_info->active)
		return 0;

	ret = pinctrl_select_state(pnctrl_info->pinctrl,
				   pnctrl_info->active);
	if (ret) {
		pr_err("%s: active state select failed with %d\n",
			__func__, ret);
		return -EIO;
	}

	clk2_config.enable = 1;
	ret = afe_set_lpass_clk_cfg(IDX_RSVD_3, &clk2_config);
	if (ret < 0) {
		pr_err("%s: failed to set clock, ret = %d\n", __func__, ret);
		return -EINVAL;
	}

	return 0;
}

static void audio_ext_clk2_unprepare(struct clk *clk)
{
	struct audio_ext_ap_clk2 *audio_clk2 = to_audio_ap_clk2(clk);
	struct pinctrl_info *pnctrl_info = &audio_clk2->pnctrl_info;
	int ret;

	if (!pnctrl_info->pinctrl || !pnctrl_info->sleep)
		return;

	ret = pinctrl_select_state(pnctrl_info->pinctrl,
				   pnctrl_info->sleep);
	if (ret)
		pr_err("%s: sleep state select failed with %d\n",
			__func__, ret);

	clk2_config.enable = 0;
	ret = afe_set_lpass_clk_cfg(IDX_RSVD_3, &clk2_config);
	if (ret < 0)
		pr_err("%s: failed to reset clock, ret = %d\n", __func__, ret);
}

#if defined(CONFIG_WCD9335_CODEC_MCLK_USE_MSM_GPIO)
static inline struct audio_ext_ap_clk1 *to_audio_ap_clk1(struct clk *clk)
{
	return container_of(clk, struct audio_ext_ap_clk1, c);
}

static int audio_ext_clk1_prepare(struct clk *clk)
{
	struct audio_ext_ap_clk1 *audio_clk1 = to_audio_ap_clk1(clk);
	struct pinctrl_info *pinctrl_info = &audio_clk1->pinctrl_info;

	int ret;
	pr_err("debug %s \n", __func__);
	tasha_cdc_clk.i2s_cfg_minor_version = AFE_API_VERSION_I2S_CONFIG;
	tasha_cdc_clk.clk_val1 = Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ;
	tasha_cdc_clk.clk_val2 = Q6AFE_LPASS_OSR_CLK_9_P600_MHZ;
	tasha_cdc_clk.clk_src = Q6AFE_LPASS_CLK_SRC_INTERNAL;
	tasha_cdc_clk.clk_root = 5;
	tasha_cdc_clk.clk_set_mode = Q6AFE_LPASS_MODE_BOTH_VALID;
	tasha_cdc_clk.reserved = 0;

	ret = pinctrl_select_state(pinctrl_info->pinctrl,
		pinctrl_info->active);
	if (ret) {
		pr_err("%s: active state select failed with %d\n",
			__func__, ret);
		ret = -EIO;
		goto err;
	}

	ret =  afe_set_lpass_clock(AFE_PORT_ID_PRIMARY_MI2S_RX, &tasha_cdc_clk);

	if (ret < 0) {
		pr_err("%s afe_set_lpass_clock failed\n",
			__func__);
		goto err;
	}

err:
	return 0;
}

static void audio_ext_clk1_unprepare(struct clk *clk)
{
	struct audio_ext_ap_clk1 *audio_clk1 = to_audio_ap_clk1(clk);
	struct pinctrl_info *pinctrl_info = &audio_clk1->pinctrl_info;

	int ret;

	ret = pinctrl_select_state(pinctrl_info->pinctrl,
		pinctrl_info->sleep);
	if (ret) {
		pr_err("%s: sleep state select failed with %d\n",
			__func__, ret);
	}

	tasha_cdc_clk.clk_val1 = Q6AFE_LPASS_IBIT_CLK_DISABLE;
	ret =  afe_set_lpass_clock(AFE_PORT_ID_PRIMARY_MI2S_RX, &tasha_cdc_clk);

	if (ret < 0) {
		pr_err("%s afe_set_lpass_clock failed\n",
			__func__);
	}

	return;
}
#endif

static struct clk_ops audio_ext_ap_clk_ops = {
	.prepare = audio_ext_clk_prepare,
	.unprepare = audio_ext_clk_unprepare,
};

static struct clk_ops audio_ext_ap_clk2_ops = {
	.prepare = audio_ext_clk2_prepare,
	.unprepare = audio_ext_clk2_unprepare,
};

static struct audio_ext_pmi_clk audio_pmi_clk = {
	.gpio = -EINVAL,
	.c = {
		.dbg_name = "audio_ext_pmi_clk",
		.ops = &clk_ops_dummy,
		CLK_INIT(audio_pmi_clk.c),
	},
};

static struct audio_ext_pmi_clk audio_pmi_lnbb_clk = {
	.gpio = -EINVAL,
	.c = {
		.dbg_name = "audio_ext_pmi_lnbb_clk",
		.ops = &clk_ops_dummy,
		CLK_INIT(audio_pmi_lnbb_clk.c),
	},
};

static struct audio_ext_ap_clk audio_ap_clk = {
	.gpio = -EINVAL,
	.c = {
		.dbg_name = "audio_ext_ap_clk",
		.ops = &audio_ext_ap_clk_ops,
		CLK_INIT(audio_ap_clk.c),
	},
};

static struct audio_ext_ap_clk2 audio_ap_clk2 = {
	.c = {
		.dbg_name = "audio_ext_ap_clk2",
		.ops = &audio_ext_ap_clk2_ops,
		CLK_INIT(audio_ap_clk2.c),
	},
};

static struct clk_lookup audio_ref_clock[] = {
	CLK_LIST(audio_ap_clk),
	CLK_LIST(audio_pmi_clk),
	CLK_LIST(audio_pmi_lnbb_clk),
	CLK_LIST(audio_ap_clk2),
};

static int audio_get_pinctrl(struct platform_device *pdev)
{
	struct pinctrl_info *pnctrl_info;
	struct pinctrl *pinctrl;
	int ret;

	pnctrl_info = &audio_ap_clk2.pnctrl_info;

	if (pnctrl_info->pinctrl) {
		dev_dbg(&pdev->dev, "%s: already requested before\n",
			__func__);
		return -EINVAL;
	}

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		dev_dbg(&pdev->dev, "%s: Unable to get pinctrl handle\n",
			__func__);
		return -EINVAL;
	}
	pnctrl_info->pinctrl = pinctrl;
	/* get all state handles from Device Tree */
	pnctrl_info->sleep = pinctrl_lookup_state(pinctrl, "sleep");
	if (IS_ERR(pnctrl_info->sleep)) {
		dev_err(&pdev->dev, "%s: could not get sleep pinstate\n",
			__func__);
		goto err;
	}
	pnctrl_info->active = pinctrl_lookup_state(pinctrl, "active");
	if (IS_ERR(pnctrl_info->active)) {
		dev_err(&pdev->dev, "%s: could not get active pinstate\n",
			__func__);
		goto err;
	}
	/* Reset the TLMM pins to a default state */
	ret = pinctrl_select_state(pnctrl_info->pinctrl,
				   pnctrl_info->sleep);
	if (ret) {
		dev_err(&pdev->dev, "%s: Disable TLMM pins failed with %d\n",
			__func__, ret);
		goto err;
	}
	return 0;

err:
	devm_pinctrl_put(pnctrl_info->pinctrl);
	return -EINVAL;
}

static int audio_ref_clk_probe(struct platform_device *pdev)
{
	int clk_gpio;
	int ret;
	struct clk *audio_clk;

#if defined(CONFIG_WCD9335_CODEC_MCLK_USE_MSM_GPIO)
	ret = of_property_read_u32(pdev->dev.of_node,
									"qcom,codec-mclk-clk-freq",
									&audio_ap_clk1.mclk_freq);
	if (audio_ap_clk1.mclk_freq == 9600000) {
		ret = audio_get_pinctrl(pdev, AP_CLK1);
		if (ret) {
			pr_err("debug %s: Parsing pinctrl failed\n",
				__func__);
		}
	} else {
#endif
	clk_gpio = of_get_named_gpio(pdev->dev.of_node,
				     "qcom,audio-ref-clk-gpio", 0);
	if (clk_gpio > 0) {
		ret = gpio_request(clk_gpio, "EXT_CLK");
		if (ret) {
			dev_err(&pdev->dev,
				"Request ext clk gpio failed %d, err:%d\n",
				clk_gpio, ret);
			goto err;
		}
		if (of_property_read_bool(pdev->dev.of_node,
					"qcom,node_has_rpm_clock")) {
			audio_clk = clk_get(&pdev->dev, NULL);
			if (IS_ERR(audio_clk)) {
				dev_err(&pdev->dev, "Failed to get RPM div clk\n");
				ret = PTR_ERR(audio_clk);
				goto err_gpio;
			}
			audio_pmi_clk.c.parent = audio_clk;
			audio_pmi_clk.gpio = clk_gpio;
		} else
			audio_ap_clk.gpio = clk_gpio;

	} else {
		if (of_property_read_bool(pdev->dev.of_node,
					"qcom,node_has_rpm_clock")) {
			audio_clk = clk_get(&pdev->dev, NULL);
			if (IS_ERR(audio_clk)) {
				dev_err(&pdev->dev, "Failed to get lnbbclk2\n");
				ret = PTR_ERR(audio_clk);
				goto err;
			}
			audio_pmi_lnbb_clk.c.parent = audio_clk;
			audio_pmi_lnbb_clk.gpio = -EINVAL;
		}
	}

	ret = audio_get_pinctrl(pdev);
	if (ret)
		dev_dbg(&pdev->dev, "%s: Parsing pinctrl failed\n",
			__func__);

	ret = of_msm_clock_register(pdev->dev.of_node, audio_ref_clock,
			      ARRAY_SIZE(audio_ref_clock));
	if (ret) {
		dev_err(&pdev->dev, "%s: audio ref clock register failed\n",
			__func__);
		goto err_gpio;
	}

	return 0;

err_gpio:
	gpio_free(clk_gpio);

err:
	return ret;
}

static int audio_ref_clk_remove(struct platform_device *pdev)
{
	struct pinctrl_info *pnctrl_info = &audio_ap_clk2.pnctrl_info;

	if (audio_pmi_clk.gpio > 0)
		gpio_free(audio_pmi_clk.gpio);
	else if (audio_ap_clk.gpio > 0)
		gpio_free(audio_ap_clk.gpio);

	if (pnctrl_info->pinctrl) {
		devm_pinctrl_put(pnctrl_info->pinctrl);
		pnctrl_info->pinctrl = NULL;
	}

	return 0;
}

static const struct of_device_id audio_ref_clk_match[] = {
	{.compatible = "qcom,audio-ref-clk"},
	{}
};
MODULE_DEVICE_TABLE(of, audio_ref_clk_match);

static struct platform_driver audio_ref_clk_driver = {
	.driver = {
		.name = "audio-ref-clk",
		.owner = THIS_MODULE,
		.of_match_table = audio_ref_clk_match,
	},
	.probe = audio_ref_clk_probe,
	.remove = audio_ref_clk_remove,
};

static int __init audio_ref_clk_platform_init(void)
{
	return platform_driver_register(&audio_ref_clk_driver);
}
module_init(audio_ref_clk_platform_init);

static void __exit audio_ref_clk_platform_exit(void)
{
	platform_driver_unregister(&audio_ref_clk_driver);
}
module_exit(audio_ref_clk_platform_exit);

MODULE_DESCRIPTION("Audio Ref Clock module platform driver");
MODULE_LICENSE("GPL v2");
