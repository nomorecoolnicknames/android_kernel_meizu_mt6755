/*
 *    Copyright (C) 2006 Benjamin Herrenschmidt, IBM Corp.
 *			 <benh@kernel.crashing.org>
 *    and		 Arnd Bergmann, IBM Corp.
 *    Merged from powerpc/kernel/of_platform.c and
 *    sparc{,64}/kernel/of_device.c by Stephen Rothwell
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 */
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/amba/bus.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
/* clk API for the staged audio power-on below */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <dt-bindings/clock/mt6755-clk.h>

const struct of_device_id of_default_bus_match_table[] = {
	{ .compatible = "simple-bus", },
	{ .compatible = "simple-mfd", },
#ifdef CONFIG_ARM_AMBA
	{ .compatible = "arm,amba-bus", },
#endif /* CONFIG_ARM_AMBA */
	{} /* Empty terminated list */
};

static int of_dev_node_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

/**
 * of_find_device_by_node - Find the platform_device associated with a node
 * @np: Pointer to device tree node
 *
 * Returns platform_device pointer, or NULL if not found
 */
struct platform_device *of_find_device_by_node(struct device_node *np)
{
	struct device *dev;

	dev = bus_find_device(&platform_bus_type, NULL, np, of_dev_node_match);
	return dev ? to_platform_device(dev) : NULL;
}
EXPORT_SYMBOL(of_find_device_by_node);

#ifdef CONFIG_OF_ADDRESS
/*
 * The following routines scan a subtree and registers a device for
 * each applicable node.
 *
 * Note: sparc doesn't use these routines because it has a different
 * mechanism for creating devices from device tree nodes.
 */

/**
 * of_device_make_bus_id - Use the device node data to assign a unique name
 * @dev: pointer to device structure that is linked to a device tree node
 *
 * This routine will first try using the translated bus address to
 * derive a unique name. If it cannot, then it will prepend names from
 * parent nodes until a unique name can be derived.
 */
void of_device_make_bus_id(struct device *dev)
{
	struct device_node *node = dev->of_node;
	const __be32 *reg;
	u64 addr;

	/* Construct the name, using parent nodes if necessary to ensure uniqueness */
	while (node->parent) {
		/*
		 * If the address can be translated, then that is as much
		 * uniqueness as we need. Make it the first component and return
		 */
		reg = of_get_property(node, "reg", NULL);
		if (reg && (addr = of_translate_address(node, reg)) != OF_BAD_ADDR) {
			dev_set_name(dev, dev_name(dev) ? "%llx.%s:%s" : "%llx.%s",
				     (unsigned long long)addr, node->name,
				     dev_name(dev));
			return;
		}

		/* format arguments only used if dev_name() resolves to NULL */
		dev_set_name(dev, dev_name(dev) ? "%s:%s" : "%s",
			     strrchr(node->full_name, '/') + 1, dev_name(dev));
		node = node->parent;
	}
}

/**
 * of_device_alloc - Allocate and initialize an of_device
 * @np: device node to assign to device
 * @bus_id: Name to assign to the device.  May be null to use default name.
 * @parent: Parent device.
 */
struct platform_device *of_device_alloc(struct device_node *np,
				  const char *bus_id,
				  struct device *parent)
{
	struct platform_device *dev;
	int rc, i, num_reg = 0, num_irq;
	struct resource *res, temp_res;

	dev = platform_device_alloc("", -1);
	if (!dev)
		return NULL;

	/* count the io and irq resources */
	while (of_address_to_resource(np, num_reg, &temp_res) == 0)
		num_reg++;
	num_irq = of_irq_count(np);

	/* Populate the resource table */
	if (num_irq || num_reg) {
		res = kzalloc(sizeof(*res) * (num_irq + num_reg), GFP_KERNEL);
		if (!res) {
			platform_device_put(dev);
			return NULL;
		}

		dev->num_resources = num_reg + num_irq;
		dev->resource = res;
		for (i = 0; i < num_reg; i++, res++) {
			rc = of_address_to_resource(np, i, res);
			WARN_ON(rc);
		}
		if (of_irq_to_resource_table(np, res, num_irq) != num_irq)
			pr_debug("not all legacy IRQ resources mapped for %s\n",
				 np->name);
	}

	dev->dev.of_node = of_node_get(np);
	dev->dev.parent = parent ? : &platform_bus;

	if (bus_id)
		dev_set_name(&dev->dev, "%s", bus_id);
	else
		of_device_make_bus_id(&dev->dev);

	return dev;
}
EXPORT_SYMBOL(of_device_alloc);

static void of_dma_deconfigure(struct device *dev)
{
	arch_teardown_dma_ops(dev);
}

/**
 * of_platform_device_create_pdata - Alloc, initialize and register an of_device
 * @np: pointer to node to create device for
 * @bus_id: name to assign device
 * @platform_data: pointer to populate platform_data pointer with
 * @parent: Linux device model parent device.
 *
 * Returns pointer to created platform device, or NULL if a device was not
 * registered.  Unavailable devices will not get registered.
 */
static struct platform_device *of_platform_device_create_pdata(
					struct device_node *np,
					const char *bus_id,
					void *platform_data,
					struct device *parent)
{
	struct platform_device *dev;

	if (!of_device_is_available(np) ||
	    of_node_test_and_set_flag(np, OF_POPULATED))
		return NULL;

	/* Sub-step markers inside the device create, so a hang is pinned to
	 * alloc (IRQ map) vs add. */
	{ extern void forge_m681_mark(unsigned char); forge_m681_mark(0xD1); }
	dev = of_device_alloc(np, bus_id, parent);
	if (!dev)
		goto err_clear_flag;
	{ extern void forge_m681_mark(unsigned char); forge_m681_mark(0xD2); }

	dev->dev.bus = &platform_bus_type;
	dev->dev.platform_data = platform_data;
	of_dma_configure(&dev->dev, dev->dev.of_node);
	of_msi_configure(&dev->dev, dev->dev.of_node);

	{ extern void forge_m681_mark(unsigned char); forge_m681_mark(0xD3); }
	if (of_device_add(dev) != 0) {
		of_dma_deconfigure(&dev->dev);
		platform_device_put(dev);
		goto err_clear_flag;
	}
	{ extern void forge_m681_mark(unsigned char); forge_m681_mark(0xD4); }

	return dev;

err_clear_flag:
	of_node_clear_flag(np, OF_POPULATED);
	return NULL;
}

/**
 * of_platform_device_create - Alloc, initialize and register an of_device
 * @np: pointer to node to create device for
 * @bus_id: name to assign device
 * @parent: Linux device model parent device.
 *
 * Returns pointer to created platform device, or NULL if a device was not
 * registered.  Unavailable devices will not get registered.
 */
struct platform_device *of_platform_device_create(struct device_node *np,
					    const char *bus_id,
					    struct device *parent)
{
	return of_platform_device_create_pdata(np, bus_id, NULL, parent);
}
EXPORT_SYMBOL(of_platform_device_create);

#ifdef CONFIG_ARM_AMBA
static struct amba_device *of_amba_device_create(struct device_node *node,
						 const char *bus_id,
						 void *platform_data,
						 struct device *parent)
{
	struct amba_device *dev;
	const void *prop;
	int i, ret;

	pr_debug("Creating amba device %s\n", node->full_name);

	if (!of_device_is_available(node) ||
	    of_node_test_and_set_flag(node, OF_POPULATED))
		return NULL;

	dev = amba_device_alloc(NULL, 0, 0);
	if (!dev) {
		pr_err("%s(): amba_device_alloc() failed for %s\n",
		       __func__, node->full_name);
		goto err_clear_flag;
	}

	/* setup generic device info */
	dev->dev.of_node = of_node_get(node);
	dev->dev.parent = parent ? : &platform_bus;
	dev->dev.platform_data = platform_data;
	if (bus_id)
		dev_set_name(&dev->dev, "%s", bus_id);
	else
		of_device_make_bus_id(&dev->dev);
	of_dma_configure(&dev->dev, dev->dev.of_node);

	/* Allow the HW Peripheral ID to be overridden */
	prop = of_get_property(node, "arm,primecell-periphid", NULL);
	if (prop)
		dev->periphid = of_read_ulong(prop, 1);

	/* Decode the IRQs and address ranges */
	for (i = 0; i < AMBA_NR_IRQS; i++)
		dev->irq[i] = irq_of_parse_and_map(node, i);

	ret = of_address_to_resource(node, 0, &dev->res);
	if (ret) {
		pr_err("%s(): of_address_to_resource() failed (%d) for %s\n",
		       __func__, ret, node->full_name);
		goto err_free;
	}

	ret = amba_device_add(dev, &iomem_resource);
	if (ret) {
		pr_err("%s(): amba_device_add() failed (%d) for %s\n",
		       __func__, ret, node->full_name);
		goto err_free;
	}

	return dev;

err_free:
	amba_device_put(dev);
err_clear_flag:
	of_node_clear_flag(node, OF_POPULATED);
	return NULL;
}
#else /* CONFIG_ARM_AMBA */
static struct amba_device *of_amba_device_create(struct device_node *node,
						 const char *bus_id,
						 void *platform_data,
						 struct device *parent)
{
	return NULL;
}
#endif /* CONFIG_ARM_AMBA */

/**
 * of_devname_lookup() - Given a device node, lookup the preferred Linux name
 */
static const struct of_dev_auxdata *of_dev_lookup(const struct of_dev_auxdata *lookup,
				 struct device_node *np)
{
	struct resource res;

	if (!lookup)
		return NULL;

	for(; lookup->compatible != NULL; lookup++) {
		if (!of_device_is_compatible(np, lookup->compatible))
			continue;
		if (!of_address_to_resource(np, 0, &res))
			if (res.start != lookup->phys_addr)
				continue;
		pr_debug("%s: devname=%s\n", np->full_name, lookup->name);
		return lookup;
	}

	return NULL;
}

/* Codec gate: bit0 (=1) revives the vdec* OF nodes below, bit1 (=2) venc*,
 * bit2 (=4) the vcodec driver probe (consumed in drivers/base/platform.c
 * and in videocodec_kernel_driver.c's DT compatible fallback). core_param,
 * not module_param: this file and drivers/base/platform.c share
 * KBUILD_MODNAME "platform" (two module_params would collide), and
 * core_param gives the unprefixed cmdline token forge_codec_undeny=N.
 * Default 7 (vdec | venc | vcodec): LK truncates the cmdline at ~99
 * characters and the parameter is 0444, so the default is what applies.
 * If reviving the vdec nodes breaks video decode, use 6 (venc | vcodec). */
unsigned int forge_codec_undeny __read_mostly = 7;
core_param(forge_codec_undeny, forge_codec_undeny, uint, 0444);

#if IS_ENABLED(CONFIG_MTK_SND_SOC_NEW_ARCH)
/* [FORGE_AUDIO] staged audio device creation. The audio DT devices can be
 * held back at boot and created later in waves through the sysfs knob
 * mtk_soc_machine.forge_audio_stage: 1 = AFE/pcm platforms (CCF clocks,
 * AFE MMIO, audgpio pinctrl), 2 = codec (first PMIC writes), 3 = machine
 * card. Compiled out in audio-off builds. Nodes are classified by
 * compatible, not by a substring of the name: "mediatek,audiosys" (CCF
 * provider) and "mediatek,audio_sram" (mapped via of_find_compatible_node,
 * no driver) are left alone; accdet is separate.
 * Boot default 3: all audio nodes are populated at boot in DT order. At
 * that point the LK-left state and the Clk_On hold (mtk-soc-afe-control.c
 * forge_audio_hold) keep the audio bus alive; a later runtime population
 * hits the clk_disable_unused audio bus gating (see
 * forge_audio_stage1_power()). The knob still accepts values above the
 * current stage (set the default to 0 for a staged build). */
int forge_audio_stage_current = 3;

static int forge_audio_node_stage(struct device_node *np)
{
	const char *c = NULL;

	if (of_device_is_compatible(np, "mediatek,audio"))
		return 3;
	if (of_device_is_compatible(np, "mediatek,mt_soc_codec_63xx"))
		return 2;
	/* btcvsd is excluded: its reg spans CONN space (0x18000000 PKV +
	 * SRAM_BANK2), unpowered until wmt_loader; BT-SCO is not needed for
	 * audio. */
	if (of_device_is_compatible(np, "mediatek,audio_bt_cvsd"))
		return 0;
	if (!of_property_read_string(np, "compatible", &c) && c &&
	    !strncmp(c, "mediatek,mt_soc_btcvsd", 22))
		return 0;
	if (c && !strncmp(c, "mediatek,mt_soc_", 16))
		return 1;
	return 0;
}

/* Restore the boot-time power/clock state before creating the stage-1
 * devices at runtime. The probe path (dl1 -> InitAfeControl ->
 * AudDrv_Bus_Init -> Afe_Set_Reg AUDIO_TOP_CON0) does raw AFE MMIO and
 * skips the AUD MTCMOS ("don't prepare MTCMOS in probe"), relying on the
 * boot-time state; at boot the probes run before clk_disable_unused /
 * domain cleanup, but at runtime the AUD MTCMOS / INFRA_AUDIO CG are gated
 * and the first AFE access hangs the AXI bus. One enable at a time, each
 * with a pr_emerg breadcrumb (it reaches console-ramoops), so a hang names
 * the failing layer. */
static void forge_audio_stage1_power(void)
{
	static const struct {
		const char *compat;
		int idx;
		const char *tag;
	} pre[] = {
		{ "mediatek,mt6755-scpsys",   SCP_SYS_AUD,
		  "SCP_SYS_AUD mtcmos" },
		{ "mediatek,mt6755-infrasys", INFRA_AUDIO,
		  "INFRA_AUDIO cg" },
		{ "mediatek,mt6755-infrasys", INFRA_AUDIO_26M_BCLK,
		  "INFRA_AUDIO_26M_BCLK cg" },
		{ "mediatek,mt6755-audiosys", AUDIO_AFE,
		  "AUDIO_AFE cg" },
	};
	static bool done;
	struct of_phandle_args a;
	struct clk *clk;
	int i, ret;

	if (done)
		return;
	done = true;
	for (i = 0; i < ARRAY_SIZE(pre); i++) {
		a.np = of_find_compatible_node(NULL, NULL, pre[i].compat);
		if (!a.np) {
			pr_emerg("[FORGE_AUDIO] pre-power: no node %s\n",
				 pre[i].compat);
			continue;
		}
		a.args_count = 1;
		a.args[0] = pre[i].idx;
		clk = of_clk_get_from_provider(&a);
		if (IS_ERR(clk)) {
			pr_emerg("[FORGE_AUDIO] pre-power: %s get=%ld\n",
				 pre[i].tag, PTR_ERR(clk));
			continue;
		}
		pr_emerg("[FORGE_AUDIO] pre-power: enabling %s\n", pre[i].tag);
		ret = clk_prepare_enable(clk);
		pr_emerg("[FORGE_AUDIO] pre-power: %s ret=%d\n",
			 pre[i].tag, ret);
	}
}

static void forge_audio_create_node(struct device_node *np,
				    struct device *parent)
{
	extern void forge_m681_mark_ofnode(const char *name);
	struct platform_device *pd;
	struct device_node *child;

	if (of_node_check_flag(np, OF_POPULATED))
		return;
	/* DRAM survivor marker + console-ramoops line BEFORE the create: on a
	 * hard wedge the last marker/breadcrumb names the dying node. */
	forge_m681_mark_ofnode(np->full_name);
	pr_emerg("[FORGE_AUDIO] creating %s\n", np->full_name);
	pd = of_platform_device_create(np, NULL, parent);
	pr_emerg("[FORGE_AUDIO] stage-create %s -> %s\n", np->full_name,
		 pd ? dev_name(&pd->dev) : "FAILED");
	for_each_child_of_node(np, child)
		forge_audio_create_node(child, pd ? &pd->dev : parent);
}

int forge_audio_stage_populate(int stage)
{
	struct device_node *np;
	struct platform_device *ppd;
	int s;

	if (stage > 3)
		stage = 3;
	if (stage <= forge_audio_stage_current)
		return 0;
	pr_emerg("[FORGE_AUDIO] stage %d -> %d populate enter\n",
		 forge_audio_stage_current, stage);
	if (forge_audio_stage_current < 1)
		forge_audio_stage1_power();
	for_each_of_allnodes(np) {
		s = forge_audio_node_stage(np);
		if (s > forge_audio_stage_current && s <= stage) {
			ppd = np->parent ?
				of_find_device_by_node(np->parent) : NULL;
			forge_audio_create_node(np, ppd ? &ppd->dev : NULL);
		}
	}
	forge_audio_stage_current = stage;
	pr_emerg("[FORGE_AUDIO] stage %d populate DONE\n", stage);
	return 0;
}
#endif /* CONFIG_MTK_SND_SOC_NEW_ARCH */

/**
 * of_platform_bus_create() - Create a device for a node and its children.
 * @bus: device node of the bus to instantiate
 * @matches: match table for bus nodes
 * @lookup: auxdata table for matching id and platform_data with device nodes
 * @parent: parent for new device, or NULL for top level.
 * @strict: require compatible property
 *
 * Creates a platform_device for the provided device_node, and optionally
 * recursively create devices for all the child nodes.
 */
static int of_platform_bus_create(struct device_node *bus,
				  const struct of_device_id *matches,
				  const struct of_dev_auxdata *lookup,
				  struct device *parent, bool strict)
{
	const struct of_dev_auxdata *auxdata;
	struct device_node *child;
	struct platform_device *dev;
	const char *bus_id = NULL;
	void *platform_data = NULL;
	int rc = 0;

	/* Make sure it has a compatible property */
	if (strict && (!of_get_property(bus, "compatible", NULL))) {
		pr_debug("%s() - skipping %s, no compatible prop\n",
			 __func__, bus->full_name);
		return 0;
	}

	auxdata = of_dev_lookup(lookup, bus);
	if (auxdata) {
		bus_id = auxdata->name;
		platform_data = auxdata->platform_data;
	}

	if (of_device_is_compatible(bus, "arm,primecell")) {
		/*
		 * Don't return an error here to keep compatibility with older
		 * device tree files.
		 */
		of_amba_device_create(bus, bus_id, platform_data, parent);
		return 0;
	}

	/* Node-create skip for non-essential peripherals whose blocks sit in
	 * power/clock domains that are not up: a register access during
	 * of_device_alloc/device_add, before any driver probe, hangs the bus
	 * (no fault). Kept: mmc0 (eMMC), uart, i2c, pwrap (PMIC),
	 * pinctrl/gpio, rtc, clk controllers, gic/timer/cpuxgpt, usb (adb).
	 * Each skip marks 0xBC. */
	{ extern void forge_m681_mark_aux(unsigned char, unsigned int);
	  extern int forge_should_skip_display(void);
	  /* always-deny: non-display peripherals that wedge the bus / are unused */
	  static const char * const forge_of_deny[] = {
		/* Not denied: reg-less nodes (touch, fingerprint, sensors) do
		 * no MMIO in of_device_alloc/device_add and cannot stall the
		 * bus; denying them only stops their platform_device creation. */
		"nfc", "irq_nfc",
		"md_ccif", "ccci",
		/* "consys" is not denied: wmt_detect needs consys@18070000, and
		 * consys is not powered until userspace wmt_loader.
		 * "thermal"/"tscpu"/"wmt" match no real node. "fmradio" stays
		 * denied. */
		"thermal", "tscpu", "wmt", "fmradio",
		/* msdc1 (the external microSD host, mediatek,mt6755-mmc at
		 * 0x11240000) is not denied: the MSDC clock domain is always on
		 * (no MTCMOS). msdc2 = SDIO host, denied; msdc3 has no node. */
		"msdc2", "msdc3", "fdvt",
		"jpeg", "vcodec", "venc", "vdec",
		"irtx",
		"mrdump_ext_rst", "eint_wpc", "usb_typec", "swtp",
		"ext_buck_oc", "ext_buck_vmd1", "rt5081_pmu_eint", "rt5081_pd",
		/* rf_clock_buffer/pmic_clock_buffer are not denied: both nodes
		 * are reg-less (no MMIO at device_add), and clk_buf_init() finds
		 * them with of_find_compatible_node in any case. */
		"cpuhvfs", "vcorefs", "eem", "ptp_fsm", "devapc",
		"systracker", "watchpoint", "freqhop", "freqhopping",
		"toprgu",
		NULL };
	  /* the WHOLE display stack -- denied ONLY when the self-healing gate says
	   * to skip (i.e. the previous boot wedged here). */
	  static const char * const forge_display_deny[] = {
		NULL };
	  const char *ofn = bus->full_name ? bus->full_name : "";
	  int dk;
#if IS_ENABLED(CONFIG_MTK_SND_SOC_NEW_ARCH)
	  /* [FORGE_AUDIO] staged audio device creation: audio nodes held
	   * back at boot are created later by forge_audio_stage_populate().
	   * The subtree skip also covers btcvsd children. Marks 0xBA + the
	   * wanted stage. */
	  { int as = forge_audio_node_stage(bus);
	    if (as > 0 && as > forge_audio_stage_current) {
		forge_m681_mark_aux(0xBA, (unsigned int)as);
		return 0;
	    } }
#endif
	  for (dk = 0; forge_of_deny[dk]; dk++) {
		/* Codec gate: bit0 revives the vdec* nodes (vdec_gcon@16000000
		 * + vdec_full_top@16020000), bit1 venc* (venc_gcon@17000000 +
		 * venc@17002000). "jpeg" stays denied; the jpgenc/jpgdec node
		 * names contain neither substring. */
		if ((forge_codec_undeny & 0x1) && !strcmp(forge_of_deny[dk], "vdec"))
			continue;
		if ((forge_codec_undeny & 0x2) && !strcmp(forge_of_deny[dk], "venc"))
			continue;
		if (strstr(ofn, forge_of_deny[dk])) {
			forge_m681_mark_aux(0xBC, (unsigned int)dk);
			return 0;
		}
	  }
	  if (forge_should_skip_display())
		for (dk = 0; forge_display_deny[dk]; dk++)
			if (strstr(ofn, forge_display_deny[dk])) {
				forge_m681_mark_aux(0xBD, (unsigned int)dk);
				return 0;
			}
	}

	/* Name the node about to be created, so a hang inside
	 * of_device_alloc/device_add (no driver probe 0xC0) is located. */
	{ extern void forge_m681_mark_ofnode(const char *name);
	  forge_m681_mark_ofnode(bus->full_name); }
	dev = of_platform_device_create_pdata(bus, bus_id, platform_data, parent);
	if (!dev || !of_match_node(matches, bus))
		return 0;

	for_each_child_of_node(bus, child) {
		pr_debug("   create child: %s\n", child->full_name);
		rc = of_platform_bus_create(child, matches, lookup, &dev->dev, strict);
		if (rc) {
			of_node_put(child);
			break;
		}
	}
	of_node_set_flag(bus, OF_POPULATED_BUS);
	return rc;
}

/**
 * of_platform_bus_probe() - Probe the device-tree for platform buses
 * @root: parent of the first level to probe or NULL for the root of the tree
 * @matches: match table for bus nodes
 * @parent: parent to hook devices from, NULL for toplevel
 *
 * Note that children of the provided root are not instantiated as devices
 * unless the specified root itself matches the bus list and is not NULL.
 */
int of_platform_bus_probe(struct device_node *root,
			  const struct of_device_id *matches,
			  struct device *parent)
{
	struct device_node *child;
	int rc = 0;

	root = root ? of_node_get(root) : of_find_node_by_path("/");
	if (!root)
		return -EINVAL;

	pr_debug("of_platform_bus_probe()\n");
	pr_debug(" starting at: %s\n", root->full_name);

	/* Do a self check of bus type, if there's a match, create children */
	if (of_match_node(matches, root)) {
		rc = of_platform_bus_create(root, matches, NULL, parent, false);
	} else for_each_child_of_node(root, child) {
		if (!of_match_node(matches, child))
			continue;
		rc = of_platform_bus_create(child, matches, NULL, parent, false);
		if (rc) {
			of_node_put(child);
			break;
		}
	}

	of_node_put(root);
	return rc;
}
EXPORT_SYMBOL(of_platform_bus_probe);

/**
 * of_platform_populate() - Populate platform_devices from device tree data
 * @root: parent of the first level to probe or NULL for the root of the tree
 * @matches: match table, NULL to use the default
 * @lookup: auxdata table for matching id and platform_data with device nodes
 * @parent: parent to hook devices from, NULL for toplevel
 *
 * Similar to of_platform_bus_probe(), this function walks the device tree
 * and creates devices from nodes.  It differs in that it follows the modern
 * convention of requiring all device nodes to have a 'compatible' property,
 * and it is suitable for creating devices which are children of the root
 * node (of_platform_bus_probe will only create children of the root which
 * are selected by the @matches argument).
 *
 * New board support should be using this function instead of
 * of_platform_bus_probe().
 *
 * Returns 0 on success, < 0 on failure.
 */
int of_platform_populate(struct device_node *root,
			const struct of_device_id *matches,
			const struct of_dev_auxdata *lookup,
			struct device *parent)
{
	struct device_node *child;
	int rc = 0;

	root = root ? of_node_get(root) : of_find_node_by_path("/");
	if (!root)
		return -EINVAL;

	for_each_child_of_node(root, child) {
		rc = of_platform_bus_create(child, matches, lookup, parent, true);
		if (rc) {
			of_node_put(child);
			break;
		}
	}
	of_node_set_flag(root, OF_POPULATED_BUS);

	of_node_put(root);
	return rc;
}
EXPORT_SYMBOL_GPL(of_platform_populate);

int of_platform_default_populate(struct device_node *root,
				 const struct of_dev_auxdata *lookup,
				 struct device *parent)
{
	return of_platform_populate(root, of_default_bus_match_table, lookup,
				    parent);
}
EXPORT_SYMBOL_GPL(of_platform_default_populate);

static int of_platform_device_destroy(struct device *dev, void *data)
{
	/* Do not touch devices not populated from the device tree */
	if (!dev->of_node || !of_node_check_flag(dev->of_node, OF_POPULATED))
		return 0;

	/* Recurse for any nodes that were treated as busses */
	if (of_node_check_flag(dev->of_node, OF_POPULATED_BUS))
		device_for_each_child(dev, NULL, of_platform_device_destroy);

	if (dev->bus == &platform_bus_type)
		platform_device_unregister(to_platform_device(dev));
#ifdef CONFIG_ARM_AMBA
	else if (dev->bus == &amba_bustype)
		amba_device_unregister(to_amba_device(dev));
#endif

	of_dma_deconfigure(dev);
	of_node_clear_flag(dev->of_node, OF_POPULATED);
	of_node_clear_flag(dev->of_node, OF_POPULATED_BUS);
	return 0;
}

/**
 * of_platform_depopulate() - Remove devices populated from device tree
 * @parent: device which children will be removed
 *
 * Complementary to of_platform_populate(), this function removes children
 * of the given device (and, recurrently, their children) that have been
 * created from their respective device tree nodes (and only those,
 * leaving others - eg. manually created - unharmed).
 *
 * Returns 0 when all children devices have been removed or
 * -EBUSY when some children remained.
 */
void of_platform_depopulate(struct device *parent)
{
	if (parent->of_node && of_node_check_flag(parent->of_node, OF_POPULATED_BUS)) {
		device_for_each_child(parent, NULL, of_platform_device_destroy);
		of_node_clear_flag(parent->of_node, OF_POPULATED_BUS);
	}
}
EXPORT_SYMBOL_GPL(of_platform_depopulate);

#ifdef CONFIG_OF_DYNAMIC
static int of_platform_notify(struct notifier_block *nb,
				unsigned long action, void *arg)
{
	struct of_reconfig_data *rd = arg;
	struct platform_device *pdev_parent, *pdev;
	bool children_left;

	switch (of_reconfig_get_state_change(action, rd)) {
	case OF_RECONFIG_CHANGE_ADD:
		/* verify that the parent is a bus */
		if (!of_node_check_flag(rd->dn->parent, OF_POPULATED_BUS))
			return NOTIFY_OK;	/* not for us */

		/* already populated? (driver using of_populate manually) */
		if (of_node_check_flag(rd->dn, OF_POPULATED))
			return NOTIFY_OK;

		/* pdev_parent may be NULL when no bus platform device */
		pdev_parent = of_find_device_by_node(rd->dn->parent);
		pdev = of_platform_device_create(rd->dn, NULL,
				pdev_parent ? &pdev_parent->dev : NULL);
		of_dev_put(pdev_parent);

		if (pdev == NULL) {
			pr_err("%s: failed to create for '%s'\n",
					__func__, rd->dn->full_name);
			/* of_platform_device_create tosses the error code */
			return notifier_from_errno(-EINVAL);
		}
		break;

	case OF_RECONFIG_CHANGE_REMOVE:

		/* already depopulated? */
		if (!of_node_check_flag(rd->dn, OF_POPULATED))
			return NOTIFY_OK;

		/* find our device by node */
		pdev = of_find_device_by_node(rd->dn);
		if (pdev == NULL)
			return NOTIFY_OK;	/* no? not meant for us */

		/* unregister takes one ref away */
		of_platform_device_destroy(&pdev->dev, &children_left);

		/* and put the reference of the find */
		of_dev_put(pdev);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block platform_of_notifier = {
	.notifier_call = of_platform_notify,
};

void of_platform_register_reconfig_notifier(void)
{
	WARN_ON(of_reconfig_notifier_register(&platform_of_notifier));
}
#endif /* CONFIG_OF_DYNAMIC */

#endif /* CONFIG_OF_ADDRESS */
