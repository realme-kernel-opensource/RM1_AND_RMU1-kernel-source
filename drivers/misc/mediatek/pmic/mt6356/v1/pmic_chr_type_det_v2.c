/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/seq_file.h>
#include <linux/power_supply.h>
#include <linux/time.h>
#include <linux/uaccess.h>

#include <mt-plat/mtk_battery.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mt-plat/mtk_boot.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_boot.h>

#ifdef CONFIG_MTK_USB2JTAG_SUPPORT
#include <mt-plat/mtk_usb2jtag.h>
#endif

/* #define __FORCE_USB_TYPE__ */
#define __SW_CHRDET_IN_PROBE_PHASE__

static CHARGER_TYPE g_chr_type;
#ifdef __SW_CHRDET_IN_PROBE_PHASE__
static struct work_struct chr_work;
#endif

static DEFINE_MUTEX(chrdet_lock);
#ifdef VENDOR_EDIT
//PengNan@BSP.CHG.Basic, 2017/09/19, add for charger type detection.
static DEFINE_MUTEX(charger_type_mutex);
#endif /*VENDOR_EDIT*/


#if !defined(CONFIG_FPGA_EARLY_PORTING)
static bool first_connect = true;
#endif
#ifdef VENDOR_EDIT
CHARGER_TYPE MTK_CHR_Type_num;

#endif
static struct power_supply *chrdet_psy;
static int chrdet_inform_psy_changed(CHARGER_TYPE chg_type, bool chg_online)
{
	int ret = 0;
	union power_supply_propval propval;

	pr_err("charger type: %s: online = %d, type = %d\n", __func__, chg_online, chg_type);

	/* Inform chg det power supply */
	if (chg_online) {
		propval.intval = chg_online;
		ret = power_supply_set_property(chrdet_psy, POWER_SUPPLY_PROP_ONLINE, &propval);
		if (ret < 0)
			pr_err("%s: psy online failed, ret = %d\n", __func__, ret);

		propval.intval = chg_type;
		ret = power_supply_set_property(chrdet_psy, POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
		if (ret < 0)
			pr_err("%s: psy type failed, ret = %d\n", __func__, ret);

		return ret;
	}

	propval.intval = chg_type;
	ret = power_supply_set_property(chrdet_psy, POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0)
		pr_err("%s: psy type failed, ret(%d)\n", __func__, ret);

	propval.intval = chg_online;
	ret = power_supply_set_property(chrdet_psy, POWER_SUPPLY_PROP_ONLINE,
		&propval);
	if (ret < 0)
		pr_err("%s: psy online failed, ret(%d)\n", __func__, ret);
	return ret;
}

#if defined(CONFIG_FPGA_EARLY_PORTING)
/*****************************************************************************
 * FPGA
 ******************************************************************************/
int hw_charging_get_charger_type(void)
{
	g_chr_type = STANDARD_HOST;
	chrdet_inform_psy_changed(g_chr_type, 1);
	return g_chr_type;
}

#else
/*****************************************************************************
 * EVB / Phone
 ******************************************************************************/
static void hw_bc11_init(void)
{
	int timeout = 20;

	msleep(200);

	if (first_connect == true) {
		/* add make sure USB Ready */
		if (is_usb_rdy() == false) {
			pr_err("CDP, block\n");
			while (is_usb_rdy() == false && timeout > 0) {
				msleep(100);
				timeout--;
			}
			if (timeout == 0)
				pr_err("CDP, timeout\n");
			else
				pr_err("CDP, free\n");
		} else
			pr_err("CDP, PASS\n");
		first_connect = false;
	}

	/* RG_bc11_BIAS_EN=1 */
	bc11_set_register_value(PMIC_RG_BC11_BIAS_EN, 1);
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0);
	/* bc11_RST=1 */
	bc11_set_register_value(PMIC_RG_BC11_RST, 1);
	/* bc11_BB_CTRL=1 */
	bc11_set_register_value(PMIC_RG_BC11_BB_CTRL, 1);
	/* add pull down to prevent PMIC leakage */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	msleep(50);

	Charger_Detect_Init();
}

static unsigned int hw_bc11_DCD(void)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_IPU_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x2);
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=01 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x1);
	/* RG_bc11_CMP_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x2);
#ifndef VENDOR_EDIT
//PengNan@BSP.CHG.Basic, 2017/09/28, add for sdp/non-charger detection.
	msleep(80);
#else
	msleep(40);
#endif /*VENDOR_EDIT*/
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	return wChargerAvail;
}

static unsigned int hw_bc11_stepA1(void)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);
	msleep(80);
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	return wChargerAvail;
}

static unsigned int hw_bc11_stepA2(void)
{
	unsigned int wChargerAvail = 0;
	/* RG_bc11_VSRC_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
	/* RG_bc11_IPD_EN[1:0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);
#ifndef VENDOR_EDIT
	msleep(80);
#else
	msleep(120);
#endif
	/* mdelay(80); */
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	return wChargerAvail;
}

static unsigned int hw_bc11_stepB2(void)
{
	unsigned int wChargerAvail = 0;

	/*enable the voltage source to DM*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x1);
	/* enable the pull-down current to DP */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x2);
	/* VREF threshold voltage for comparator  =0.325V */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* enable the comparator to DP */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x2);
	msleep(80);
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
	/*reset to default value*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	if (wChargerAvail == 1) {
		bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
		pr_err("charger type: DCP, keep DM voltage source in stepB2\n");
	}
	return wChargerAvail;

}
#ifdef VENDOR_EDIT
//PengNan@BSP.CHG.Basic, 2017/09/18, 2017/09/18, add stepC in bc1.2 for some third adapter.
static unsigned int hw_bc11_stepC(void)
{
	unsigned int wChargerAvail = 0;

	/* RG_bc11_IPU_EN[1:0]=10 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x2);
	/* RG_bc11_VREF_VTH = [1:0]=01 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x1);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);
	msleep(80);
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
	
	#ifdef VENDOR_EDIT
	//PengNan@BSP.CHG.Basic, 2017/08/25, add for chager type detection dump
	pr_err("%s:Reg[0x%x]=0x%x \n",__func__,MT6356_CHR_CON8, upmu_get_reg_value(MT6356_CHR_CON8));
	#endif /*VENDOR_EDIT*/
	
	/*reset to default value*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	if (wChargerAvail == 1) {
		bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
		pr_err("charger type: DCP, keep DM voltage source in stepB2\n");
	}

	return wChargerAvail;
}
#endif /*VENDOR_EDIT*/
static void hw_bc11_done(void)
{
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	/* RG_bc11_VREF_VTH = [1:0]=0 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_BIAS_EN=0 */
	bc11_set_register_value(PMIC_RG_BC11_BIAS_EN, 0x0);
	Charger_Detect_Release();
}

static void dump_charger_name(CHARGER_TYPE type)
{
	switch (type) {
	case CHARGER_UNKNOWN:
		pr_err("charger type: %d, CHARGER_UNKNOWN\n", type);
		break;
	case STANDARD_HOST:
		pr_err("charger type: %d, Standard USB Host\n", type);
		break;
	case CHARGING_HOST:
		pr_err("charger type: %d, Charging USB Host\n", type);
		break;
	case NONSTANDARD_CHARGER:
		pr_err("charger type: %d, Non-standard Charger\n", type);
		break;
	case STANDARD_CHARGER:
		pr_err("charger type: %d, Standard Charger\n", type);
		break;
	case APPLE_2_1A_CHARGER:
		pr_err("charger type: %d, APPLE_2_1A_CHARGER\n", type);
		break;
	case APPLE_1_0A_CHARGER:
		pr_err("charger type: %d, APPLE_1_0A_CHARGER\n", type);
		break;
	case APPLE_0_5A_CHARGER:
		pr_err("charger type: %d, APPLE_0_5A_CHARGER\n", type);
		break;
	default:
		pr_err("charger type: %d, Not Defined!!!\n", type);
		break;
	}
}

int hw_charging_get_charger_type(void)
{
	CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;

#ifdef CONFIG_MTK_USB2JTAG_SUPPORT
	if (usb2jtag_mode()) {
		pr_err("[USB2JTAG] in usb2jtag mode, skip charger detection\n");
		return STANDARD_HOST;
	}
#endif

	hw_bc11_init();

	if (hw_bc11_DCD()) {
		if (hw_bc11_stepA1())
			CHR_Type_num = APPLE_2_1A_CHARGER;
		else
			CHR_Type_num = NONSTANDARD_CHARGER;
	} else {
		if (hw_bc11_stepA2()) {
			if (hw_bc11_stepB2())
				CHR_Type_num = STANDARD_CHARGER;
#ifdef VENDOR_EDIT
//PengNan@BSP.CHG.Basic, 2017/09/18, 2017/09/18, add stepC in bc1.2 for some third adapter.
			else {
				if (hw_bc11_stepC())
					CHR_Type_num = STANDARD_CHARGER;
				else
					CHR_Type_num = CHARGING_HOST;
			}
#else
			else
				CHR_Type_num = CHARGING_HOST;
#endif /*VENDOR_EDIT*/
		} else
			CHR_Type_num = STANDARD_HOST;
	}

	if (CHR_Type_num != STANDARD_CHARGER)
		hw_bc11_done();
	else
		pr_err("charger type: skip bc11 release for BC12 DCP SPEC\n");

	dump_charger_name(CHR_Type_num);


#ifdef __FORCE_USB_TYPE__
	CHR_Type_num = STANDARD_HOST;
	pr_err("charger type: Froce to STANDARD_HOST\n");
#endif

	return CHR_Type_num;
}


#ifdef VENDOR_EDIT
//PengNan@BSP.CHG.Basic, 2017/09/19, add for charger type detection.
extern int oppo_chg_get_chg_type(void);
int mt_charger_type_detection(void)
{
//	static CHARGER_TYPE CHR_Type_Num = CHARGER_UNKNOWN;
	//chg_debug( "mt_charger_type_detection, CHR_Type_num = %d\r\n",CHR_Type_num);

	mutex_lock(&charger_type_mutex);
	if(!pmic_get_register_value(PMIC_RGS_CHRDET))
	{
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
	}
	else
	{
		if(MTK_CHR_Type_num == CHARGER_UNKNOWN)
		{
			MTK_CHR_Type_num = hw_charging_get_charger_type();

			//add secondary detection for non-standard charger
		   if(MTK_CHR_Type_num == NONSTANDARD_CHARGER){
			   	 MTK_CHR_Type_num = hw_charging_get_charger_type();
				 pr_err("secondary detection for non-standard charger!\n");
		   	}

		}
	}

	mutex_unlock(&charger_type_mutex);
	pr_err( "%s, CHR_Type_num = %d\r\n",__func__,MTK_CHR_Type_num);

	return MTK_CHR_Type_num;
}
#endif /*VENDOR_EDIT*/

/*****************************************************************************
 * Charger Detection
 ******************************************************************************/
#ifdef VENDOR_EDIT
/* Qiao.Hu@@Prd6.BaseDrv.USB.Basic, 2017/07/24, Add for charger */
extern bool oppo_chg_wake_update_work(void);
enum {
	ADAPTER_FW_UPDATE_NONE,
	ADAPTER_FW_NEED_UPDATE,
	ADAPTER_FW_UPDATE_SUCCESS,
	ADAPTER_FW_UPDATE_FAIL,
};

#endif /* VENDOR_EDIT */
void do_charger_detect(void)
{
	if (!mt_usb_is_device()) {
		g_chr_type = CHARGER_UNKNOWN;
		pr_err("charger type: UNKNOWN, Now is usb host mode. Skip detection!!!\n");
		return;
	}

	if (is_meta_mode()) {
		/* Skip charger type detection to speed up meta boot.*/
		pr_notice("charger type: force Standard USB Host in meta\n");
		g_chr_type = STANDARD_HOST;
		chrdet_inform_psy_changed(g_chr_type, 1);
		return;
	}
	mutex_lock(&chrdet_lock);

	if (pmic_get_register_value(PMIC_RGS_CHRDET)) {
		pr_err("charger type: charger IN\n");
		#ifdef VENDOR_EDIT
		/* Qiao.Hu@@Prd6.BaseDrv.USB.Basic, 2017/07/24, Add for charger */
	//	oppo_chg_wake_update_work();
		#endif /* VENDOR_EDIT */
		#ifndef VENDOR_EDIT
		//PengNan@BSP.CHG.Basic, 2017/09/19, add for charger type detection.
		g_chr_type = hw_charging_get_charger_type();
		#else
		g_chr_type = mt_charger_type_detection();
		#endif /*VENDOR_EDIT*/
		chrdet_inform_psy_changed(g_chr_type, 1);
		oppo_chg_wake_update_work();
	} else {
		pr_err("charger type: charger OUT\n");
		g_chr_type = CHARGER_UNKNOWN;
		#ifdef VENDOR_EDIT
		/* Qiao.Hu@@Prd6.BaseDrv.USB.Basic, 2017/07/24, Add for charger */
		MTK_CHR_Type_num = mt_charger_type_detection();
		oppo_chg_wake_update_work();
		#endif /* VENDOR_EDIT */
		chrdet_inform_psy_changed(g_chr_type, 0);
	}

	mutex_unlock(&chrdet_lock);
}



/*****************************************************************************
 * PMIC Int Handler
 ******************************************************************************/
void chrdet_int_handler(void)
{
	/*
	 * pr_err("[chrdet_int_handler]CHRDET status = %d....\n",
	 *	pmic_get_register_value(PMIC_RGS_CHRDET));
	 */
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	if (!pmic_get_register_value(PMIC_RGS_CHRDET)) {
		int boot_mode = 0;

		boot_mode = get_boot_mode();

		if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
		    || boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
			pr_err("[chrdet_int_handler] Unplug Charger/USB\n");
#ifndef CONFIG_TCPC_CLASS
            mt_power_off();		
#else
			return;
#endif
		}
	}
#endif
	do_charger_detect();
}


/************************************************/
/* Charger Probe Related
*************************************************/
#ifdef __SW_CHRDET_IN_PROBE_PHASE__
static void do_charger_detection_work(struct work_struct *data)
{
	if (pmic_get_register_value(PMIC_RGS_CHRDET))
		do_charger_detect();
}
#endif


static int __init pmic_chrdet_init(void)
{
	mutex_init(&chrdet_lock);
	chrdet_psy = power_supply_get_by_name("charger");
	if (!chrdet_psy) {
		pr_err("%s: get power supply failed\n", __func__);
		return -EINVAL;
	}

#ifdef __SW_CHRDET_IN_PROBE_PHASE__
	/* do charger detect here to prevent HW miss interrupt*/
	INIT_WORK(&chr_work, do_charger_detection_work);
	schedule_work(&chr_work);
#endif

	pmic_register_interrupt_callback(INT_CHRDET_EDGE, chrdet_int_handler);
	pmic_enable_interrupt(INT_CHRDET_EDGE, 1, "PMIC");

	return 0;
}

late_initcall(pmic_chrdet_init);

#endif
