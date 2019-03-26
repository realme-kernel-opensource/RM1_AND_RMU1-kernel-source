/************************************************************************************
 ** File: - SDM660.LA.1.0\android\kernel\msm-4.4\drivers\soc\oppo\oppo_fp_common\oppo_fp_common.c
 ** VENDOR_EDIT
 ** Copyright (C), 2008-2017, OPPO Mobile Comm Corp., Ltd
 **
 ** Description:
 **      fp_common compatibility configuration
 **
 ** Version: 1.0
 ** Date created: 18:03:11,11/02/2017
 ** Author: Ziqing.guo@Prd.BaseDrv
 **
 ** --------------------------- Revision History: --------------------------------
 **  <author>         <data>         <desc>
 **  Ziqing.guo       2017/02/11     create the file
 **  Ziqing.guo       2017/02/13     add FPC1260, update the mapping table of fingerprint ID
 **  Ziqing.guo       2017/02/20     add 16118 for the mapping table
 **  Ziqing.guo       2017/03/09     change the name of /proc/fp_id (for example, ofilm_a -> a_ofilm)
 **  Hongdao.yu       2017/03/09     add 16037 15103 for the mapping table
 **  Hongdao.yu       2017/03/14     add macro to control 15103 not config fp_id3
 **  Hongdao.yu       2017/03/22     remake ID match code
 **  Hongdao.yu       2017/03/30     add 17001 16017(N) config and delete 16037 15103 config
 **  Ziqing.guo       2017/04/29     add for displaying secure boot switch
 **  Hongdao.yu       2017/05/26     add 16027 config and fix compile err in new proj of no using fp_get_matched_chip_module
 **  Ziqing.guo       2017/06/06     add to remove proc fs in case of probing fail
 **  Ziqing.guo       2017/06/08     add for 17011,17021
 **  Ziqing.guo       2017/06/09     add the proc fs secureSNBound
 **  Ziqing.guo       2017/07/12     add for fpc 1022,1270 (17011)
 **  Hongdao.yu       2017/07/13     add engineermode menu config of snr,inclination test
 **  Ziqing.guo       2017/07/18     update ID mapping table for 17011/17021
 **  Ran.Chen         2017/07/18     add fpc 1022,1023 for MTK6763
 **  Ran.Chen         2017/08/09     add msleep(20) in MTK6763 before read fp_id_gpio
 **  Ziqing.guo       2017/08/11     add GOODIX 3268 for 17011/17021
 **  Ran.Chen         2017/08/15     add GOODIX 3268 for MTK6763
 **  Hongdao.yu       2017/08/15     add 8976pro secure boot showing
 **  Ping.Liu         2017/08/18     add for 17071,17371 
 **  Ziqing.guo       2017/08/31     add a_goodix(3268), b_goodix(5288) for 17011
 **  Bin.Li           2017/09/06     add for 6763 secureboot
 **  Ran.Chen         2017/10/09     add for 17101
 **  Ran.Chen         2017/10/09     add fp_id_retry  for MTK6763
 **  Ziqing.guo       2017/11/13     add for 17081
 **  Ziqing.guo       2017/11/16     add for 17091
 **  Ziqing.guo       2017/11/30     add 17011 (1270) for android O
 **  Ziqing.guo       2017/12/08     add for 17085
 **  Bin.Li           2017/12/12     remove secure part
 **  Bin.Li           2017/12/27     add for 17197
 **  Ran.Chen         2018/01/29     modify for fp_id, Code refactoring
 **  Ziqing.guo       2018/03/07     add for 18001
 **  Ziqing.guo       2018/03/13     fix the problem for coverity CID 14566
 **  Hongdao.yu       2018/03/13     add for gf_5298(5228+158)
 **  Hongdao.yu       2018/03/26     add for gf_5298(5228+158) glass
 **  Ping.Liu         2018/05/17     add for 18311
 **  Ran.Chen         2018/05/26     add for 18316
 **  Ping.Liu         2018/06/01     modify for 1023_glass(1023+2060)
 **  Ran.Chen         2018/06/06     add fpc1023+2060 (vdd_io 3v) for 18316
 **  Ziqing.guo       2018/06/28     add fpc1023+2060 (vdd_io 3v) for 18321
 **  Ziqing.guo       2018/07/09     add fpc1023+2060 (vdd_io 3v) for 18325(18326)
 **  Ziqing.guo       2018/07/11     add for silead(18325, 18005)
 **  Ping.Liu         2018/07/14     add for 18011
 **  Shupeng.Zhou     2018/08/21     add for fpc1028
 **  Shupeng.Zhou     2018/09/09     add for fpc1511
 ************************************************************************************/

#include <linux/module.h>
#include <linux/proc_fs.h>
#if CONFIG_OPPO_FINGERPRINT_PLATFORM == 6763 || CONFIG_OPPO_FINGERPRINT_PLATFORM == 6771
#include <sec_boot_lib.h>
#else
#include <soc/qcom/smem.h>
#endif
#include <soc/oppo/oppo_project.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/string.h>
#include "../include/oppo_fp_common.h"

#define CHIP_PRIMAX     "primax"
#define CHIP_CT         "CT"
#define CHIP_OFILM      "ofilm"
#define CHIP_QTECH      "Qtech"
#define CHIP_TRULY      "truly"

#define CHIP_GOODIX     "G"
#define CHIP_FPC        "F"
#define CHIP_SILEAD     "S"
#define CHIP_UNKNOWN    "unknown"

#define ENGINEER_MENU_FPC1140  "-1,-1"  /* content in " " represents SNR,inclination test item in order in engineer menu, and -1/1 means off/on */
#define ENGINEER_MENU_FPC1022  "-1,-1"
#define ENGINEER_MENU_FPC1023  "1,-1"
#define ENGINEER_MENU_FPC1260  "1,-1"
#define ENGINEER_MENU_FPC1270  "-1,-1"
#define ENGINEER_MENU_GOODIX   "1,1"
#define ENGINEER_MENU_GOODIX_3268   "-1,-1"
#define ENGINEER_MENU_GOODIX_5288   "-1,-1"
#define ENGINEER_MENU_GOODIX_5228   "-1,-1"
#define ENGINEER_MENU_GOODIX_OPTICAL   "-1,-1"
#define ENGINEER_MENU_SILEAD_OPTICAL   "-1,-1"
#define ENGINEER_MENU_GOODIX_5298   "-1,-1"
#define ENGINEER_MENU_DEFAULT  "-1,-1"

static struct proc_dir_entry *fp_id_dir = NULL;
static char *fp_id_name = "fp_id";
static struct proc_dir_entry *oppo_fp_common_dir = NULL;
static char *oppo_fp_common_dir_name = "oppo_fp_common";
static char fp_manu[FP_ID_MAX_LENGTH] = CHIP_UNKNOWN; /* the length of this string should be less than FP_ID_MAX_LENGTH */
static struct fp_data *fp_data_ptr = NULL;
char g_engineermode_menu_config[ENGINEER_MENU_SELECT_MAXLENTH] = ENGINEER_MENU_DEFAULT;

#if CONFIG_OPPO_FINGERPRINT_PROJCT == 17011 || CONFIG_OPPO_FINGERPRINT_PROJCT == 17015 || CONFIG_OPPO_FINGERPRINT_PROJCT == 17081 || CONFIG_OPPO_FINGERPRINT_PROJCT == 17085
fp_module_config_t fp_module_config_list[] = {
    {{0, 0, 0},  FP_FPC_1270, CHIP_FPC, ENGINEER_MENU_FPC1270},
    {{0, 1, 0},  FP_FPC_1023, CHIP_FPC, ENGINEER_MENU_FPC1023},
    {{0, 1, 1},  FP_FPC_1022, CHIP_FPC, ENGINEER_MENU_FPC1022},
    {{1, 0, 0},  FP_FPC_1023, CHIP_FPC, ENGINEER_MENU_FPC1023},
    {{1, 0, 1},  FP_GOODIX_3268, CHIP_GOODIX, ENGINEER_MENU_GOODIX_3268},
    {{1, 1, 0},  FP_GOODIX_5288, CHIP_GOODIX, ENGINEER_MENU_GOODIX_5288},
    {{1, 1, 1},  FP_FPC_1022, CHIP_FPC, ENGINEER_MENU_FPC1022},
};
#elif CONFIG_OPPO_FINGERPRINT_PROJCT == 18316
fp_module_config_t fp_module_config_list[] = {
    {{1, 0, 1},  FP_GOODIX_5298_GLASS, CHIP_GOODIX, ENGINEER_MENU_GOODIX_5298},
    {{0, 0, 1},  FP_FPC_1023_GLASS, CHIP_FPC, ENGINEER_MENU_FPC1023},
    {{1, 0, 0},  FP_GOODIX_5298, CHIP_GOODIX, ENGINEER_MENU_GOODIX_5298},
};
#elif CONFIG_OPPO_FINGERPRINT_PROJCT == 18321 || CONFIG_OPPO_FINGERPRINT_PROJCT == 18326
fp_module_config_t fp_module_config_list[] = {
    {{0, 0, 1},  FP_FPC_1023_GLASS, CHIP_FPC, ENGINEER_MENU_FPC1023},
};
#elif CONFIG_OPPO_FINGERPRINT_PROJCT == 18005 || CONFIG_OPPO_FINGERPRINT_PROJCT == 18323
fp_module_config_t fp_module_config_list[] = {
    {{1, 1, 0},  FP_SILEAD_OPTICAL_70,  CHIP_SILEAD,  ENGINEER_MENU_SILEAD_OPTICAL},
};
#elif CONFIG_OPPO_FINGERPRINT_PLATFORM == 6763
fp_module_config_t fp_module_config_list[] = {
    {{1, 0, 0},  FP_FPC_1023, CHIP_FPC, ENGINEER_MENU_FPC1023},
    {{0, 1, 0},  FP_FPC_1023, CHIP_FPC, ENGINEER_MENU_FPC1023},
    {{0, 1, 1},  FP_FPC_1022, CHIP_FPC, ENGINEER_MENU_FPC1022},
    {{0, 0, 1},  FP_GOODIX_3268, CHIP_GOODIX, ENGINEER_MENU_GOODIX_3268},
};
#elif CONFIG_OPPO_FINGERPRINT_PLATFORM == 6771
fp_module_config_t fp_module_config_list[] = {
    {{1, 0, 1},  FP_GOODIX_5298_GLASS, CHIP_GOODIX, ENGINEER_MENU_GOODIX_5298},
    {{1, 0, 0},  FP_GOODIX_5298, CHIP_GOODIX, ENGINEER_MENU_GOODIX_5298},
    {{0, 1, 0},  FP_FPC_1023, CHIP_FPC, ENGINEER_MENU_FPC1023},
    {{0, 1, 1},  FP_FPC_1022, CHIP_FPC, ENGINEER_MENU_FPC1022},
    {{0, 0, 1},  FP_FPC_1023_GLASS, CHIP_FPC, ENGINEER_MENU_FPC1023},
    {{1, -1, -1},  FP_FPC_1028_COATING, CHIP_FPC, ENGINEER_MENU_DEFAULT},
    {{0, -1, -1},  FP_FPC_1511_COATING, CHIP_FPC, ENGINEER_MENU_DEFAULT},
};
#elif CONFIG_OPPO_FINGERPRINT_PROJCT == 18171 || CONFIG_OPPO_FINGERPRINT_PROJCT == 18571 || CONFIG_OPPO_FINGERPRINT_PROJCT == 18573
fp_module_config_t fp_module_config_list[] = {
    {{0, -1, -1},  FP_FPC_1023_GLASS, CHIP_FPC, ENGINEER_MENU_FPC1023},
    {{1, -1, -1},  FP_FPC_1511_COATING, CHIP_FPC, ENGINEER_MENU_DEFAULT},
};
#else
fp_module_config_t fp_module_config_list[] = {
    {{1, 1, 1},  FP_FPC_1140, CHIP_OFILM,  ENGINEER_MENU_FPC1140},
    {{0, 0, 1},  FP_FPC_1140, CHIP_PRIMAX, ENGINEER_MENU_FPC1140},
    {{0, 1, 0},  FP_FPC_1140, CHIP_TRULY,  ENGINEER_MENU_FPC1140},
#if CONFIG_OPPO_FINGERPRINT_PROJCT == 16037
    {{1, 0, 1},  FP_FPC_1140, CHIP_QTECH,  ENGINEER_MENU_FPC1140},
#else
    {{1, 0, 1},  FP_FPC_1260, CHIP_OFILM,  ENGINEER_MENU_FPC1260},
#endif
    {{0, 0, 0},  FP_FPC_1260, CHIP_PRIMAX, ENGINEER_MENU_FPC1260},
    {{1, 1, 0},  FP_FPC_1260, CHIP_TRULY,  ENGINEER_MENU_FPC1260},
    {{1, 0, 0},  FP_FPC_1260, CHIP_QTECH,  ENGINEER_MENU_FPC1260},
    {{0, 0, -1}, FP_FPC_1140, CHIP_PRIMAX  ENGINEER_MENU_FPC1140},
    {{0, 1, -1}, FP_FPC_1140, CHIP_TRULY,  ENGINEER_MENU_FPC1140},
    {{1, 0, -1}, FP_FPC_1140, CHIP_QTECH,  ENGINEER_MENU_FPC1140},
    {{1, 1, -1}, FP_FPC_1140, CHIP_OFILM,  ENGINEER_MENU_FPC1140},
};
#endif

static int fp_request_named_gpio(struct fp_data *fp_data,
        const char *label, int *gpio)
{
    struct device *dev = fp_data->dev;
    struct device_node *np = dev->of_node;

    int ret = of_get_named_gpio(np, label, 0);
    if (ret < 0) {
        dev_err(dev, "failed to get '%s'\n", label);
        return FP_ERROR_GPIO;
    }

    *gpio = ret;
    ret = devm_gpio_request(dev, *gpio, label);
    if (ret) {
        dev_err(dev, "failed to request gpio %d\n", *gpio);
        devm_gpio_free(dev, *gpio);
        return FP_ERROR_GPIO;
    }

    dev_info(dev, "%s - gpio: %d\n", label, *gpio);
    return FP_OK;
}

static int fp_gpio_parse_dts(struct fp_data *fp_data)
{
    int ret =-1;

    if (!fp_data) {
        return FP_ERROR_GENERAL;
    }

#if CONFIG_OPPO_FINGERPRINT_PROJCT == 18171 || CONFIG_OPPO_FINGERPRINT_PROJCT == 18571 || CONFIG_OPPO_FINGERPRINT_PROJCT == 18573
    ret = fp_request_named_gpio(fp_data, "oppo,fp-id1",
            &fp_data->gpio_id1);
    if (ret) {
        return FP_ERROR_GPIO;
    }
#elif CONFIG_OPPO_FINGERPRINT_PROJCT != 18005 && CONFIG_OPPO_FINGERPRINT_PROJCT != 18323
    ret = fp_request_named_gpio(fp_data, "oppo,fp-id1",
            &fp_data->gpio_id1);
    if (ret) {
        return FP_ERROR_GPIO;
    }

    ret = fp_request_named_gpio(fp_data, "oppo,fp-id2",
            &fp_data->gpio_id2);
    if (ret) {
        return FP_ERROR_GPIO;
    }
#endif

    switch (get_project()) {
#if CONFIG_OPPO_FINGERPRINT_PROJCT == 17001 || CONFIG_OPPO_FINGERPRINT_PROJCT == 16017
        case OPPO_17001:
        case OPPO_16027:
            {
                ret = fp_request_named_gpio(fp_data, "oppo,fp-id0",
                        &fp_data->gpio_id0);
                if (ret) {
                    return FP_ERROR_GPIO;
                }

                ret = fp_request_named_gpio(fp_data, "oppo,fp-id3",
                        &fp_data->gpio_id3);
                if (ret) {
                    return FP_ERROR_GPIO;
                }
                break;
            }
        case OPPO_16017:
            {
                ret = fp_request_named_gpio(fp_data, "oppo,fp-id0",
                        &fp_data->gpio_id0);
                if (ret) {
                    return FP_ERROR_GPIO;
                }
                break;
            }
#elif CONFIG_OPPO_FINGERPRINT_PLATFORM == 6763
        case OPPO_17031:
        case OPPO_17051:
        case OPPO_17071:
        case OPPO_17371:
        case OPPO_17321:
        case OPPO_17101:
            {
                ret = fp_request_named_gpio(fp_data, "oppo,fp-id0",
                        &fp_data->gpio_id0);
                if (ret) {
                    return FP_ERROR_GPIO;
                }
                break;
            }
#elif CONFIG_OPPO_FINGERPRINT_PLATFORM == 6771
        case OPPO_17331:
        case OPPO_17197:
        case OPPO_18311:
        case OPPO_18011:
        case OPPO_18611:
            {
                ret = fp_request_named_gpio(fp_data, "oppo,fp-id0",
                        &fp_data->gpio_id0);
                if (ret) {
                    return FP_ERROR_GPIO;
                }
                break;
            }
#endif
#if CONFIG_OPPO_FINGERPRINT_PROJCT == 16037
        case OPPO_16037:
            ret = fp_request_named_gpio(fp_data, "oppo,fp-id3",
                    &fp_data->gpio_id0);
            if (ret) {
                return FP_ERROR_GPIO;
            }
            break;
#endif
#if CONFIG_OPPO_FINGERPRINT_PROJCT == 15103
        case OPPO_15103:
            break;
#endif
#if CONFIG_OPPO_FINGERPRINT_PROJCT == 18171 || CONFIG_OPPO_FINGERPRINT_PROJCT == 18571 || CONFIG_OPPO_FINGERPRINT_PROJCT == 18573
        case OPPO_18171:
        case OPPO_18172:
        case OPPO_18571:
            break;
#endif
        default:
            {
                ret = fp_request_named_gpio(fp_data, "oppo,fp-id3",
                        &fp_data->gpio_id3);
                if (ret) {
                    return FP_ERROR_GPIO;
                }
                break;
            }
    }

    return FP_OK;
}

static ssize_t fp_id_node_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char page[10];
    char *p = page;
    int len = 0;

    p += sprintf(p, "%s", fp_manu);
    len = p - page;
    if (len > *pos) {
        len -= *pos;
    }
    else {
        len = 0;
    }

    if (copy_to_user(buf, page, len < count ? len  : count)) {
        return -EFAULT;
    }

    *pos = *pos + (len < count ? len  : count);

    return len < count ? len  : count;
}

static struct file_operations fp_id_node_ctrl = {
    .read = fp_id_node_read,
};

static int fp_get_matched_chip_module(struct device *dev, int fp_id1, int fp_id2, int fp_id3)
{
    int i;
    for (i = 0; i < sizeof(fp_module_config_list)/sizeof(fp_module_config_t); ++i) {
        if ((fp_module_config_list[i].gpio_id_config_list[0] == fp_id1) &&
                (fp_module_config_list[i].gpio_id_config_list[1] == fp_id2) &&
                (fp_module_config_list[i].gpio_id_config_list[2] == fp_id3)) {
            switch (fp_module_config_list[i].fp_vendor_chip) {
                case FP_FPC_1022:
                    strcpy(fp_manu, CHIP_FPC);
                    strcat(fp_manu, "_1022");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_FPC_1022;
                case FP_FPC_1023:
                    strcpy(fp_manu, CHIP_FPC);
                    strcat(fp_manu, "_1023");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_FPC_1023;
                case FP_FPC_1140:
                    strcpy(fp_manu, CHIP_FPC);
                    strcat(fp_manu, "_1140");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_FPC_1140;
                case FP_FPC_1260:
                    strcpy(fp_manu, CHIP_FPC);
                    strcat(fp_manu, "_1260");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_FPC_1260;
                case FP_FPC_1270:
                    strcpy(fp_manu, CHIP_FPC);
                    strcat(fp_manu, "_1270");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_FPC_1270;
                case FP_GOODIX_3268:
                    strcpy(fp_manu, CHIP_GOODIX);
                    strcat(fp_manu, "_3268");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_GOODIX_3268;
                case FP_GOODIX_5288:
                    strcpy(fp_manu, CHIP_GOODIX);
                    strcat(fp_manu, "_5288");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_GOODIX_5288;
                case FP_GOODIX_5298:
                    strcpy(fp_manu, CHIP_GOODIX);
                    strcat(fp_manu, "_5298");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_GOODIX_5298;
                case FP_GOODIX_5298_GLASS:
                    strcpy(fp_manu, CHIP_GOODIX);
                    strcat(fp_manu, "_5298_GLASS");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_GOODIX_5298;
                case FP_GOODIX_5228:
                    strcpy(fp_manu, CHIP_GOODIX);
                    strcat(fp_manu, "_5228");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_GOODIX_5228;
                case FP_GOODIX_OPTICAL_95:
                    strcpy(fp_manu, CHIP_GOODIX);
                    strcat(fp_manu, "_OPTICAL_95");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_GOODIX_OPTICAL_95;
                case FP_SILEAD_OPTICAL_70:
                    strcpy(fp_manu, CHIP_SILEAD);
                    strcat(fp_manu, "_OPTICAL_70");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_SILEAD_OPTICAL_70;
                case FP_FPC_1023_GLASS:
                    strcpy(fp_manu, CHIP_FPC);
                    strcat(fp_manu, "_1023_GLASS");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_FPC_1023_GLASS;
                case FP_FPC_1028_COATING:
                    strcpy(fp_manu, CHIP_FPC);
                    strcat(fp_manu, "_1028_COATING");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_FPC_1028_COATING;
               case FP_FPC_1511_COATING:
                    strcpy(fp_manu, CHIP_FPC);
                    strcat(fp_manu, "_1511_COATING");
                    strcpy(g_engineermode_menu_config, fp_module_config_list[i].engineermode_menu_config);
                    return FP_FPC_1511_COATING;
                default:
                    dev_err(dev, "gpio ids matched but no matched vendor chip!");
                    return FP_UNKNOWN;
            }
        }
    }
    strcpy(fp_manu, CHIP_UNKNOWN);
    strcpy(g_engineermode_menu_config, ENGINEER_MENU_DEFAULT);
    return FP_UNKNOWN;
}

static int fp_register_proc_fs(struct fp_data *fp_data)
{
    uint32_t fp_id_retry;
    fp_id_retry = 0;

#if CONFIG_OPPO_FINGERPRINT_PROJCT == 18005 || CONFIG_OPPO_FINGERPRINT_PROJCT == 18323
    fp_data->fp_id1 = 1;
    fp_data->fp_id2 = 1;
#elif CONFIG_OPPO_FINGERPRINT_PROJCT == 18171 || CONFIG_OPPO_FINGERPRINT_PROJCT == 18571 || CONFIG_OPPO_FINGERPRINT_PROJCT == 18573
    fp_data->fp_id1 = gpio_get_value(fp_data->gpio_id1);;
    fp_data->fp_id2 = -1;
#elif CONFIG_OPPO_FINGERPRINT_PROJCT == 18611
    fp_data->fp_id1 = -1;
    fp_data->fp_id2 = -1;
#else
    fp_data->fp_id1 = gpio_get_value(fp_data->gpio_id1);
    fp_data->fp_id2 = gpio_get_value(fp_data->gpio_id2);
#endif

    switch (get_project()) {
#if CONFIG_OPPO_FINGERPRINT_PLATFORM == 660 /*sdm660*/
        case OPPO_16051:
        case OPPO_16103:
        case OPPO_16118:
        case OPPO_17011:
        case OPPO_17021:
        case OPPO_17081:
        case OPPO_17085:
        case OPPO_17091:
        case OPPO_18001:
        case OPPO_18316:
        case OPPO_18321:
        case OPPO_18005:
        case OPPO_18323:
            {
                fp_data->fp_id3 = gpio_get_value(fp_data->gpio_id3);
                fp_data->fpsensor_type = fp_get_matched_chip_module(fp_data->dev, fp_data->fp_id1, fp_data->fp_id2, fp_data->fp_id3);
                break;
            }
#endif
#if CONFIG_OPPO_FINGERPRINT_PLATFORM == 8953 /*msm8953*/
        case OPPO_16017:
            {
                fp_data->fp_id3 = -1;  /*gpio_id3 is not used on 16017,so it equals -1 to match fp_module_config_list*/
                fp_data->fpsensor_type = fp_get_matched_chip_module(fp_data->dev, fp_data->fp_id1, fp_data->fp_id2, fp_data->fp_id3);
                break;
            }
        case OPPO_17001:
        case OPPO_16027:
            {
                fp_data->fp_id3 = gpio_get_value(fp_data->gpio_id3);  /*for 16027 fp_id3 is ID0 from HW Config*/
                fp_data->fpsensor_type = fp_get_matched_chip_module(fp_data->dev, fp_data->fp_id1, fp_data->fp_id2, fp_data->fp_id3);
                break;
            }
        case OPPO_18171:
        case OPPO_18172:
        case OPPO_18571:
            {
                fp_data->fp_id3 = -1;
                fp_data->fpsensor_type = fp_get_matched_chip_module(fp_data->dev, fp_data->fp_id1, fp_data->fp_id2, fp_data->fp_id3);
                break;
            }
#endif
#if CONFIG_OPPO_FINGERPRINT_PLATFORM == 6763
        case OPPO_17031:
        case OPPO_17051:
        case OPPO_17071:
        case OPPO_17371:
        case OPPO_17321:
        case OPPO_17101:
            {
                fp_data->fp_id0 = gpio_get_value(fp_data->gpio_id0);
                fp_data->fpsensor_type = fp_get_matched_chip_module(fp_data->dev, fp_data->fp_id0, fp_data->fp_id1, fp_data->fp_id2);
                for (fp_id_retry = 0; (fp_data->fpsensor_type == FP_UNKNOWN) && (fp_id_retry < 3); fp_id_retry++) {
                    dev_err(fp_data->dev, "FP_UNKNOWN: fp_id0= %d, fp_id1= %d, fp_id2= %d, fp_id_retry= %d\n", \
                            fp_data->fp_id0, fp_data->fp_id1, fp_data->fp_id2, fp_id_retry);
                    msleep(20);
                    fp_data->fp_id1 = gpio_get_value(fp_data->gpio_id1);
                    fp_data->fp_id2 = gpio_get_value(fp_data->gpio_id2);
                    fp_data->fp_id0 = gpio_get_value(fp_data->gpio_id0);
                    fp_data->fpsensor_type = fp_get_matched_chip_module(fp_data->dev, fp_data->fp_id0, fp_data->fp_id1, fp_data->fp_id2);
                }
                break;
            }
#endif
#if CONFIG_OPPO_FINGERPRINT_PLATFORM == 6771
        case OPPO_17331:
        case OPPO_17197:
        case OPPO_18311:
        case OPPO_18011:
        case OPPO_18611:
            {
                fp_data->fp_id0 = gpio_get_value(fp_data->gpio_id0);
                fp_data->fpsensor_type = fp_get_matched_chip_module(fp_data->dev, fp_data->fp_id0, fp_data->fp_id1, fp_data->fp_id2);
                for (fp_id_retry = 0; (fp_data->fpsensor_type == FP_UNKNOWN) && (fp_id_retry < 3); fp_id_retry++) {
                    dev_err(fp_data->dev, "FP_UNKNOWN: fp_id0= %d, fp_id1= %d, fp_id2= %d, fp_id_retry= %d\n", \
                            fp_data->fp_id0, fp_data->fp_id1, fp_data->fp_id2, fp_id_retry);
                    msleep(20);
                    fp_data->fp_id1 = gpio_get_value(fp_data->gpio_id1);
                    fp_data->fp_id2 = gpio_get_value(fp_data->gpio_id2);
                    fp_data->fp_id0 = gpio_get_value(fp_data->gpio_id0);
                    fp_data->fpsensor_type = fp_get_matched_chip_module(fp_data->dev, fp_data->fp_id0, fp_data->fp_id1, fp_data->fp_id2);
                }
                break;
            }
#endif
#if CONFIG_OPPO_FINGERPRINT_PLATFORM == 8976 /*msm8976pro*/
        case OPPO_16037:
            {
                fp_data->fp_id0 = gpio_get_value(fp_data->gpio_id0);
                fp_data->fpsensor_type = fp_get_matched_chip_module(fp_data->dev, fp_data->fp_id1, fp_data->fp_id2, fp_data->fp_id0);
                break;
            }
        case OPPO_15103:
            {
                fp_data->fp_id3 = -1;
                fp_data->fpsensor_type = fp_get_matched_chip_module(fp_data->dev, fp_data->fp_id1, fp_data->fp_id2, fp_data->fp_id3);
                break;
            }
#endif
        default:
            {
                strcpy(fp_manu, CHIP_UNKNOWN);
                fp_data->fpsensor_type = fp_get_matched_chip_module(fp_data->dev, -1, -1, -1);
                break;
            }
    }

    if (strlen(fp_manu) >= FP_ID_MAX_LENGTH) {
        dev_err(fp_data->dev, "fp_name should be shorter than %d\n", FP_ID_MAX_LENGTH);
        return FP_ERROR_GENERAL;
    }

    /*  make the proc /proc/fp_id  */
    fp_id_dir = proc_create(fp_id_name, 0664, NULL, &fp_id_node_ctrl);
    if (fp_id_dir == NULL) {
        return FP_ERROR_GENERAL;
    }

    return FP_OK;
}

fp_vendor_t get_fpsensor_type(void)
{
    fp_vendor_t fpsensor_type = FP_UNKNOWN;

    fpsensor_type = fp_data_ptr->fpsensor_type;

    return fpsensor_type;
}

static int oppo_fp_common_probe(struct platform_device *fp_dev)
{
    int ret = 0;
    struct device *dev = &fp_dev->dev;
    struct fp_data *fp_data = NULL;

    fp_data = devm_kzalloc(dev, sizeof(struct fp_data), GFP_KERNEL);
    if (fp_data == NULL) {
        dev_err(dev, "fp_data kzalloc failed\n");
        ret = -ENOMEM;
        goto exit;
    }

    fp_data->dev = dev;
    fp_data_ptr = fp_data;
    ret = fp_gpio_parse_dts(fp_data);
    if (ret) {
        goto exit;
    }

#if CONFIG_OPPO_FINGERPRINT_PLATFORM == 6763 || CONFIG_OPPO_FINGERPRINT_PLATFORM == 6771
    msleep(20);
#endif

    ret = fp_register_proc_fs(fp_data);
    if (ret) {
        goto exit;
    }
    return FP_OK;

exit:

    if (oppo_fp_common_dir) {
        remove_proc_entry(oppo_fp_common_dir_name, NULL);
    }

    if (fp_id_dir) {
        remove_proc_entry(fp_id_name, NULL);
    }

    dev_err(dev, "fp_data probe failed ret = %d\n", ret);
    if (fp_data) {
        devm_kfree(dev, fp_data);
    }

    return ret;
}

static int oppo_fp_common_remove(struct platform_device *pdev)
{
    return FP_OK;
}

static struct of_device_id oppo_fp_common_match_table[] = {
    {   .compatible = "oppo,fp_common", },
    {}
};

static struct platform_driver oppo_fp_common_driver = {
    .probe = oppo_fp_common_probe,
    .remove = oppo_fp_common_remove,
    .driver = {
        .name = "oppo_fp_common",
        .owner = THIS_MODULE,
        .of_match_table = oppo_fp_common_match_table,
    },
};

static int __init oppo_fp_common_init(void)
{
    return platform_driver_register(&oppo_fp_common_driver);
}

static void __exit oppo_fp_common_exit(void)
{
    platform_driver_unregister(&oppo_fp_common_driver);
}

subsys_initcall(oppo_fp_common_init);
module_exit(oppo_fp_common_exit)
