/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     OV13855mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 * Setting version:
 * ------------
 *   update full pd setting for OV13855EB_03B
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "ov13855mipiraw_Sensor.h"

#ifndef VENDOR_EDIT
#define VENDOR_EDIT
#endif
#ifdef VENDOR_EDIT
/*Feng.Hu@Camera.Driver 20170815 add for multi project using one build*/
#include <soc/oppo/oppo_project.h>
#endif

#define PFX "OV13855_camera_sensor"
#define LOG_INF(fmt, args...)   pr_err(PFX "[%s] " fmt, __FUNCTION__, ##args)
#ifdef VENDOR_EDIT
//zhengjiang.zhu@Camera.Drv, 2017/10/2 add for register device info
#define DEVICE_VERSION_OV13855    "ov13855"
extern void register_imgsensor_deviceinfo(char *name, char *version, u8 module_id);
static kal_uint8 deviceInfo_register_value = 0x00;
//zhengjiang.zhu@Prd6.BaseDrv.Camera, 2017/02/14 add for slow shutter
//static bool bNeedSetNormalMode = KAL_FALSE;
static kal_uint32 streaming_control(kal_bool enable);
#endif

extern bool read_ov13855_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size);
//extern void kdSetI2CSpeed(u16 i2cSpeed);

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = {
    .sensor_id = OV13855_SENSOR_ID,

    .checksum_value = 0x8b86a64,//test_Pattern_mode

    .pre = {
        .pclk = 108000000,    //record different mode's pclk
        .linelength =  1122,
        .framelength = 3214,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2112,
        .grabwindow_height = 1568,
        .mipi_data_lp2hs_settle_dc = 90,//unit , ns , 85
        .max_framerate = 300,
        .mipi_pixel_rate = 225600000,
    },
    .cap = {
        .pclk = 108000000,
        .linelength = 1122,
        .framelength = 3240,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4224,
        .grabwindow_height = 3136,
        .mipi_data_lp2hs_settle_dc = 90, //80
        .mipi_pixel_rate = 451200000,
        .max_framerate = 300,
    },
    /*size@24fps, same as capture*/
    .cap1 = {
        .pclk = 108000000,
        .linelength = 2244,//
        .framelength = 3214,//3214,4010
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4224,
        .grabwindow_height = 3136,
        .mipi_data_lp2hs_settle_dc = 90, //80
        .mipi_pixel_rate = 451200000,
        .max_framerate = 150,
    },
    .normal_video = {
        .pclk = 108000000,
        .linelength = 1122,
        .framelength = 3214,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4224,
        .grabwindow_height = 2376,
        .mipi_data_lp2hs_settle_dc = 90, //80
        .max_framerate = 300,
        .mipi_pixel_rate = 451200000,
    },
    .hs_video = {
        .pclk = 108000000,
        .linelength = 1122,
        .framelength = 804,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640,
        .grabwindow_height = 480,
        .mipi_data_lp2hs_settle_dc = 90,
        .mipi_pixel_rate = 225600000,
        .max_framerate = 1200,
    },
    .slim_video = {
        .pclk = 108000000,
        .linelength =  1122,
        .framelength = 3216,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1024,
        .grabwindow_height = 768,
        .mipi_data_lp2hs_settle_dc = 90,
        .mipi_pixel_rate = 225600000,
        .max_framerate = 30,
    },

    .margin =8,
    .min_shutter = 0x4,        //min shutter
    .max_frame_length = 0x7fff,//max framelength by sensor register's limitation
    .ae_shut_delay_frame = 0,  //check
    .ae_sensor_gain_delay_frame = 0,  //check
    .ae_ispGain_delay_frame = 2,
    .ihdr_support = 0,
    .ihdr_le_firstline = 0,
    .sensor_mode_num = 5,//support sensor mode num

    .cap_delay_frame = 3,        //enter capture delay frame num
    .pre_delay_frame = 2,         //enter preview delay frame num
    .video_delay_frame = 2,        //enter video delay frame num
    .hs_video_delay_frame = 2,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 2,//enter slim video delay frame num
    //#ifdef VENDOR_EDIT
    //zhengjiang.zhu@Cam.driver, 2017/09/11 modify 8ma to 4ma
    .isp_driving_current = ISP_DRIVING_4MA, //mclk driving current
    //#endif
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
    .i2c_addr_table = { 0x6C,0x20, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
    .i2c_speed = 400,
};

static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,
    .sensor_mode = IMGSENSOR_MODE_INIT,
    .shutter = 0x3D0,
    .gain = 0x100,
    .dummy_pixel = 0,
    .dummy_line = 0,
    .current_fps = 30,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = KAL_FALSE,
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_en = 0,
    .i2c_write_id = 0x20,
};

/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] =
{
    { 4256, 3168, 0,   8, 4256, 3152, 2128, 1576,  8,	4,	 2112, 1568, 0,   0,   2112, 1568}, // Preview
    { 4256, 3168, 0,   8, 4256, 3152, 4256, 3152,  16,	8,	 4224, 3136, 0,   0,   4224, 3136},//capture
    { 4256, 3168, 0,   8, 4256, 2376, 4256, 2376,  16,	380,	 4224, 2376, 0,   0,   4224, 2376},//normal-video
    { 4256, 3168, 64, 64, 4128, 3104, 1032,  776,  196,	148,	 640,	480, 0,   0,    640,  480},//hs-video
    { 4256, 3168, 64, 64, 4128, 3104, 1032,  776,  4,	4,	 1024,	768, 0,   0,   1024,  768},//slim-video
};

/*PD information update*/
static SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX	= 32,
    .i4PitchY	= 32,
    .i4PairNum  =8,//32*32 is cropped into 16*8 sub block of 8 pairnum, per sub block includes just only one pd pair pixels
    .i4SubBlkW  =16,
    .i4SubBlkH  =8,
    .i4BlockNumX = 132,
    .i4BlockNumY = 98,
    .i4PosL = {{14, 6},{30, 6},{6, 10},{22, 10},{14, 22},{30, 22},{6, 26},{22, 26},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}},
    .i4PosR = {{14, 2},{30, 2},{6, 14},{22, 14},{14, 18},{30, 18},{6, 30},{22, 30},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}},
};
// 16:9 video pd information
static SET_PD_BLOCK_INFO_T imgsensor_pd_info_16_9 =
{
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX	= 32,
    .i4PitchY	= 32,
    .i4PairNum  =8,//32*32 is cropped into 16*8 sub block of 8 pairnum, per sub block includes just only one pd pair pixels
    .i4SubBlkW  =16,
    .i4SubBlkH  =8,
    .i4BlockNumX = 132,
    .i4BlockNumY = 74,
    .i4PosL = {{14, 6},{30, 6},{6, 10},{22, 10},{14, 22},{30, 22},{6, 26},{22, 26},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}},
    .i4PosR = {{14, 2},{30, 2},{6, 14},{22, 14},{14, 18},{30, 18},{6, 30},{22, 30},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}},
    .i4Crop = {{0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
};
// Quarter size skip mode pd information
static SET_PD_BLOCK_INFO_T imgsensor_pd_info_QuarterSize =
{
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX  = 32,
    .i4PitchY  = 32,
    .i4PairNum	=8,//32*32 is cropped into 16*8 sub block of 8 pairnum, per sub block includes just only one pd pair pixels
    .i4SubBlkW	=16,
    .i4SubBlkH	=8,
    .i4BlockNumX = 132,
    .i4BlockNumY = 98,
    .i4PosL = {{14, 6},{30, 6},{6, 10},{22, 10},{14, 22},{30, 22},{6, 26},{22, 26},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}},
    .i4PosR = {{14, 2},{30, 2},{6, 14},{22, 14},{14, 18},{30, 18},{6, 30},{22, 30},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}},
    .i4Crop = {{0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
};


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);

    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
    //kdSetI2CSpeed(imgsensor_info.i2c_speed);
}

static void set_dummy(void)
{
    //check
    //LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);

    write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
    write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
    write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
    write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
}

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    //kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;

    //LOG_INF("framerate = %d, min_framelength_en=%d\n", framerate,min_framelength_en);
    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    //LOG_INF("frame_length =%d\n", frame_length);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
    {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
    {
        imgsensor.min_frame_length = imgsensor.frame_length;
    }
    spin_unlock(&imgsensor_drv_lock);
    //LOG_INF("framerate = %d, min_framelength_en=%d\n", framerate,min_framelength_en);
    set_dummy();
}
//#ifdef VENDOR_EDIT
//zhengjiang.zhu@Prd6.BaseDrv.Camera, 2017/02/14 add for slow shutter
static void write_shutter(kal_uint32 shutter)
//#else
//static void write_shutter(kal_uint16 shutter)
//#endif
{
	//check
	static int long_exposure_status = 0;
	kal_uint16 realtime_fps = 0;
	kal_uint32 exposure_time = 0;
	kal_uint32 long_shutter = 0;

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;//increase current frame_length that makes shutter <= frame_length - margin.
	} else {
		imgsensor.frame_length = imgsensor.min_frame_length;
	}
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	//#ifndef VENDOR_EDIT
	//zhengjiang.zhu@Prd6.BaseDrv.Camera, 2017/02/14 add for slow shutter
	//shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	//#endif
	//frame_length and shutter should be an even number.
	shutter = (shutter >> 1) << 1;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;

	if (imgsensor.autoflicker_en == KAL_TRUE) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			realtime_fps = 296;
			set_max_framerate(realtime_fps, 0);
	    } else if (realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
			set_max_framerate(realtime_fps , 0);
	    } else {
			imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
			write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	    }
	} else {
		imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	}
	LOG_INF("frame_length is %d   shutter=%d\n", imgsensor.frame_length, shutter);
    if (shutter > 32767) {
		if (shutter > 2096800) {
			shutter = 2096000;
        }

		LOG_INF("enter to long exposure mode\n");
		exposure_time = shutter * imgsensor_info.cap.linelength / 108;//us
		long_shutter = ((exposure_time - 34500) / 256) * (100000 / 926);
		LOG_INF("Calc long exposure exposure_time:%d long_shutter %d \n", exposure_time, long_shutter);
		long_shutter = (long_shutter >> 1) << 1;
		//write_cmos_sensor(0x0100, 0x01);
		write_cmos_sensor(0x3208, 0x00);
		write_cmos_sensor(0x3018, 0xfd);
		write_cmos_sensor(0x3019, 0xf2);
		write_cmos_sensor(0x3c83, 0x7c);
		write_cmos_sensor(0x3c99, 0x03);
		/* wirte long shtter */
		write_cmos_sensor(0x3c9a, (long_shutter >> 24) & 0xff);
		write_cmos_sensor(0x3c9b, (long_shutter >> 16) & 0xff);
		write_cmos_sensor(0x3c9c, (long_shutter >> 8) & 0xff);
		write_cmos_sensor(0x3c9d, long_shutter & 0xff);
		write_cmos_sensor(0x380E, 0x0E);
		write_cmos_sensor(0x380F, 0x00);
		write_cmos_sensor(0x3500, 0x00);
		write_cmos_sensor(0x3501, 0xD0);
		write_cmos_sensor(0x3502, 0x00);
		write_cmos_sensor(0x3018, 0xf0);
		write_cmos_sensor(0x3019, 0xf0);
		write_cmos_sensor(0x3208, 0x10);
		write_cmos_sensor(0x3208, 0xa0);
		long_exposure_status = 1;
    } else {
		write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);
		write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
		write_cmos_sensor(0x3502, (shutter << 4) & 0xF0);
		if (long_exposure_status == 1) {
			long_exposure_status = 0;
			//write_cmos_sensor(0x0100, 0x01);
			write_cmos_sensor(0x3208, 0x02);
			write_cmos_sensor(0x3018, 0xf9);
			write_cmos_sensor(0x3019, 0xf2);
			write_cmos_sensor(0x3c83, 0xf9);
			write_cmos_sensor(0x3c99, 0x00);
			write_cmos_sensor(0x380e, 0x0c);
			write_cmos_sensor(0x380f, 0xA8);
			write_cmos_sensor(0x3501, 0x80);
			write_cmos_sensor(0x3018, 0xf0);
			write_cmos_sensor(0x3019, 0xf0);
			write_cmos_sensor(0x3208, 0x12);
			write_cmos_sensor(0x3208, 0xa2);
			LOG_INF("exit long exposure mode");
		}
	}
	//#else
	/*Warning : shutter must be even. Odd might happen Unexpected Results */
	//write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);
	//write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
	//write_cmos_sensor(0x3502, (shutter<<4)  & 0xF0);
	//#endif

	LOG_INF("shutter =%d, framelength =%d, realtime_fps =%d\n", shutter,imgsensor.frame_length, realtime_fps);
}
//#ifdef VENDOR_EDIT
//zhengjiang.zhu@Prd6.BaseDrv.Camera, 2017/02/14 add for slow shutter
static void set_shutter(kal_uint32 shutter)
//#else
//static void set_shutter(kal_uint16 shutter)
//#endif
{
    unsigned long flags;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
    write_shutter(shutter);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 iReg = 0x0000;

    //platform 1xgain = 64, sensor driver 1*gain = 0x80
    iReg = gain*128/BASEGAIN;

    if (iReg < 0x80)// sensor 1xGain
    {
        iReg = 0X80;
    }
    if (iReg > 0x7c0)// sensor 15.5xGain
    {
        iReg = 0X7C0;
    }

    return iReg;
}

static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;
    unsigned long flags;

    reg_gain = gain2reg(gain);
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.gain = reg_gain;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
    write_cmos_sensor(0x03508,(reg_gain >> 8));
    write_cmos_sensor(0x03509,(reg_gain&0xff));


    return gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
}

static void night_mode(kal_bool enable)
{
    LOG_INF("night_mode do nothing");
}

static void sensor_init(void)
{
    LOG_INF("sensor_init MULTI_WRITE");
    write_cmos_sensor(0x0103, 0x01);//SW Reset, need delay
    mdelay(10);

    write_cmos_sensor(0x0300, 0x02);
    write_cmos_sensor(0x0301, 0x00);
    write_cmos_sensor(0x0302, 0x5E);

    write_cmos_sensor(0x0304, 0x00);
    write_cmos_sensor(0x0305, 0x01);//PLL
    write_cmos_sensor(0x030b, 0x06);//EB_07B
    write_cmos_sensor(0x030c, 0x02);//EB_07C
    write_cmos_sensor(0x030d, 0x88);//EB_07C
    write_cmos_sensor(0x0312, 0x11);//EB_07C
    write_cmos_sensor(0x3022, 0x01);
    write_cmos_sensor(0x3012, 0x40);
    write_cmos_sensor(0x3013, 0x72);  //EB_04B
    write_cmos_sensor(0x3016, 0x72);//Pll
    write_cmos_sensor(0x301b, 0xf0);
    write_cmos_sensor(0x301f, 0xd0);
    write_cmos_sensor(0x3106, 0x15);
    write_cmos_sensor(0x3107, 0x23);
    write_cmos_sensor(0x3500, 0x00);
    write_cmos_sensor(0x3502, 0x00);
    write_cmos_sensor(0x3508, 0x02);
    write_cmos_sensor(0x3509, 0x00);
    write_cmos_sensor(0x350a, 0x00);
    write_cmos_sensor(0x350e, 0x00);
    write_cmos_sensor(0x3510, 0x00);
    write_cmos_sensor(0x3511, 0x02);
    write_cmos_sensor(0x3512, 0x00);
    write_cmos_sensor(0x3600, 0x2b);
    write_cmos_sensor(0x3601, 0x52);
    write_cmos_sensor(0x3602, 0x60);
    write_cmos_sensor(0x3612, 0x05);
    write_cmos_sensor(0x3613, 0xa4);
    write_cmos_sensor(0x3620, 0x80);
    write_cmos_sensor(0x3621, 0x10);  //EB_04E
    write_cmos_sensor(0x3622, 0x30);
    write_cmos_sensor(0x3624, 0x1c);
    write_cmos_sensor(0x3640, 0x10);  //0EB_4E
    write_cmos_sensor(0x3641, 0x70);  //0EB_4D
    write_cmos_sensor(0x3661, 0x80);
    write_cmos_sensor(0x3664, 0x73);
    write_cmos_sensor(0x3665, 0xa7);
    write_cmos_sensor(0x366e, 0xff);
    write_cmos_sensor(0x366f, 0xf4);
    write_cmos_sensor(0x3674, 0x00);
    write_cmos_sensor(0x3679, 0x0c);
    write_cmos_sensor(0x367f, 0x01);
    write_cmos_sensor(0x3680, 0x0c);
    write_cmos_sensor(0x3681, 0x50);//0EB_07A
    write_cmos_sensor(0x3682, 0x50);//0EB_07A
    write_cmos_sensor(0x3683, 0xa9);
    write_cmos_sensor(0x3684, 0xa9);//0EB_07A
    write_cmos_sensor(0x3709, 0x5f);//0EB_07C
    write_cmos_sensor(0x371a, 0x3e);
    write_cmos_sensor(0x3738, 0xcc);
    write_cmos_sensor(0x373d, 0x26);
    write_cmos_sensor(0x3764, 0x20);
    write_cmos_sensor(0x3765, 0x20);
    write_cmos_sensor(0x37a1, 0x36);
    write_cmos_sensor(0x37a8, 0x3b);
    write_cmos_sensor(0x37ab, 0x31);
    write_cmos_sensor(0x37c3, 0xf1);
    write_cmos_sensor(0x37c5, 0x00);
    write_cmos_sensor(0x37d8, 0x03);
    write_cmos_sensor(0x37da, 0xc2);
    write_cmos_sensor(0x37dc, 0x02);
    write_cmos_sensor(0x37e0, 0x00);
    write_cmos_sensor(0x37e1, 0x0a);
    write_cmos_sensor(0x37e2, 0x14);
    write_cmos_sensor(0x37e5, 0x03);
    write_cmos_sensor(0x3800, 0x00);
    write_cmos_sensor(0x3804, 0x10);
    write_cmos_sensor(0x3815, 0x01);
    write_cmos_sensor(0x3817, 0x01);
    write_cmos_sensor(0x3821, 0x00);
    write_cmos_sensor(0x3822, 0xc2);
    write_cmos_sensor(0x3823, 0x18);
    write_cmos_sensor(0x3832, 0x00);
    write_cmos_sensor(0x3c80, 0x00);
    write_cmos_sensor(0x3c87, 0x01);
    write_cmos_sensor(0x3c8c, 0x19);
    write_cmos_sensor(0x3c8d, 0x1c);
    write_cmos_sensor(0x3c90, 0x00);
    write_cmos_sensor(0x3c91, 0x00);
    write_cmos_sensor(0x3c92, 0x00);
    write_cmos_sensor(0x3c93, 0x00);
    write_cmos_sensor(0x3c94, 0x40); //EB_04B
    write_cmos_sensor(0x3c95, 0x54);
    write_cmos_sensor(0x3c96, 0x34);
    write_cmos_sensor(0x3c97, 0x04);
    write_cmos_sensor(0x3c98, 0x00);
    write_cmos_sensor(0x3d8c, 0x73);
    write_cmos_sensor(0x3d8d, 0xc0);
    write_cmos_sensor(0x3f00, 0x0b);
    write_cmos_sensor(0x3f03, 0x00);
    write_cmos_sensor(0x4001, 0xe0);
    write_cmos_sensor(0x4008, 0x00);
    write_cmos_sensor(0x4011, 0xf0);
    write_cmos_sensor(0x4017, 0x08); //EB_04B
    write_cmos_sensor(0x4052, 0x00);
    write_cmos_sensor(0x4053, 0x80);
    write_cmos_sensor(0x4054, 0x00);
    write_cmos_sensor(0x4055, 0x80);
    write_cmos_sensor(0x4056, 0x00);
    write_cmos_sensor(0x4057, 0x80);
    write_cmos_sensor(0x4058, 0x00);
    write_cmos_sensor(0x4059, 0x80);
    write_cmos_sensor(0x405e, 0x20);
    write_cmos_sensor(0x4500, 0x07);
    write_cmos_sensor(0x4503, 0x00);
    write_cmos_sensor(0x450a, 0x04);
    write_cmos_sensor(0x4809, 0x04);
    write_cmos_sensor(0x480c, 0x12);
    write_cmos_sensor(0x481f, 0x30); //EB_04D
    write_cmos_sensor(0x4833, 0x10);
    write_cmos_sensor(0x4d00, 0x03);
    write_cmos_sensor(0x4d01, 0xc9);
    write_cmos_sensor(0x4d02, 0xbc);
    write_cmos_sensor(0x4d03, 0xd7);
    write_cmos_sensor(0x4d04, 0xf0);
    write_cmos_sensor(0x4d05, 0xa2);
    write_cmos_sensor(0x5000, 0xff);
    write_cmos_sensor(0x5001, 0x07);
    write_cmos_sensor(0x5040, 0x39);
    write_cmos_sensor(0x5041, 0x10);
    write_cmos_sensor(0x5042, 0x10);
    write_cmos_sensor(0x5043, 0x84);
    write_cmos_sensor(0x5044, 0x62);
    write_cmos_sensor(0x5180, 0x00);
    write_cmos_sensor(0x5181, 0x10);
    write_cmos_sensor(0x5182, 0x02);
    write_cmos_sensor(0x5183, 0x0f);
    write_cmos_sensor(0x5200, 0x1b);
    write_cmos_sensor(0x520b, 0x07);
    write_cmos_sensor(0x520c, 0x0f);
    write_cmos_sensor(0x5300, 0x04);
    write_cmos_sensor(0x5301, 0x0c);
    write_cmos_sensor(0x5302, 0x0c);
    write_cmos_sensor(0x5303, 0x0f);
    write_cmos_sensor(0x5304, 0x00);
    write_cmos_sensor(0x5305, 0x70);
    write_cmos_sensor(0x5306, 0x00);
    write_cmos_sensor(0x5307, 0x80);
    write_cmos_sensor(0x5308, 0x00);
    write_cmos_sensor(0x5309, 0xa5);
    write_cmos_sensor(0x530a, 0x00);
    write_cmos_sensor(0x530b, 0xd3);
    write_cmos_sensor(0x530c, 0x00);
    write_cmos_sensor(0x530d, 0xf0);
    write_cmos_sensor(0x530e, 0x01);
    write_cmos_sensor(0x530f, 0x10);
    write_cmos_sensor(0x5310, 0x01);
    write_cmos_sensor(0x5311, 0x20);
    write_cmos_sensor(0x5312, 0x01);
    write_cmos_sensor(0x5313, 0x20);
    write_cmos_sensor(0x5314, 0x01);
    write_cmos_sensor(0x5315, 0x20);
    write_cmos_sensor(0x5316, 0x08);
    write_cmos_sensor(0x5317, 0x08);
    write_cmos_sensor(0x5318, 0x10);
    write_cmos_sensor(0x5319, 0x88);
    write_cmos_sensor(0x531a, 0x88);
    write_cmos_sensor(0x531b, 0xa9);
    write_cmos_sensor(0x531c, 0xaa);
    write_cmos_sensor(0x531d, 0x0a);
    write_cmos_sensor(0x5405, 0x02);
    write_cmos_sensor(0x5406, 0x67);
    write_cmos_sensor(0x5407, 0x01);
    write_cmos_sensor(0x5408, 0x4a);

    write_cmos_sensor(0x0100,0x00);
    LOG_INF("sensor_init End");
    }

    static void preview_setting(void)
    {
    LOG_INF("preview_setting RES_2112x1568_30fps\n");
    write_cmos_sensor(0x0100,0x00);
    mdelay(10);

    write_cmos_sensor(0x0303, 0x01);
    write_cmos_sensor(0x3501, 0x40);
    write_cmos_sensor(0x3662, 0x10);
    write_cmos_sensor(0x3714, 0x28);
    write_cmos_sensor(0x3737, 0x08);
    write_cmos_sensor(0x3739, 0x20);
    write_cmos_sensor(0x37c2, 0x14);
    write_cmos_sensor(0x37d9, 0x0c);
    write_cmos_sensor(0x37e3, 0x08);
    write_cmos_sensor(0x37e4, 0x38); //EB_04B
    write_cmos_sensor(0x37e6, 0x08);

    write_cmos_sensor(0x3801, 0x00);//s_x=0x00

    write_cmos_sensor(0x3802, 0x00);
    write_cmos_sensor(0x3803, 0x06);//x_y=0x08

    write_cmos_sensor(0x3805, 0x9f);//e_x=0x109f;e_x-s_x=4255-0->4255+1=4256

    write_cmos_sensor(0x3806, 0x0c);
    write_cmos_sensor(0x3807, 0x55);//e_y=0xc4f;e_y-e_x=3151-8->3143+1=3144

    write_cmos_sensor(0x3808, 0x08);
    write_cmos_sensor(0x3809, 0x40);//o_x=0x840=2112

    write_cmos_sensor(0x380a, 0x06);
    write_cmos_sensor(0x380b, 0x20);//o_y=0x620=1568

    write_cmos_sensor(0x380c, 0x04);
    write_cmos_sensor(0x380d, 0x62);//vts=0x8c4=2244

    write_cmos_sensor(0x380e, 0x0C);//hts=0x648=1608
    write_cmos_sensor(0x380f, 0x8E);

    write_cmos_sensor(0x3811, 0x08);//x_offset=8

    write_cmos_sensor(0x3813, 0x06);//y_offset=4

    write_cmos_sensor(0x3814, 0x03);
    write_cmos_sensor(0x3816, 0x03);
    write_cmos_sensor(0x3820, 0xa8);
    write_cmos_sensor(0x3826, 0x04);
    write_cmos_sensor(0x3827, 0x90);
    write_cmos_sensor(0x3829, 0x07);
    write_cmos_sensor(0x4009, 0x0d);
    write_cmos_sensor(0x4050, 0x04);
    write_cmos_sensor(0x4051, 0x0b);
    write_cmos_sensor(0x4501, 0x88); //skip mode for quarter size PDAF
    write_cmos_sensor(0x4837, 0x1c);
    write_cmos_sensor(0x4902, 0x01);
    write_cmos_sensor(0x5004, 0x04);
    write_cmos_sensor(0x5016, 0x10);// ;pd_cycle_x_0
    write_cmos_sensor(0x5017, 0x10);// ;pd_cycle_y_0
    write_cmos_sensor(0x5018, 0x00);// ;pd_y_0
    write_cmos_sensor(0x5019, 0x07);// ;pd_x1_0
    write_cmos_sensor(0x501a, 0x03); //;pd_x2_0
    write_cmos_sensor(0x501b, 0x07);// ;pd_x3_0
    write_cmos_sensor(0x501c, 0x03);// ;pd_x4_0
    write_cmos_sensor(0x501e, 0x00); //;win_x_start
    write_cmos_sensor(0x501f, 0x00);
    write_cmos_sensor(0x5020, 0x00); //;win_y_start
    write_cmos_sensor(0x5021, 0x06);
    write_cmos_sensor(0x5022, 0x08); //;pd_win_width
    write_cmos_sensor(0x5023, 0x40);
    write_cmos_sensor(0x5024, 0x06); //;pd_win_height
    write_cmos_sensor(0x5025, 0x20);
    write_cmos_sensor(0x3820, 0xa8);

    LOG_INF("preview mode disable long exposure+");
    //Disable long exp. setting
    //write_cmos_sensor(0x3208, 0x02);
    //write_cmos_sensor(0x3018, 0xf9);
    //write_cmos_sensor(0x3019, 0xf2);
    write_cmos_sensor(0x3c83, 0xf9);
    write_cmos_sensor(0x3c99, 0x00);
    //write_cmos_sensor(0x380e, 0x0c);
    //write_cmos_sensor(0x380f, 0x8e);
    //write_cmos_sensor(0x3501, 0x80);
    //write_cmos_sensor(0x3018, 0xf0);
    //write_cmos_sensor(0x3019, 0xf0);
    //write_cmos_sensor(0x3208, 0x12);
    //write_cmos_sensor(0x3208, 0xa2);
    write_cmos_sensor(0x0100,0x01);
    LOG_INF("preview mode disable long exposure-");
}

static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("capture_setting RES_4224x3136_zsl_30fps currefps = %d\n", currefps);
    write_cmos_sensor(0x0100,0x00);

    mdelay(5);

    if (currefps == 150){
        //15fps for PIP

        write_cmos_sensor(0x0303, 0x00);
        write_cmos_sensor(0x3501, 0x80);
        write_cmos_sensor(0x3662, 0x12);
        write_cmos_sensor(0x3714, 0x24);
        write_cmos_sensor(0x3737, 0x04);
        write_cmos_sensor(0x3739, 0x12);
        write_cmos_sensor(0x37c2, 0x04);
        write_cmos_sensor(0x37d9, 0x0c);
        write_cmos_sensor(0x37e3, 0x04);
        write_cmos_sensor(0x37e4, 0x2A); //EB_04B
        write_cmos_sensor(0x37e6, 0x04);
        write_cmos_sensor(0x3801, 0x00);
        write_cmos_sensor(0x3802, 0x00);
        write_cmos_sensor(0x3803, 0x08);
        write_cmos_sensor(0x3805, 0x9f);
        write_cmos_sensor(0x3806, 0x0c);
        write_cmos_sensor(0x3807, 0x57);
        write_cmos_sensor(0x3808, 0x10);//0x1080
        write_cmos_sensor(0x3809, 0x80);
        write_cmos_sensor(0x380a, 0x0c);//0xC40
        write_cmos_sensor(0x380b, 0x40);
        write_cmos_sensor(0x380c, 0x04);
        write_cmos_sensor(0x380d, 0x62);

        /*write_cmos_sensor(0x380e, 0x0C);
        write_cmos_sensor(0x380f, 0x8E);*/

        write_cmos_sensor(0x380e,0x0f);//vts,4010
        write_cmos_sensor(0x380f,0xaa);

        write_cmos_sensor(0x3811, 0x10);
        write_cmos_sensor(0x3813, 0x08);
        write_cmos_sensor(0x3814, 0x01);
        write_cmos_sensor(0x3816, 0x01);
        write_cmos_sensor(0x3820, 0xa8);
        write_cmos_sensor(0x3826, 0x11);
        write_cmos_sensor(0x3827, 0x1c);
        write_cmos_sensor(0x3829, 0x03);
        write_cmos_sensor(0x4009, 0x0f);
        write_cmos_sensor(0x4050, 0x04);
        write_cmos_sensor(0x4051, 0x0b);
        write_cmos_sensor(0x4501, 0x00);
        write_cmos_sensor(0x4837, 0x0e);
        write_cmos_sensor(0x4902, 0x01);
        write_cmos_sensor(0x5004, 0x00);


    }else{
        //30fps

        write_cmos_sensor(0x0303, 0x00);
        write_cmos_sensor(0x3501, 0x80);
        write_cmos_sensor(0x3662, 0x12);
        write_cmos_sensor(0x3714, 0x24);
        write_cmos_sensor(0x3737, 0x04);
        write_cmos_sensor(0x3739, 0x12);
        write_cmos_sensor(0x37c2, 0x04);
        write_cmos_sensor(0x37d9, 0x0c);
        write_cmos_sensor(0x37e3, 0x04);
        write_cmos_sensor(0x37e4, 0x2A); //EB_04B
        write_cmos_sensor(0x37e6, 0x04);

        write_cmos_sensor(0x3801, 0x00);//s_x =0

        write_cmos_sensor(0x3802, 0x00);
        write_cmos_sensor(0x3803, 0x08);//s_y = 8

        write_cmos_sensor(0x3805, 0x9f);//e_x = 4255;s_y-s_x=4255;4255+1=4256-2*x_offset=4224

        write_cmos_sensor(0x3806, 0x0c);
        write_cmos_sensor(0x3807, 0x57);//e_y = 3159;e_y-e_x=3151;3151+1=3152-2*y_offset=3136

        write_cmos_sensor(0x3808, 0x10);
        write_cmos_sensor(0x3809, 0x80);//o_x=4224

        write_cmos_sensor(0x380a, 0x0c);
        write_cmos_sensor(0x380b, 0x40);//o_y=3136

        write_cmos_sensor(0x380c, 0x04);
        write_cmos_sensor(0x380d, 0x62);//hts=1122

        write_cmos_sensor(0x380e, 0x0C);
        write_cmos_sensor(0x380f, 0xA8);//vts=3214

        write_cmos_sensor(0x3811, 0x10);//x_offset=16
        write_cmos_sensor(0x3813, 0x08);//y_offset=8

        write_cmos_sensor(0x3814, 0x01);
        write_cmos_sensor(0x3816, 0x01);
        write_cmos_sensor(0x3820, 0xa8);
        write_cmos_sensor(0x3826, 0x11);
        write_cmos_sensor(0x3827, 0x1c);
        write_cmos_sensor(0x3829, 0x03);
        write_cmos_sensor(0x4009, 0x0f);
        write_cmos_sensor(0x4050, 0x04);
        write_cmos_sensor(0x4051, 0x0b);
        write_cmos_sensor(0x4501, 0x00);
        write_cmos_sensor(0x4837, 0x0e);
        write_cmos_sensor(0x4902, 0x01);
        write_cmos_sensor(0x5004, 0x00);
}

    LOG_INF("capture mode disable long exposure+");
    //Disable long exp. setting
    //write_cmos_sensor(0x3208, 0x02);
    //write_cmos_sensor(0x3018, 0xf9);
    //write_cmos_sensor(0x3019, 0xf2);
    write_cmos_sensor(0x3c83, 0xf9);
    write_cmos_sensor(0x3c99, 0x00);
    //write_cmos_sensor(0x380e, 0x0c);
    //write_cmos_sensor(0x380f, 0x8e);
    //write_cmos_sensor(0x3501, 0x80);
    //write_cmos_sensor(0x3018, 0xf0);
    //write_cmos_sensor(0x3019, 0xf0);
    //write_cmos_sensor(0x3208, 0x12);
    //write_cmos_sensor(0x3208, 0xa2);
    write_cmos_sensor(0x0100, 0x01);
    LOG_INF("capture mode disable long exposure-");
}

static void normal_video_setting(kal_uint16 currefps)
{
    LOG_INF("normal_video_setting RES_4224x3136_zsl_30fps\n");
    write_cmos_sensor(0x0100,0x00);
    mdelay(5);

    write_cmos_sensor(0x0303, 0x00);
    write_cmos_sensor(0x3501, 0x80);
    write_cmos_sensor(0x3662, 0x12);
    write_cmos_sensor(0x3714, 0x24);
    write_cmos_sensor(0x3737, 0x04);
    write_cmos_sensor(0x3739, 0x12);
    write_cmos_sensor(0x37c2, 0x04);
    write_cmos_sensor(0x37d9, 0x0c);
    write_cmos_sensor(0x37e3, 0x04);
    write_cmos_sensor(0x37e4, 0x2A);//EB_04B
    write_cmos_sensor(0x37e6, 0x04);
    write_cmos_sensor(0x3801, 0x00);
    write_cmos_sensor(0x3802, 0x01);
    write_cmos_sensor(0x3803, 0x80);
    write_cmos_sensor(0x3805, 0x9f);
    write_cmos_sensor(0x3806, 0x0a);
    write_cmos_sensor(0x3807, 0xdf);
    write_cmos_sensor(0x3808, 0x10);
    write_cmos_sensor(0x3809, 0x80);
    write_cmos_sensor(0x380a, 0x09);
    write_cmos_sensor(0x380b, 0x48);
    write_cmos_sensor(0x380c, 0x04);
    write_cmos_sensor(0x380d, 0x62);
    write_cmos_sensor(0x380e, 0x0C);
    write_cmos_sensor(0x380f, 0x8E);
    write_cmos_sensor(0x3811, 0x10);
    write_cmos_sensor(0x3813, 0x10);
    write_cmos_sensor(0x3814, 0x01);
    write_cmos_sensor(0x3816, 0x01);
    write_cmos_sensor(0x3820, 0xa8);
    write_cmos_sensor(0x3826, 0x11);
    write_cmos_sensor(0x3827, 0x1c);
    write_cmos_sensor(0x3829, 0x03);
    write_cmos_sensor(0x4009, 0x0f);
    write_cmos_sensor(0x4050, 0x04);
    write_cmos_sensor(0x4051, 0x0b);
    write_cmos_sensor(0x4501, 0x00);
    write_cmos_sensor(0x4837, 0x0e);
    write_cmos_sensor(0x4902, 0x01);
    write_cmos_sensor(0x5004, 0x00);

	write_cmos_sensor(0x0100,0x01);
}

static void hs_video_setting(void)
{
    write_cmos_sensor(0x0100,0x00);
    mdelay(5);
    LOG_INF("hs_video_setting RES_1024x768_120fps\n");

    write_cmos_sensor(0x0303, 0x01);//PLL

    write_cmos_sensor(0x3501, 0x20);
    write_cmos_sensor(0x3662, 0x08);
    write_cmos_sensor(0x3714, 0x30);
    write_cmos_sensor(0x3737, 0x08);
    write_cmos_sensor(0x3739, 0x20);
    write_cmos_sensor(0x37c2, 0x2c);
    write_cmos_sensor(0x37d9, 0x06);
    write_cmos_sensor(0x37e3, 0x08);
    write_cmos_sensor(0x37e4, 0x36); //EB_04B
    write_cmos_sensor(0x37e6, 0x08);

    //crop
    write_cmos_sensor(0x3801, 0x40);
    write_cmos_sensor(0x3802, 0x00);
    write_cmos_sensor(0x3803, 0x40);
    write_cmos_sensor(0x3805, 0x5f);
    write_cmos_sensor(0x3806, 0x0c);
    write_cmos_sensor(0x3807, 0x5f);
    write_cmos_sensor(0x3808, 0x02);
    write_cmos_sensor(0x3809, 0x80);
    write_cmos_sensor(0x380a, 0x01);
    write_cmos_sensor(0x380b, 0xE0);
    write_cmos_sensor(0x380c, 0x04);
    write_cmos_sensor(0x380d, 0x62);

    write_cmos_sensor(0x380e, 0x03);//0x324->120fps
    write_cmos_sensor(0x380f, 0x24);

    write_cmos_sensor(0x3811, 0xc4);//offset for viewAngle
    write_cmos_sensor(0x3813, 0x94);

    write_cmos_sensor(0x3814, 0x07);
    write_cmos_sensor(0x3816, 0x07);
    write_cmos_sensor(0x3820, 0xac);
    write_cmos_sensor(0x3826, 0x04);
    write_cmos_sensor(0x3827, 0x48);
    write_cmos_sensor(0x3829, 0x03);
    write_cmos_sensor(0x4009, 0x05);
    write_cmos_sensor(0x4050, 0x02);
    write_cmos_sensor(0x4051, 0x05);
    write_cmos_sensor(0x4501, 0x00);
    write_cmos_sensor(0x4837, 0x1C);
    write_cmos_sensor(0x4902, 0x02);
    write_cmos_sensor(0x5004, 0x00);

    write_cmos_sensor(0x0100,0x01);
}

static void slim_video_setting(void)
{
    LOG_INF("hs_video_setting RES_1024x768_30fps\n");
    write_cmos_sensor(0x0100,0x00);
    mdelay(5);

    write_cmos_sensor(0x0303, 0x01);//PLL

    write_cmos_sensor(0x3501, 0x20);
    write_cmos_sensor(0x3662, 0x08);
    write_cmos_sensor(0x3714, 0x30);
    write_cmos_sensor(0x3737, 0x08);
    write_cmos_sensor(0x3739, 0x20);
    write_cmos_sensor(0x37c2, 0x2c);
    write_cmos_sensor(0x37d9, 0x06);
    write_cmos_sensor(0x37e3, 0x08);
    write_cmos_sensor(0x37e4, 0x36); //EB_04B
    write_cmos_sensor(0x37e6, 0x08);

    //crop
    write_cmos_sensor(0x3801, 0x40);
    write_cmos_sensor(0x3802, 0x00);
    write_cmos_sensor(0x3803, 0x40);
    write_cmos_sensor(0x3805, 0x5f);
    write_cmos_sensor(0x3806, 0x0c);
    write_cmos_sensor(0x3807, 0x5f);
    write_cmos_sensor(0x3808, 0x04);
    write_cmos_sensor(0x3809, 0x00);
    write_cmos_sensor(0x380a, 0x03);
    write_cmos_sensor(0x380b, 0x00);
    write_cmos_sensor(0x380c, 0x04);
    write_cmos_sensor(0x380d, 0x62);
    write_cmos_sensor(0x380e, 0x0c);//0xc90->30fps
    write_cmos_sensor(0x380f, 0x90);

    write_cmos_sensor(0x3811, 0x04);
    write_cmos_sensor(0x3813, 0x04);

    write_cmos_sensor(0x3814, 0x07);
    write_cmos_sensor(0x3816, 0x07);
    write_cmos_sensor(0x3820, 0xac);
    write_cmos_sensor(0x3826, 0x04);
    write_cmos_sensor(0x3827, 0x48);
    write_cmos_sensor(0x3829, 0x03);
    write_cmos_sensor(0x4009, 0x05);
    write_cmos_sensor(0x4050, 0x02);
    write_cmos_sensor(0x4051, 0x05);
    write_cmos_sensor(0x4501, 0x00);
    write_cmos_sensor(0x4837, 0x1C);
    write_cmos_sensor(0x4902, 0x02);
    write_cmos_sensor(0x5004, 0x00);

    write_cmos_sensor(0x0100,0x01);
}

static kal_uint32 return_sensor_id(void)
{
    return ( (read_cmos_sensor(0x300a) << 16) | (read_cmos_sensor(0x300b) << 8) | read_cmos_sensor(0x300c));
}
#ifdef VENDOR_EDIT
//zhengjiang.zhu@Camera.Drv, 2017/10/2 add for register device info
static kal_uint16 read_module_id(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,0xA0);
    return get_byte;
}
#endif
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 module_id = 0;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				#ifdef VENDOR_EDIT
				module_id = read_module_id(0x0000);
				if (deviceInfo_register_value == 0x00){
					register_imgsensor_deviceinfo("Cam_b", DEVICE_VERSION_OV13855, module_id);
					deviceInfo_register_value = 0x01;
				}
				#endif
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail:0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 1;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}

static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 1;
    kal_uint32 sensor_id = 0;
    LOG_INF("PLATFORM:MT6797,MIPI 4LANE +++++ ++++ \n");
    LOG_INF("preview 2112*1568@30fps,1080Mbps/lane; video 1024*768@120fps,864Mbps/lane; capture 4224*3136@30fps,1080Mbps/lane\n");

    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    sensor_init();

    spin_lock(&imgsensor_drv_lock);
    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.shutter = 0x3D0;
    imgsensor.gain = 0x100;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    imgsensor.pdaf_mode= 0;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("open End +++++ ++++ \n");
    return ERROR_NONE;
}

static kal_uint32 close(void)
{
    LOG_INF("E\n");
    streaming_control(KAL_FALSE);
    return ERROR_NONE;
}   /*  close  */

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.current_fps = imgsensor.current_fps;
    //imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        //imgsensor.autoflicker_en = KAL_FALSE;
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        //imgsensor.autoflicker_en = KAL_FALSE;
    }

    spin_unlock(&imgsensor_drv_lock);

    capture_setting(imgsensor.current_fps);

    return ERROR_NONE;
}   /* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    //imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);
    return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();
    return ERROR_NONE;
}

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
    return ERROR_NONE;
}

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight    = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth    = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;

    return ERROR_NONE;
}   /*  get_resolution  */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    if(scenario_id == 0){
        LOG_INF("scenario_id = %d\n", scenario_id);
    }

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->PDAF_Support = 1; /*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode*/

    //sensor_info->HDR_Support = 0; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR*/
    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;   // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}   /*  get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            preview(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            capture(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            normal_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            slim_video(image_window, sensor_config_data);
            break;
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }

    return ERROR_NONE;
}   /* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d\n ", framerate);
    // SetVideoMode Function should fix framerate
    if (framerate == 0) {
        // Dynamic frame rate
        return ERROR_NONE;
    }

    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE)) {
        imgsensor.current_fps = 296;
    } else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE)) {
        imgsensor.current_fps = 146;
    } else {
        imgsensor.current_fps = framerate;
    }
    spin_unlock(&imgsensor_drv_lock);

    set_max_framerate(imgsensor.current_fps,1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d \n", enable, framerate);

    spin_lock(&imgsensor_drv_lock);
    if (enable) {
        imgsensor.autoflicker_en = KAL_TRUE;
    } else {
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    //kal_int16 dummyLine;
    kal_uint32 frameHeight;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    if (framerate == 0) {
        return ERROR_NONE;
    }

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frameHeight = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
                LOG_INF("frameHeight = %d\n", frameHeight);
            spin_lock(&imgsensor_drv_lock);
            if(frameHeight > imgsensor_info.pre.framelength)
                imgsensor.dummy_line = frameHeight - imgsensor_info.pre.framelength;
            else
                imgsensor.dummy_line = 0;
            imgsensor.frame_length =imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if(imgsensor.frame_length>imgsensor.shutter)
                set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frameHeight = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            if(frameHeight > imgsensor_info.normal_video.framelength)
                imgsensor.dummy_line = frameHeight - imgsensor_info.normal_video.framelength;
            else
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if(imgsensor.frame_length>imgsensor.shutter)
                set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            frameHeight = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
            spin_lock(&imgsensor_drv_lock);
            if(frameHeight > imgsensor_info.cap.framelength)
                imgsensor.dummy_line = frameHeight - imgsensor_info.cap.framelength;
            else
                imgsensor.dummy_line = 0;
            imgsensor.frame_length =imgsensor_info.cap.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if(imgsensor.frame_length>imgsensor.shutter)
                set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frameHeight = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            if(frameHeight > imgsensor_info.hs_video.framelength)
                imgsensor.dummy_line = frameHeight - imgsensor_info.hs_video.framelength;
            else
                imgsensor.dummy_line = 0;
            imgsensor.frame_length =imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if(imgsensor.frame_length>imgsensor.shutter)
                set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frameHeight = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            if(frameHeight > imgsensor_info.slim_video.framelength)
                imgsensor.dummy_line = frameHeight - imgsensor_info.slim_video.framelength;
            else
                imgsensor.dummy_line = 0;
            imgsensor.frame_length =imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if(imgsensor.frame_length>imgsensor.shutter)
                set_dummy();
        default:  //coding with  preview scenario by default
            frameHeight = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            if(frameHeight > imgsensor_info.pre.framelength)
                imgsensor.dummy_line = frameHeight - imgsensor_info.pre.framelength;
            else
                imgsensor.dummy_line = 0;
            imgsensor.frame_length =imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if(imgsensor.frame_length>imgsensor.shutter)
                set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    if(scenario_id == 0){
        LOG_INF("[3058]scenario_id = %d\n", scenario_id);
    }

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    //check,power+volum+
    LOG_INF("enable: %d\n", enable);

    if (enable) {
        write_cmos_sensor(0x4503, 0x80);
    } else {
        write_cmos_sensor(0x4503, 0x00);
    }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
    LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
    if (enable){
        write_cmos_sensor(0x0100, 0x01);
    } else {
        write_cmos_sensor(0x0100, 0x00);
    }

    mdelay(10);
    return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;
    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
    SET_PD_BLOCK_INFO_T *PDAFinfo;
    #ifndef VENDOR_EDIT
    /*zhengjiang.zhu@Camera.Drv, 2017/10/18 remove for feature_id log   Bug:1132127*/
    if(!((feature_id == 3040) || (feature_id == 3058))){
        printk("feature_id = %d\n", feature_id);
     }
    #endif
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        #ifdef VENDOR_EDIT
        /*zhengjiang.zhu@Camera.driver, 2017/10/17	add  for  module id*/
        case SENSOR_FEATURE_CHECK_MODULE_ID:
            *feature_return_para_32 = imgsensor_info.module_id;
            break;
        #endif
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            //get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data_32, (MUINT32 *)(*(feature_data_32+1)));
            break;
        case SENSOR_FEATURE_GET_PDAF_DATA:
            LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
            read_ov13855_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data_32;
            spin_unlock(&imgsensor_drv_lock);
            LOG_INF("current fps :%d\n", imgsensor.current_fps);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data_32);
            //wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(*(feature_data_32+1));
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
            LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
            streaming_control(KAL_FALSE);
            break;
        case SENSOR_FEATURE_SET_STREAMING_RESUME:
            LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
            if (*feature_data != 0) {
                set_shutter(*feature_data);
             }
            streaming_control(KAL_TRUE);
            break;

        case SENSOR_FEATURE_GET_PDAF_INFO:
            //LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
            PDAFinfo= (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

        switch (*feature_data) {
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T));
                break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info_16_9,sizeof(SET_PD_BLOCK_INFO_T));
                break;
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info_QuarterSize,sizeof(SET_PD_BLOCK_INFO_T));
                break;
            case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            case MSDK_SCENARIO_ID_SLIM_VIDEO:
            default:
                break;
        }
            break;

        case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
            //LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
            //PDAF capacity enable or not, 2p8 only full size support PDAF
            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;// xxx is not supported
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
                    break;
                default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
            }
        break;
        case SENSOR_FEATURE_SET_PDAF:
            LOG_INF("PDAF mode :%d\n", imgsensor.pdaf_mode);
            break;
        case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
            {
            	kal_uint32 rate;

            	switch (*feature_data) {
            	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            		rate = imgsensor_info.cap.mipi_pixel_rate;
            		break;
            	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            		rate = imgsensor_info.normal_video.mipi_pixel_rate;
            		break;
            	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            		rate = imgsensor_info.hs_video.mipi_pixel_rate;
            		break;
            	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            	default:
            		rate = imgsensor_info.pre.mipi_pixel_rate;
            		break;
            	}
            	*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
            }
            break;
        default:
            break;
    }

    return ERROR_NONE;
}   /*  feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 OV13855_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}
