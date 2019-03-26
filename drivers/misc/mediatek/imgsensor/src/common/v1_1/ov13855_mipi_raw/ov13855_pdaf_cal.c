#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

#include "kd_camera_typedef.h"
//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov13855mipiraw_Sensor.h"
#define PFX "OV13855_eeprom"
#define LOG_INF(format, args...)	pr_err(PFX "[%s] " format, __FUNCTION__, ##args)

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);

#define USHORT             unsigned short
//#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define OV13855_EEPROM_READ_ID  0xA1
#define OV13855_EEPROM_WRITE_ID   0xA0//eeprom id 0xA0

#define OV13855_I2C_SPEED        400
#define OV13855_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048
unsigned char OV13855_eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;

bool selective_read_eeprom_ov13855(kal_uint16 addr, BYTE* data)
{
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};
    if (addr > OV13855_MAX_OFFSET)
    {
        return false;
    }
    if (iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, OV13855_EEPROM_WRITE_ID) < 0)
    {
        return false;
    }
    return true;
}

static bool _read_ov13855_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size )
{
    int i = 0;
    int offset = addr;
    for (i = 0; i < size; i++)
    {
        if (!selective_read_eeprom_ov13855(offset, &data[i]))
        {
            return false;
        }
        //LOG_INF("read_eeprom proc[%d] %d\n",offset, data[i]);
        //if (i >= 1360)
        //LOG_INF("read_eeprom proc[%d] %d\n",offset, data[i]);
        offset++;
    }
    get_done = true;
    last_size = size;
    last_offset = addr;
    return true;
}

bool read_ov13855_eeprom(kal_uint16 addr, unsigned char* data, kal_uint32 size)
{
    int addr_proc1 = 0x1400;//from the first valid data on
    int size_proc1 = 496;//the first total valid data size
    int i=0;
    int addr_proc2 = 0x1600;//from the second valid data on
    int size_proc2 = 876;//the second total valid data size
    int total_pd_size = size_proc1+size_proc2;
    unsigned char OV13855_eeprom_proc1_data[496]= {0};
    unsigned char OV13855_eeprom_proc2_data[876]= {0};

    LOG_INF("read_ov13855_eeprom, size = %d\n", total_pd_size);

    if (!get_done || last_size != size || last_offset != addr)
    {
        if (!_read_ov13855_eeprom(addr_proc1, OV13855_eeprom_proc1_data, size_proc1))
        {
            get_done = 0;
            last_size = 0;
            last_offset = 0;
            return false;
        }
    }

    if (!get_done || last_size != size_proc2 || last_offset != addr_proc2) {
        if (!_read_ov13855_eeprom(addr_proc2, OV13855_eeprom_proc2_data, size_proc2)) {
            get_done = 0;
            last_size = 0;
            last_offset = 0;
            return false;
        }
    }
    for (i=0; i<total_pd_size; i++)
    {
        if(i<size_proc1)
        {
            OV13855_eeprom_data[i] = OV13855_eeprom_proc1_data[i];
        } else {
            OV13855_eeprom_data[i] = OV13855_eeprom_proc2_data[i-size_proc1];
        }
    }
    memcpy(data, OV13855_eeprom_data, total_pd_size);
    return true;
}
