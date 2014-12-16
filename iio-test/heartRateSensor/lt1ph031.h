#ifndef LT1PH03_H
#define LT1PH03_H

#include "../iio.h"
//#include <linux/iio/common/st_sensors.h>

#define OPTICAL_DEV_NAME	"lt1ph03"
#define OPTICAL_MAX_NAME		17
#define OPTICAL_MAX_4WAI		1

#define OPTICAL_TX_MAX_LENGTH		2
#define OPTICAL_RX_MAX_LENGTH		6
//VungGV
#define ST_SENSORS_MAX_NAME			17
#define OPTICAL_I2C_MULTIREAD	0x80

/* ADDSEL is LOW */
#define REGS_PART_ID		0x00
#define REGS_REV_ID			0x01
#define REGS_SEQ_ID			0x02
#define REGS_INT_CFG		0x03
#define REGS_IRQ_ENABLE		0x04
#define REGS_IRQ_MODE1		0x05
#define REGS_IRQ_MODE2		0x06
#define REGS_HW_HEY			0x07
#define REGS_MEAS_RATE		0x08
#define REGS_ALS_RATE		0x09
#define REGS_PS_RATE		0x0A
#define REGS_ALS_LOW_TH0	0x0B
#define REGS_ALS_LOW_TH1	0x0C
#define REGS_ALS_HI_TH0		0x0D
#define REGS_ALS_HI_TH1		0x0E
#define REGS_PS_LED21		0x0F
#define REGS_PS_LED3		0x10
#define REGS_PS1_TH0		0x11
#define REGS_PS1_TH1		0x12
#define REGS_PS2_TH			0x13
//0x14 reserved
#define REGS_PS3_TH			0x15
//0x16 reserved
#define REGS_PARAM_WR		0x17
#define REGS_COMMAND		0x18
#define REGS_RESPONSE		0x20
#define REGS_IRQ_STATUS		0x21
#define REGS_ALS_VIS_DATA0	0x22
#define REGS_ALS_VIS_DATA1	0x23
#define REGS_ALS_IR_DATA0	0x24
#define REGS_ALS_IR_DATA1	0x25
#define REGS_PS1_DATA0		0x26
#define REGS_PS1_DATA1		0x27
#define REGS_PS2_DATA0		0x28
#define REGS_PS2_DATA1		0x29
#define REGS_PS3_DATA0		0x2A
#define REGS_PS3_DATA1		0x2B
#define REGS_AUX_DATA0		0x2C
#define REGS_AUX_DATA1		0x2D
#define REGS_PARAM_RD		0x2E
#define REGS_CHIP_STAT		0x30

#define B_INT_STICKY			(0)
#define B_INT_AUTO_CLEAR		(BIT(1))
#define B_INT_OUTPUT_DISABLE	(0)
#define B_INT_OUTPUT_ENABLE		(BIT(0))

#define B_CMD_INT_ENABLE		(BIT(5))
#define B_PS1_INT_ENBALE		(BIT(2))
#define B_ALS_INT_DISABLE		(0)

#define B_PS1_IM_COMPLETE		(0)
#define B_PS1_IM_CROSS_TH		(1UL<<4)
#define B_PS1_IM_GT_TH			(3UL<<4)
#define B_ALS_IM_NONE			(0)

#define B_CMD_INT_ANY			(0)
#define B_CMD_INT_ERR			(BIT(2))

#define B_MEAS_RATE_10MS		(0x84)
#define B_MEAS_RATE_20MS		(0x94)
#define B_MEAS_RATE_100MS		(0xB9)
#define B_MEAS_RATE_496MS		(0xDF)
#define B_MEAS_RATE_1984MS		(0xFF)

#define B_ALS_RATE_FORCE		(0)
#define B_ALS_RATE_1			(0x08)
#define B_ALS_RATE_10			(0x32)
#define B_ALS_RATE_100			(0x69)

#define B_PS_RATE_FORCE			(0)
#define B_PS_RATE_1				(0x08)
#define B_PS_RATE_10			(0x32)
#define B_PS_RATE_100			(0x69)

#define B_PS_LED_NONE			(0)
#define B_PS_LED_5_6MA			(0x01)
#define B_PS_LED_11_2MA			(0x02)
#define B_PS_LED_22_4MA			(0x03)
#define B_PS_LED_44_8MA			(0x04)

#define B_IRQ_CMD_FLAG			(BIT(5))
#define B_IRQ_PS1_FLAG			(BIT(2))
#define B_IRQ_ALS0_FLAG			(BIT(0))
#define B_IRQ_ALS1_FLAG			(BIT(1))

#define B_CHIP_RUNNING			(BIT(2))
#define B_CHIP_SUSPEND			(BIT(1))
#define B_CHIP_SLEEP			(BIT(0))

// endian 		.info_mask = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \

/*#define OPTICAL_CHANNELS(device_type, index, mod, bits, addr) \
{ \
	.type = device_type, \
	.modified = 1, \
	.info_mask = (1 << IIO_CHAN_INFO_SCALE_SHARED) | \
		 (1 << IIO_CHAN_INFO_CALIBBIAS_SEPARATE),\
	.scan_index = index, \
	.channel2 = mod, \
	.address = addr, \
	.scan_type = { \
		.sign = 's', \
		.realbits = bits, \
		.shift = 0, \
		.storagebits = 16, \
	}, \
	.extend_name = NULL, \
}
//		.endianness = endian,\*/

#define OPTICAL_DEV_ATTR_SAMP_FREQ() \
		IIO_DEV_ATTR_SAMP_FREQ(S_IWUGO | S_IRUGO, \
			optical_sysfs_get_sampling_frequency, \
			optical_sysfs_set_sampling_frequency)

#define OPTICAL_DEV_ATTR_SAMP_FREQ_AVAIL() \
		IIO_DEV_ATTR_SAMP_FREQ_AVAIL( \
			optical_sysfs_sampling_frequency_avail)

#define OPTICAL_DEV_ATTR_SCALE_AVAIL(name) \
		IIO_DEVICE_ATTR(name, S_IRUGO, \
			optical_sysfs_scale_avail, NULL , 0);

#define OPTICAL_DEV_ATTR_PROX_VAL(name) \
		IIO_DEVICE_ATTR(name, S_IRUGO, \
			optical_sysfs_prox_val, NULL, 0)

#define OPTICAL_DEV_ATTR_LIGHT_VAL(name) \
		IIO_DEVICE_ATTR(name, S_IRUGO, \
			optical_sysfs_light_val, NULL, 0)

#define OPTICAL_DEV_ATTR_HR_VAL(name) \
		IIO_DEVICE_ATTR(name, S_IRUGO, \
			optical_sysfs_hr_val, NULL, 0)

#define SEND_COMMAND(sdata,cmd)		\
	do{sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_COMMAND, cmd);}while(0)	
#define SEND_COMMAND_WITH_PARAM(sdata,cmd,param)	\
	do{\
		sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_PARAM_WR, param);\
		sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_COMMAND, cmd);\
	}while(0)
#define SEND_COMMAND_PARAM_SET(sdata,offset,param)	SEND_COMMAND_WITH_PARAM(sdata,(offset+0x0A),param)

struct optical_transfer_buffer 
{
	struct mutex buf_lock;
	u8 rx_buf[OPTICAL_RX_MAX_LENGTH];
	u8 tx_buf[OPTICAL_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct optical_transfer_function 
{
	int (*read_byte) (struct optical_transfer_buffer *tb,struct device *dev, u8 reg_addr, u8 *res_byte);
	int (*write_byte) (struct optical_transfer_buffer *tb,struct device *dev, u8 reg_addr, u8 data);
	int (*read_multiple_byte) (struct optical_transfer_buffer *tb,struct device *dev, u8 reg_addr, int len, u8 *data,bool multiread_bit);
};

struct optical_data 
{
	struct device *dev;
	struct iio_trigger *trig;
	struct optical_sensors_list *sensor;
//	struct st_sensor_fullscale_avl *current_fullscale;
	bool enabled;
	bool multiread_bit;
	char *buffer_data;
//	unsigned int odr;
	unsigned int (*get_irq_data_ready) (struct iio_dev *indio_dev);
	const struct optical_transfer_function *tf;
	struct optical_transfer_buffer tb;
	struct regulator *vdd_ana;
	struct regulator *vdd_io;
	struct delayed_work poll_sensor_work;

};

struct optical_power {
	u8 addr;
	u8 mask;
	u8 value_off;
	u8 value_on;
};

/**
 * struct st_sensor_data_ready_irq - ST sensor device data-ready interrupt
 * @addr: address of the register.
 * @mask: mask to write the on/off value.
 * struct ig1 - represents the Interrupt Generator 1 of sensors.
 * @en_addr: address of the enable ig1 register.
 * @en_mask: mask to write the on/off value for enable.
 */
struct optical_data_ready_irq {
	u8 addr;
	u8 mask;
	struct {
		u8 en_addr;
		u8 en_mask;
	} ig1;
};

/**
 * struct st_sensors - ST sensors list
 * @wai: Contents of WhoAmI register. This is an array of acceptable WhoAmI results.
 * @sensors_supported: List of supported sensors by struct itself.
 * @ch: IIO channels for the sensor.
 * @odr: Output data rate register and ODR list available.
 * @pw: Power register of the sensor.
 * @enable_axis: Enable one or more axis of the sensor.
 * @fs: Full scale register and full scale list available.
 * @bdu: Block data update register.
 * @drdy_irq: Data ready register of the sensor.
 * @multi_read_bit: Use or not particular bit for [I2C/SPI] multi-read.
 * @bootime: samples to discard when sensor passing from power-down to power-up.
 */
struct optical_sensors_list {
	u8 wai[OPTICAL_MAX_4WAI];
	char sensors_supported[OPTICAL_MAX_4WAI][ST_SENSORS_MAX_NAME];
	struct iio_chan_spec *ch;
	//struct st_sensor_odr odr;
	struct optical_power pw;
	//struct st_sensor_axis enable_axis;
	//struct st_sensor_fullscale fs;
	//struct st_sensor_bdu bdu;
	//struct optical_data_ready_irq drdy_irq;
    //struct st_sensor_oneshot oneshot;
    
	bool multi_read_bit;
	unsigned int bootime;
};

#endif


