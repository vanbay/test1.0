/*
 * STMicroelectronics pressure driver
 *
 * Copyright 2012-2013 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
//#include <linux/irqdomain.h>
#include <linux/of_irq.h>
#include <asm/unaligned.h>
#include "../iio.h"
#include "../sysfs.h"
//#include <linux/iio/common/st_sensors.h>
//#include <linux/iio/common/st_sensors_i2c.h>
#include "../trigger.h"
#include "../ring_generic.h"
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include "../ring_sw.h"
#include "../ring_hw.h"
//#include <../buffer.h>?
#include "../trigger_consumer.h"
//#include <linux/iio/triggered_buffer.h>?
#include <linux/irqreturn.h>
//#include <../events.h>
//#include <../types.h>
#include "lt1ph03.h"

//iio_chan_spec l1ph03
#define L1PH03_SCAN_PROX_X (0)
#define L1PH03_SCAN_LIGHT_X (1)

#define OPTICAL_SENSOR_DEBUG

static ssize_t optical_sysfs_get_sampling_frequency(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t optical_sysfs_set_sampling_frequency(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);
static ssize_t optical_sysfs_sampling_frequency_avail(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t optical_sysfs_prox_val(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t optical_sysfs_light_val(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t optical_sysfs_hr_val(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t optical_sysfs_scale_avail(struct device *dev,struct device_attribute *attr, char *buf);
static int optical_read_info_raw(struct iio_dev *indio_dev,struct iio_chan_spec const *ch, int *val);
static int optical_read_raw(struct iio_dev *indio_dev,struct iio_chan_spec const *ch, int *val,int *val2, long mask);
static int optical_write_raw(struct iio_dev *indio_dev,struct iio_chan_spec const *chan, int val, int val2, long mask);
static int optical_check_device_support(struct iio_dev *indio_dev,int num_sensors_list, const struct optical_sensors_list *sensors);
#if 0
static int optical_write_data_with_mask(struct iio_dev *indio_dev,u8 reg_addr, u8 mask, u8 data);
#endif
static int optical_set_enable(struct iio_dev *indio_dev,bool enable);
static int optical_init_sensor(struct iio_dev *indio_dev);

/*******************buffer functions start***********************/
static int optical_get_buffer_element(struct iio_dev *indio_dev, u8 *buf);
static irqreturn_t optical_trigger_handler(int irq, void *p);
static int optical_set_dataready_irq(struct iio_dev *indio_dev, bool enable);
static int optical_trig_set_state(struct iio_trigger *trig, bool state);
static int optical_buffer_preenable(struct iio_dev *indio_dev);
static int optical_buffer_postenable(struct iio_dev *indio_dev);
static int optical_buffer_predisable(struct iio_dev *indio_dev);
static int optical_allocate_ring(struct iio_dev *indio_dev);
static void optical_deallocate_ring(struct iio_dev *indio_dev);

/*******************buffer functions end***********************/
static int optical_allocate_trigger(struct iio_dev *indio_dev,const struct iio_trigger_ops *trigger_ops);
static void optical_deallocate_trigger(struct iio_dev *indio_dev);


static int optical_common_probe(struct iio_dev *indio_dev);
static void optical_common_remove(struct iio_dev *indio_dev);
static unsigned int optical_i2c_get_irq(struct iio_dev *indio_dev);
static int optical_i2c_read_byte(struct optical_transfer_buffer *tb,struct device *dev, u8 reg_addr, u8 *res_byte);
static int optical_i2c_read_multiple_byte(struct optical_transfer_buffer *tb, struct device *dev,u8 reg_addr, int len, u8 *data, bool multiread_bit);
static int optical_i2c_write_byte(struct optical_transfer_buffer *tb,struct device *dev, u8 reg_addr, u8 data);
static void optical_i2c_configure(struct iio_dev *indio_dev,struct i2c_client *client, struct optical_data *sdata);
static int optical_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id);
static int optical_i2c_remove(struct i2c_client *client);
static u32 optical_HR_Average(u16 data, u16 sample_rate);
static void optical_HR_CalcAPW(u32 data, u16 num, s32 *pdata);
static u8 optical_HR_DetectPeak(s32 data, u16 sample_rate, s32 threshold, u16 *heart_rate);

static const u8 reg_defaults[] = 
{
	//0x00
	0x00,//part id
	0x00,//rev id
	0x00,//seq id
	B_INT_STICKY|B_INT_OUTPUT_DISABLE,//int cfg
	//0x04
	B_CMD_INT_ENABLE|B_PS1_INT_ENBALE|B_ALS_INT_DISABLE,//irq enable
	B_PS1_IM_COMPLETE|B_ALS_IM_NONE,//int mode1
	B_CMD_INT_ERR,//int mode2
	0x17,//hw key
	//0x08
	B_MEAS_RATE_10MS,//meas rate
	B_ALS_RATE_FORCE,//als rate
	B_PS_RATE_1,//ps rate
	0x00,//als low th 7:0
	//0x0c
	0x00,//als low th 15:8
	0x00,//als hi th 7:0
	0x00,//als hi th 15:8
	B_PS_LED_44_8MA,//ps led drive current
	
};


static const struct iio_chan_spec optical_channels[] = //khong giong
{
	/*OPTICAL_CHANNELS( 
            IIO_PROXIMITY,
            0,
            IIO_MOD_X,
            16,
            REGS_PS1_DATA0
            ),
	OPTICAL_CHANNELS( 
            IIO_LIGHT,
            1,
            IIO_MOD_X,
            16,
            REGS_ALS_VIS_DATA0
            ),*/
	IIO_CHAN(IIO_PROXIMITY, 1, 0, 0, NULL, 0, IIO_MOD_X,
		 (1 << IIO_CHAN_INFO_SCALE_SEPARATE),
		 REGS_PS1_DATA0,L1PH03_SCAN_PROX_X, IIO_ST('s', 16, 16, 0), 0),

	IIO_CHAN(IIO_LIGHT, 1, 0, 0, NULL, 0, IIO_MOD_X,
		 (1 << IIO_CHAN_INFO_SCALE_SEPARATE),
		 REGS_ALS_VIS_DATA0, L1PH03_SCAN_LIGHT_X , IIO_ST('s', 16, 16, 0), 0),
	IIO_CHAN_SOFT_TIMESTAMP(2)
};


static const struct optical_sensors_list optical_sensors[] = //co 1 sensor su dung lam heartRate
{
    {
        .wai = { 0x43 },
        .sensors_supported = {
            [0] = OPTICAL_DEV_NAME,
        },
        .ch = (struct iio_chan_spec *)optical_channels,
        .pw = {
            .addr = REGS_MEAS_RATE,
            .mask = 0xFF,
            .value_on = B_MEAS_RATE_10MS,
            .value_off = (0),
        },
        .multi_read_bit = true,
        .bootime = 0,
    },
};


static wait_queue_head_t proxWait;
static wait_queue_head_t lightWait;
static wait_queue_head_t hrWait;
static u8 proxVal;
static u16 lightVal;
static bool proxValValid;
static bool lightValValid;
static bool hrValValid;
//heart rate
static u32 avgData;
static s32 accData;
static u16 heart_rate;


static ssize_t optical_sysfs_get_sampling_frequency(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#if 0
	printk (KERN_ALERT "[%s]\n", __FUNCTION__); 	
	return sprintf(buf, "%d\n", 12);//invalid number for test
#else
	int len = 0;
	u8 i;
	u8 regs[49] = {0};	
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct optical_data *sdata = iio_priv(indio_dev);
	for(i=0;i<49;i++)
	{
		sdata->tf->read_byte(&sdata->tb, sdata->dev,i, regs+i);
	}
	len = sprintf(buf,"[00]%x %x %x %x\n[04]%x %x %x %x\n[08]%x %x %x %x\n[0c]%x %x %x %x\n[10]%x %x %x %x\n[14]%x %x %x %x\n[18]%x %x %x %x\n[1c]%x %x %x %x\n[20]%x %x %x %x\n[24]%x %x %x %x\n[28]%x %x %x %x\n[2c]%x %x %x %x %x\n",\
		regs[0],regs[1],regs[2],regs[3],\
		regs[4],regs[5],regs[6],regs[7],\
		regs[8],regs[9],regs[10],regs[11],\
		regs[12],regs[13],regs[14],regs[15],\
		regs[16],regs[17],regs[18],regs[19],\
		regs[20],regs[21],regs[22],regs[23],\
		regs[24],regs[25],regs[26],regs[27],\
		regs[28],regs[29],regs[30],regs[31],\
		regs[32],regs[33],regs[34],regs[35],\
		regs[36],regs[37],regs[38],regs[39],\
		regs[40],regs[41],regs[42],regs[43],\
		regs[44],regs[45],regs[46],regs[47],\
		regs[48]);
	return len;
#endif
}

static ssize_t optical_sysfs_set_sampling_frequency(struct device *dev,

		struct device_attribute *attr, const char *buf, size_t size)
{
#if 1//temp
	int err;
	unsigned int reg_val;
	unsigned char reg;
	unsigned char val;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct optical_data *sdata = iio_priv(indio_dev);
	err = kstrtoint(buf, 16, &reg_val);
	if (err < 0)
		goto conversion_error;
	reg = (reg_val&0xff00)>>8;
	val = (reg_val&0xff);

	printk (KERN_ALERT "[%s: setReg 0x%x 0x%x]\n", __FUNCTION__,reg, val); 
	sdata->tf->write_byte(&sdata->tb, sdata->dev,reg, val);//?
	
conversion_error:
	return err < 0 ? err : size;

#else
	printk (KERN_ALERT "[%s]\n", __FUNCTION__); 
	return 0;//invalid number for test
#endif
}

static ssize_t optical_sysfs_sampling_frequency_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int len = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	printk (KERN_ALERT "[%s]\n", __FUNCTION__); 
	mutex_lock(&indio_dev->mlock);

	len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",12);//invalid number for test

	mutex_unlock(&indio_dev->mlock);
	buf[len - 1] = '\n';

	return len;
}

static ssize_t optical_sysfs_prox_val(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int len = 0;
	long timeout;

	timeout = wait_event_interruptible_timeout(proxWait,
						proxValValid,
						msecs_to_jiffies(2000));//?
	if (!timeout)
		return -EIO;
		
	len = sprintf(buf,"0x%04x\n",proxVal);
	proxValValid = false;
	return len;
}

static ssize_t optical_sysfs_light_val(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int len = 0;
	long timeout;

	timeout = wait_event_interruptible_timeout(lightWait,
						lightValValid,
						msecs_to_jiffies(2000));//?
	if (!timeout)
		return -EIO;
		
	len = sprintf(buf,"0x%04x\n",lightVal);
	lightValValid = false;
	return len;
}

static ssize_t optical_sysfs_hr_val(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int len = 0;
	long timeout;

	timeout = wait_event_interruptible_timeout(hrWait,
						hrValValid,
						msecs_to_jiffies(2000));
	if (!timeout)
		return -EIO;
		
	len = sprintf(buf,"0x%04x\n",heart_rate);
	hrValValid = false;
	return len;
}

static ssize_t optical_sysfs_scale_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int len = 0;
//	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct iio_dev *indio_dev = iio_dev_get_devdata((struct iio_dev*)dev);
	printk (KERN_ALERT "[%s]\n", __FUNCTION__); 
	mutex_lock(&indio_dev->mlock);
	len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06u ",1234);//invalid number for test
	mutex_unlock(&indio_dev->mlock);
	buf[len - 1] = '\n';

	return len;
}

static OPTICAL_DEV_ATTR_SAMP_FREQ();
static OPTICAL_DEV_ATTR_SAMP_FREQ_AVAIL();
static OPTICAL_DEV_ATTR_SCALE_AVAIL(optical_scale_available);
static OPTICAL_DEV_ATTR_PROX_VAL(prox_val);
static OPTICAL_DEV_ATTR_LIGHT_VAL(light_val);
static OPTICAL_DEV_ATTR_HR_VAL(hr_val);


static struct attribute *optical_attributes[] = {
    &iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_optical_scale_available.dev_attr.attr,
    &iio_dev_attr_sampling_frequency.dev_attr.attr,
    &iio_dev_attr_prox_val.dev_attr.attr,
    &iio_dev_attr_light_val.dev_attr.attr,
    &iio_dev_attr_hr_val.dev_attr.attr,
    NULL,
};

static const struct attribute_group optical_attribute_group = {
    .attrs = optical_attributes,
};

static struct attribute *optical_event_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_optical_scale_available.dev_attr.attr,
	NULL,
};

static struct attribute_group optical_event_attribute_group = {
	.attrs = optical_event_attributes,
	.name = "events",
};

static const struct iio_info optical_info = 
{
    .driver_module = THIS_MODULE,
    .attrs = &optical_attribute_group,
    .read_raw = &optical_read_raw,
    .write_raw = &optical_write_raw,
	.event_attrs = &optical_event_attribute_group,
};

static int optical_read_info_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *ch, int *val)
{
	int err;
	struct optical_data *sdata = iio_priv(indio_dev);
	//struct optical_data *sdata = indio_dev->dev_data;
	mutex_lock(&indio_dev->mlock);
//Cho nay can xem ki vunggv
	//if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED)
	if (indio_dev->currentmode == INDIO_RING_TRIGGERED)
	{
		err = -EBUSY;
		goto read_error;
	} 
	else 
	{
		//err = st_sensors_set_enable(indio_dev, true);
		//if (err < 0)
		//	goto read_error;

        //err = st_sensors_oneshot(indio_dev);
		//if (err < 0)
		//	goto read_error;

		//msleep((sdata->sensor->bootime * 1000) / sdata->odr);
		if(ch->type == IIO_PROXIMITY)
		{
			u8 outdata[1];
			err = sdata->tf->read_byte(&sdata->tb, sdata->dev,
				ch->address, outdata);
			*val = outdata[0];
		}
		else
		{
			u8 outdata[2];
			if((err = sdata->tf->read_byte(&sdata->tb, sdata->dev,ch->address, outdata)) >=0 )
			{
				err = sdata->tf->read_byte(&sdata->tb, sdata->dev,ch->address, outdata+1);
				*val = (s16)get_unaligned_le16(outdata);
			}
		}

		
		if (err < 0)
			goto read_error;

		*val = *val >> ch->scan_type.shift;

        //err = st_sensors_set_enable(indio_dev, false);
	}
	mutex_unlock(&indio_dev->mlock);

	return err;

read_error:
	mutex_unlock(&indio_dev->mlock);
	return err;
}

static int optical_read_raw(struct iio_dev *indio_dev,
            struct iio_chan_spec const *ch, int *val,
                            int *val2, long mask)
{
    int err;  
    //struct optical_data *adata = iio_priv(indio_dev); 
    
    switch (mask)
    {
        //case IIO_CHAN_INFO_RAW:
	case 0:
            err = optical_read_info_raw(indio_dev, ch, val);
            if (err < 0)
                goto read_error;

            return IIO_VAL_INT;

        //case IIO_CHAN_INFO_SCALE:
	case (1 << IIO_CHAN_INFO_SCALE_SHARED):
	case (1 << IIO_CHAN_INFO_SCALE_SEPARATE):
            *val = 0;                         
            return IIO_VAL_INT_PLUS_MICRO;

        default:
            return -EINVAL;
    }

read_error:
    return err;
}

static int optical_write_raw(struct iio_dev *indio_dev,
        struct iio_chan_spec const *chan, int val, int val2, long mask)
{
    int err;

    switch (mask) {
    //case IIO_CHAN_INFO_SCALE:
    case (1 << IIO_CHAN_INFO_SCALE_SEPARATE):
        err = 0;//nothing to do
        break;
    default:
        return -EINVAL;
    }

    return err;
}

static int optical_check_device_support(struct iio_dev *indio_dev,
			int num_sensors_list, const struct optical_sensors_list *sensors)
{
	u8 wai=0x00;
	int i, n, err, j;
	struct optical_data *sdata = iio_priv(indio_dev);
	//struct optical_data *sdata= indio_dev->dev_data;
	printk (KERN_ALERT "[%s %d]\n", __FUNCTION__,num_sensors_list); 
	err = sdata->tf->read_byte(&sdata->tb, sdata->dev,REGS_PART_ID, &wai);
	if (err < 0) 
	{
		dev_err(&indio_dev->dev, "failed to read Who-Am-I register.\n");
		goto read_wai_error;
	}

	for (i = 0; i < num_sensors_list; i++) 
	{
		for (j = 0; j < ARRAY_SIZE(sensors[i].wai); ++j) 
		{
			if (sensors[i].wai[j] == wai) 
			{
				printk (KERN_ALERT "[chip ID 0x%02x i=%d j=%d]\n", wai,i,j);	
				goto out;
			}
		}
	}

out:
	if (i == num_sensors_list)
		goto device_not_supported;

	for (n = 0; n < ARRAY_SIZE(sensors[i].sensors_supported); n++) 
	{
		printk (KERN_ALERT "[%s|%s]\n", indio_dev->name,&sensors[i].sensors_supported[n][0]);
		if (strcmp(indio_dev->name,&sensors[i].sensors_supported[n][0]) == 0)
			break;
	}
	if (n == ARRAY_SIZE(sensors[i].sensors_supported)) 
	{
		dev_err(&indio_dev->dev, "device name and WhoAmI mismatch.\n");
		goto sensor_name_mismatch;
	}
	
	sdata->sensor = (struct optical_sensors_list *)&sensors[i];
		
	printk("vung ok optical_check_device_support -------");
	return i;

device_not_supported:
	dev_err(&indio_dev->dev, "device not supported: WhoAmI (0x%x).\n", wai);
sensor_name_mismatch:
	err = -ENODEV;
read_wai_error:
	return err;
}
#if 0
static int optical_write_data_with_mask(struct iio_dev *indio_dev,
						u8 reg_addr, u8 mask, u8 data)
{
	int err;
	u8 new_data;
	struct optical_data *sdata = iio_priv(indio_dev);
	//struct optical_data *sdata= indio_dev->dev_data;
	err = sdata->tf->read_byte(&sdata->tb, sdata->dev, reg_addr, &new_data);
	if (err < 0)
		goto st_sensors_write_data_with_mask_error;
	new_data = ((new_data & (~mask)) | ((data << __ffs(mask)) & mask));
	err = sdata->tf->write_byte(&sdata->tb, sdata->dev, reg_addr, new_data);
st_sensors_write_data_with_mask_error:
	return err;
}
#endif

static int optical_set_enable(struct iio_dev *indio_dev,bool enable)
{
	//u8 tmp_value;
	int err = -EINVAL;
	struct optical_data *sdata = iio_priv(indio_dev);
	//struct optical_data *sdata= indio_dev->dev_data;
#if 0
	printk (KERN_ALERT "[optical_set_enable %d %d %d %d]\n", enable,sdata->sensor->pw_prox.addr,sdata->sensor->pw_prox.mask,sdata->sensor->pw_prox.value_on);
	if (enable) 
	{
		tmp_value = sdata->sensor->pw_prox.value_on;

		err = optical_write_data_with_mask(indio_dev,
				sdata->sensor->pw_prox.addr,
				sdata->sensor->pw_prox.mask, tmp_value);
		if (err < 0)
			goto set_enable_error;

		tmp_value = sdata->sensor->pw_light.value_on;

		err = optical_write_data_with_mask(indio_dev,
				sdata->sensor->pw_light.addr,
				sdata->sensor->pw_light.mask, tmp_value);
		if (err < 0)
			goto set_enable_error;

		sdata->enabled = true;
	}
	else 
	{	
		err = optical_write_data_with_mask(indio_dev,
				sdata->sensor->pw_prox.addr,
				sdata->sensor->pw_prox.mask,
				sdata->sensor->pw_prox.value_off);
		if (err < 0)
			goto set_enable_error;
		err = optical_write_data_with_mask(indio_dev,
				sdata->sensor->pw_light.addr,
				sdata->sensor->pw_light.mask,
				sdata->sensor->pw_light.value_off);
		if (err < 0)
			goto set_enable_error;
		sdata->enabled = false;
	}
#else
	if(enable)
	{
		printk (KERN_ALERT "[optical_set_enable %d]\n", enable);
		printk("vungGV---------------------------------optical_set_enable");
		err = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_IRQ_STATUS, 0x27);//clear all int
		if (err < 0)
			goto set_enable_error;
		err = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_INT_CFG, B_INT_STICKY|B_INT_OUTPUT_ENABLE);
		if (err < 0)
			goto set_enable_error;
		SEND_COMMAND(sdata,0x0D);//ps auto

		SEND_COMMAND_WITH_PARAM(sdata,0xA1,0x01);//enable ps1
		
		sdata->enabled = true;
	}
	else
	{
		u8 rspReg=0;
		/* tri-states INT pin to stop any LT_1PHXX interrupts */
		err = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_INT_CFG, B_INT_STICKY|B_INT_OUTPUT_DISABLE);
		if (err < 0)
			goto set_enable_error;
		/* Need to make sure the machine is paused */
		/* condier that error condition in response register is asynchronous */
		while (1)
		{
			printk (KERN_ALERT "[optical_set_enable %d]\n", enable);
			/* Keep sending nops until the response is zero */
			while (1)
			{
				rspReg = 0x00;
				sdata->tf->read_byte(&sdata->tb, sdata->dev,REGS_RESPONSE, &rspReg);
				if(rspReg == 0x00)
					break;
				else
					SEND_COMMAND(sdata,0x00);//nop
			}
			/* Pause the device */
			SEND_COMMAND(sdata,0x0B);//ps als pause
			/* Wait for response */
			while(1)
			{
				rspReg = 0x00;
				sdata->tf->read_byte(&sdata->tb, sdata->dev,REGS_RESPONSE, &rspReg);
				if (rspReg != 0x00)
					break;				
			}
			/* When the PsAlsPause() response is good, we expect it to be a '1'. */
			if (rspReg == 0x01)
			{
				sdata->enabled = false;
				break; 
			}
			else
			{
				//goto set_enable_error;
			}
		}	
	}
#endif

	
set_enable_error:
	return err;
}
static int optical_init_sensor(struct iio_dev *indio_dev)
{
	int err;
	struct optical_data *sdata = iio_priv(indio_dev);
	printk (KERN_ALERT "[%s]\n", __FUNCTION__); 
	mutex_init(&sdata->tb.buf_lock);
	SEND_COMMAND(sdata,0x01);//reset
	//asm volatile ("nop");
	//asm volatile ("nop");
	//asm volatile ("nop");
	//asm volatile ("nop");
	printk (KERN_ALERT "[%s 1]\n", __FUNCTION__); 
	err = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_HW_HEY, reg_defaults[REGS_HW_HEY]);
	if (err < 0)
		goto init_error;	
	printk (KERN_ALERT "[%s 2]\n", __FUNCTION__); 
	SEND_COMMAND_PARAM_SET(sdata,0x01,0x01);//enble ps only
	
	err = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_INT_CFG, reg_defaults[REGS_INT_CFG]);
	if (err < 0)
		goto init_error;
	
	err = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_IRQ_ENABLE, reg_defaults[REGS_IRQ_ENABLE]);
	if (err < 0)
		goto init_error;
	err = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_IRQ_MODE1, reg_defaults[REGS_IRQ_MODE1]);
	if (err < 0)
		goto init_error;
	err = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_IRQ_MODE2, reg_defaults[REGS_IRQ_MODE2]);
	if (err < 0)
		goto init_error;
	err = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_MEAS_RATE, reg_defaults[REGS_MEAS_RATE]);
	if (err < 0)
		goto init_error;
	err = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_ALS_RATE, reg_defaults[REGS_ALS_RATE]);
	if (err < 0)
		goto init_error;
	err = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_PS_RATE, reg_defaults[REGS_PS_RATE]);
	if (err < 0)
		goto init_error;
	err = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_PS_LED21, reg_defaults[REGS_PS_LED21]);
	if (err < 0)
		goto init_error;
	SEND_COMMAND(sdata,0x0D);//ps auto
init_error:
	if(err>=0)
	printk("vung optical_init_sensor -----------------ok");
	return err;
}


/*******************buffer functions start***********************/
static int optical_get_buffer_element(struct iio_dev *indio_dev, u8 *buf)
{
	int ret,len=0;
	u8 proxArray[2]={0},alsArray[2]={0};
	u16 proxData=0, alsData = 0;
	struct optical_data *sdata = iio_priv(indio_dev);
	ret = sdata->tf->read_byte(&sdata->tb, sdata->dev,
			REGS_PS1_DATA0, proxArray);
	if (ret < 0)
		goto read_data_channels_error;
	ret = sdata->tf->read_byte(&sdata->tb, sdata->dev,
			REGS_PS1_DATA1, proxArray+1);
	if (ret < 0)
		goto read_data_channels_error;
	proxData = get_unaligned_le16(proxArray);
	alsData = get_unaligned_le16(alsArray);

	ret = sdata->tf->write_byte(&sdata->tb, sdata->dev, REGS_IRQ_STATUS, 0x27);//clear ngat cai nay can xem ky VungGV
	if (ret < 0)
		goto read_data_channels_error;

	printk (KERN_ALERT "[%s %d %d]\n", __FUNCTION__,proxData,alsData); 	
#ifdef OPTICAL_SENSOR_DEBUG
	return 4;
#endif

	buf[len++] = proxArray[0];
	buf[len++] = proxArray[1];
	buf[len++] = alsArray[0];
	buf[len++] = alsArray[1];
	return len;
read_data_channels_error:
	printk (KERN_ALERT "[%s error %d]\n", __FUNCTION__,ret); 	
	return ret;
}

static irqreturn_t optical_trigger_handler(int irq, void *p)
{
	int len;
	struct iio_poll_func *pf = p;
	//struct iio_dev *indio_dev = pf->indio_dev;//VungGV
	struct iio_dev *indio_dev = pf->private_data;
	struct optical_data *sdata = iio_priv(indio_dev);
	//struct optical_data *sdata= indio_dev->dev_data;
	struct iio_ring_buffer *ring = indio_dev->ring;
	printk (KERN_ALERT "[%s %d]\n", __FUNCTION__,irq); 
	len = optical_get_buffer_element(indio_dev, sdata->buffer_data);

	if (len < 0)
		goto st_sensors_get_buffer_element_error;
	if (indio_dev->ring->scan_timestamp) //VungGV
		*(s64 *)((u8 *)sdata->buffer_data +
				ALIGN(len, sizeof(s64))) = pf->timestamp;

#define CONFIG_PUSH_EVENT   1
//VungGV add ev_len=1
#ifdef CONFIG_PUSH_EVENT
    if ( 0 == strcmp(indio_dev->name, "lt1ph03"))
    { 
	    iio_push_event(indio_dev,1,
		           IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY,
					        0,
					        IIO_EV_TYPE_THRESH,
					        IIO_EV_DIR_EITHER),
		           iio_get_time_ns());
		iio_push_event(indio_dev,1,
		           IIO_UNMOD_EVENT_CODE(IIO_LIGHT,
					        0,
					        IIO_EV_TYPE_THRESH,
					        IIO_EV_DIR_EITHER),
		           iio_get_time_ns());
    }
    else
    {
        goto st_sensors_get_buffer_element_error;
    }    
    #endif 

	//iio_push_to_buffers(indio_dev, sdata->buffer_data);//VungGV ??
    ring->access->store_to(ring, (u8 *)sdata->buffer_data, pf->timestamp);
	st_sensors_get_buffer_element_error:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;

}

static int optical_set_dataready_irq(struct iio_dev *indio_dev, bool enable)
{
	printk (KERN_ALERT "[%s %d]\n", __FUNCTION__,enable);  
	return 0;
}


static const struct iio_trigger_ops optical_trigger_ops = {
    .owner = THIS_MODULE,
    .set_trigger_state = &optical_trig_set_state,
};

static int optical_trig_set_state(struct iio_trigger *trig, bool state) 
{
	//struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct iio_dev *indio_dev=dev_get_drvdata(&trig->dev);
	return optical_set_dataready_irq(indio_dev, state);
}


static int optical_buffer_preenable(struct iio_dev *indio_dev)
{
	int err;
	printk (KERN_ALERT "[%s]\n", __FUNCTION__);  
	err = optical_set_enable(indio_dev, true);
	if (err < 0)
		goto st_press_set_enable_error;

//	err = iio_sw_buffer_preenable(indio_dev);
	err = iio_sw_ring_preenable(indio_dev);
st_press_set_enable_error:
	return err;
}

static int optical_buffer_postenable(struct iio_dev *indio_dev)
{
	int err;
	struct optical_data *adata = iio_priv(indio_dev);
	//struct optical_data *adata= indio_dev->dev_data;
	printk (KERN_ALERT "[%s]\n", __FUNCTION__);  
	//adata->buffer_data = kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
	adata->buffer_data = kmalloc(indio_dev->ring->scan_count, GFP_KERNEL);
	if (adata->buffer_data == NULL) 
	{
		err = -ENOMEM;
		goto allocate_memory_error;
	}

	//	err = iio_triggered_buffer_postenable(indio_dev);
	err = iio_triggered_ring_postenable(indio_dev);
	if (err < 0)
	{
		printk (KERN_ALERT "[iio_triggered_buffer_postenable err=%d]\n", err);  
		goto st_press_buffer_postenable_error;
	}
	return err;

st_press_buffer_postenable_error:
	kfree(adata->buffer_data);
allocate_memory_error:
	return err;
}

static int optical_buffer_predisable(struct iio_dev *indio_dev)
{
	int err;
	//struct optical_data *adata = iio_priv(indio_dev);
	struct optical_data *adata= indio_dev->dev_data;
	printk (KERN_ALERT "[%s]\n", __FUNCTION__);  
	//err = iio_triggered_buffer_predisable(indio_dev);
	err = iio_triggered_ring_predisable(indio_dev);
	if (err < 0)
		goto st_press_buffer_predisable_error;

	err = optical_set_enable(indio_dev, false);

st_press_buffer_predisable_error:
	kfree(adata->buffer_data);
	return err;
}

/*static const struct iio_buffer_setup_ops optical_buffer_setup_ops = {
	.preenable = &optical_buffer_preenable,
	.postenable = &optical_buffer_postenable,
	.predisable = &optical_buffer_predisable,
};*/

static const struct iio_ring_setup_ops optical_buffer_setup_ops = {
	.preenable = &optical_buffer_preenable,
	.postenable = &optical_buffer_postenable,
	.predisable = &optical_buffer_predisable,
};
static int optical_allocate_ring(struct iio_dev *indio_dev)  //VungGV?
{
	/*int ret;
	printk (KERN_ALERT "[%s]\n", __FUNCTION__); 
	ret = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
		&optical_trigger_handler, &optical_buffer_setup_ops);
	return ret;*/

	int ret = 0;
		struct iio_ring_buffer *ring;

		ring = iio_sw_rb_allocate(indio_dev);
		if (!ring) {
			ret = -ENOMEM;
			return ret;
		}
		indio_dev->ring = ring;
		/* Effectively select the ring buffer implementation */
		ring->access = &ring_sw_access_funcs;
		ring->bpe = 2;
		ring->scan_timestamp = true;
		ring->setup_ops = &optical_buffer_setup_ops;
		ring->owner = THIS_MODULE;

		/* Set default scan mode */
		iio_scan_mask_set(ring, L1PH03_SCAN_PROX_X);
		iio_scan_mask_set(ring, L1PH03_SCAN_LIGHT_X);
		//iio_scan_mask_set(ring, ADIS16260_SCAN_AUX_ADC);
		//iio_scan_mask_set(ring, ADIS16260_SCAN_TEMP);
		//iio_scan_mask_set(ring, ADIS16260_SCAN_ANGL);
		
		indio_dev->pollfunc = iio_alloc_pollfunc(&iio_pollfunc_store_time,
							 &optical_trigger_handler,
							 IRQF_ONESHOT,
							 indio_dev,
							 "lt1ph03_consumer%d",
							 indio_dev->id);
		if (indio_dev->pollfunc == NULL) {
			ret = -ENOMEM;
			goto error_iio_sw_rb_free;
		}

		indio_dev->modes |= INDIO_RING_TRIGGERED;
		return 0;

	error_iio_sw_rb_free:
		iio_sw_rb_free(indio_dev->ring);
		return ret;


}

static void optical_deallocate_ring(struct iio_dev *indio_dev)
{
	//iio_triggered_buffer_cleanup(indio_dev);
		iio_dealloc_pollfunc(indio_dev->pollfunc);
		iio_sw_rb_free(indio_dev->ring);
}
/*******************buffer functions end***********************/
#ifdef OPTICAL_SENSOR_DEBUG
static struct optical_data *gsdata;

/******************************************************************************
NAME			:	optical_HR_Average
DESCRIPTION		:	Digital filter (Low pass filter)
INPUTS			:	u16 data	-- Pulse wave
			:	u16 sample_rate	-- Sample rate
RETURN VALUE TYPE	:	u32
******************************************************************************/

static u32 optical_HR_Average(u16 data, u16 sample_rate)
{

	static u16 udata[32] = {0};
	static u32 udata_sum = 0;
	static u16 count=0;
	u16 count_max = sample_rate >> 4;

	udata_sum = udata_sum - udata[count] + data;
	udata[count] = data;

	//counter
	if(count>=count_max){
		count = 0;
	}else{
		count++;
	}

	return udata_sum / count_max;

}



/******************************************************************************
NAME			:	LT_1PHXX_PS_CalcAPW
DESCRIPTION		:	Calculate acceleration of pulse wave
INPUTS			:	u32 data	-- Pulse wave
			:	u16 num		-- Point thinning
			:	s32 *pdata	-- Acceleration of pulse wave
RETURN VALUE TYPE	:	void
******************************************************************************/

static void optical_HR_CalcAPW(u32 data, u16 num, s32 *pdata)
{

	static u16 i=0;
	static s32 sdata[256] = {0};

	sdata[i] = data;

	if(i==255){
		*pdata = sdata[i-(2*num)] + sdata[i] - 2*sdata[i-num];
		i = 0;
	}else if(i<num){
		*pdata = sdata[255+(i-(2*num))] + sdata[i] - 2*sdata[255+(i-num)];
		i = i + 1;
	}else if(i>=num && i<2*num){
		*pdata = sdata[255+(i-(2*num))] + sdata[i] - 2*sdata[i-num];
		i = i + 1;
	}else{
		*pdata = sdata[i-(2*num)] +sdata[i] - 2*sdata[i-num];
		i = i + 1;
	}

}


/******************************************************************************
NAME			:	LT_1PHXX_PS_DetectPeak
DESCRIPTION		:	Detect peak of acceleration of pulse wave
INPUTS			:	s32 data	-- Acceleration of pulse wave
			:	u16 sample_rate	-- Sample rate
			:	s32 threshold	-- Threshold
			:	u16 *heart_rate	-- Heart rate
RETURN VALUE TYPE	:	void
******************************************************************************/

static u8 optical_HR_DetectPeak(s32 data, u16 sample_rate, s32 threshold, u16 *heart_rate)
{

	static u32 time;
	static s32 min;
	static u32 min_time;
	static u32 min_time_old;
	static s32 max;
	static u32 max_time;
	static u32 max_time_old;

	time = time + 1;

	if(threshold < 0)
	{

		if (min > data)
		{
			min = data;
			min_time = time;
		}

		if (data > 0 && min < threshold)
		{
			*heart_rate = 60 * sample_rate / (min_time - min_time_old);
			min_time_old = min_time;
			min = 0;
			return 1;
		}


	}else{

		if (max < data)
		{
			max = data;
			max_time = time;
		}

		if (data < 0 && max > threshold)
		{
			*heart_rate = 60 * sample_rate / (max_time - max_time_old);
			max_time_old = max_time;
			max = 0;
			return 1;
		}


	}
	return 0;

}
#define REDUCED_PROX_RATE			5//Hz
static irqreturn_t optical_trigger_handler_debug(int irq, void *p)
{

#if 1
#define BAR_LEN		(40)
	u8 buf[BAR_LEN+2];
	int proxData;
#ifdef REDUCED_PROX_RATE	
	static u8 numOfData=0;
	static u32 heartRateSum=0;
#endif
	gsdata->tf->read_byte(&gsdata->tb, gsdata->dev,	REGS_PS1_DATA0, buf);
	gsdata->tf->read_byte(&gsdata->tb, gsdata->dev,	REGS_PS1_DATA1, buf+1);
	gsdata->tf->read_byte(&gsdata->tb, gsdata->dev,	REGS_IRQ_STATUS, buf+2);
	proxData = get_unaligned_le16(buf);	
	if(buf[2]&BIT(2))
	{
		proxVal = (u8)proxData;			
		avgData = optical_HR_Average(proxData, 100);
		optical_HR_CalcAPW(avgData, 32 , &accData);
		optical_HR_DetectPeak(accData, 100, -65, &heart_rate);
#ifdef REDUCED_PROX_RATE
		heartRateSum = heartRateSum + (u32)heart_rate;
		numOfData = (numOfData+1)%(100/REDUCED_PROX_RATE);//orignal prox rate is 100Hz
		if(numOfData==0)
		{	
			heart_rate = (u16)(heartRateSum/(100/REDUCED_PROX_RATE));
			heartRateSum = 0;
			proxValValid = true;
			wake_up(&proxWait);
			hrValValid = true;
			wake_up(&hrWait);
			printk("vung  optical_trigger_handler_debug --------------");
		}
#else
		hrValValid = true;
		wake_up(&hrWait);
		proxValValid = true;
		wake_up(&proxWait);
#endif
		//printk (KERN_ALERT "proxData=0x%04x,heartRate=%4d,avg=%4d,acc=%4d",proxData,heart_rate,avgData,accData);
		//sysfs_notify(&gsdata->dev->kobj,NULL, "prox_val");
	}
	if(buf[2]&(BIT(5)))
	{
		gsdata->tf->read_byte(&gsdata->tb, gsdata->dev,	REGS_RESPONSE, buf);
		printk (KERN_ALERT "CMD INT: REGS_RESPONSE=0x%x\n",buf[0]);
		//sysfs_notify(&gsdata->dev->kobj,NULL, "light_val");
	}
	/*
	if(buf[2]&(BIT(5)))
	{
		lightVal = (u16)alsData;
		lightValValid = true;
		wake_up(&lightWait);
		//sysfs_notify(&gsdata->dev->kobj,NULL, "light_val");
	}
	*/
#if 0
	int i;
	proxData = BAR_LEN*proxData/2000;

	for(i=0;((i<proxData)&&(i<BAR_LEN));i++)
	{
		buf[i]='=';
	}
	for(;i<BAR_LEN;i++)
	{
		buf[i]=' ';
	}
	buf[i] = '\0';
	printk (KERN_ALERT "PROX: %s\n",buf);
#endif
#else
	u8 i = 16;
	u8 regs[16] = {0};
	printk (KERN_ALERT "[%s %d]\n", __FUNCTION__,irq);  
	
	for(i=0;i<16;i++)
	{
		gsdata->tf->read_byte(&gsdata->tb, gsdata->dev,	i, regs+i);
	}
	for(i=0;i<16;i++)
		printk (KERN_ALERT "optical regs[%d]=%02x\n",i,regs[i]); 

#endif

	gsdata->tf->write_byte(&gsdata->tb, gsdata->dev, REGS_IRQ_STATUS, 0x27);//clear all
	return IRQ_HANDLED;

}

#endif
static int optical_allocate_trigger(struct iio_dev *indio_dev,
				const struct iio_trigger_ops *trigger_ops)
{
	int err = 0;
	struct optical_data *sdata = iio_priv(indio_dev);
	printk (KERN_ALERT "[%s]\n", __FUNCTION__); 
#ifdef OPTICAL_SENSOR_DEBUG
	gsdata = sdata;
#endif
	//sdata->trig = iio_trigger_alloc("%s-trigger", indio_dev->name);
	sdata->trig = iio_allocate_trigger("%s-trigger", indio_dev->name);
	if (NULL != sdata->trig)
	{
	    err = request_threaded_irq(sdata->get_irq_data_ready(indio_dev), //allocate an interrupt line VungGV
#ifdef OPTICAL_SENSOR_DEBUG
				NULL,
				optical_trigger_handler_debug,
#else
			    iio_trigger_generic_data_rdy_poll,
			    NULL,
#endif	    
			    IRQF_TRIGGER_FALLING, //trong interrupt.h
			    sdata->trig->name,
			    sdata->trig);

	    if (!err)
	    {
	       // iio_trigger_set_drvdata(sdata->trig, indio_dev); //vungGV
		dev_set_drvdata(&(sdata->trig)->dev, indio_dev);
	        sdata->trig->ops = trigger_ops;
	        sdata->trig->dev.parent = sdata->dev;

	        err = iio_trigger_register(sdata->trig);
	        if (0 <= err)
	        {
	            indio_dev->trig = sdata->trig;
	            return 0;
	        }
	        else
	        {
	            dev_err(&indio_dev->dev, "failed to register iio trigger.\n");
            }
	        free_irq(sdata->get_irq_data_ready(indio_dev), sdata->trig);
	    }
	    else
	    {
    		dev_err(&indio_dev->dev, "failed to register iio thread.\n");
        }
       // iio_trigger_free(sdata->trig);
	    iio_free_trigger(sdata->trig);
   	}
	else
	{
		err = -ENOMEM;
		dev_err(&indio_dev->dev, "failed to allocate iio trigger.\n");
	}

	return err;
}

static void optical_deallocate_trigger(struct iio_dev *indio_dev)
{
	struct optical_data *sdata = iio_priv(indio_dev);
	iio_trigger_unregister(sdata->trig);
	free_irq(sdata->get_irq_data_ready(indio_dev), sdata->trig);
//	iio_trigger_free(sdata->trig);
	iio_free_trigger(sdata->trig);
}

static int optical_common_probe(struct iio_dev *indio_dev)
{
    int err = 0;
   struct optical_data *adata = iio_priv(indio_dev);
	printk (KERN_ALERT "[%s]\n", __FUNCTION__);  
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->info = &optical_info;
#ifdef OPTICAL_SENSOR_DEBUG
	init_waitqueue_head(&proxWait);
	init_waitqueue_head(&lightWait);
	init_waitqueue_head(&hrWait);
#endif
	err = optical_check_device_support(indio_dev,
                ARRAY_SIZE(optical_sensors), optical_sensors);
    if (err < 0)
    {
        goto st_press_common_probe_error;
    }
    adata->multiread_bit = adata->sensor->multi_read_bit;
    indio_dev->channels = adata->sensor->ch;
    indio_dev->num_channels = 2;
    err = optical_init_sensor(indio_dev);
    if (err < 0)
    {
        goto st_press_common_probe_error;
    }
    if (adata->get_irq_data_ready(indio_dev) > 0)
    {
        err = optical_allocate_ring(indio_dev);
        if (err < 0)
        {	printk("vunggv error optical_allocate_ring-------");
            goto st_press_common_probe_error;
        }
        err = optical_allocate_trigger(indio_dev,&optical_trigger_ops);
        if (err < 0)
        {	printk("vunggv error optical_allocate_trigger------");
            goto st_press_probe_trigger_error;
        }

    }
    err = iio_device_register(indio_dev);
    if (err)
    {	printk("vunggv error iio_device_register------");
        goto st_press_device_register_error;
    }
	err = iio_ring_buffer_register_ex(indio_dev->ring, 0,
					  indio_dev->channels,
					  indio_dev->num_channels);
    if (err)
    {	printk("vunggv error iio_ring_register------");
        goto st_press_device_register_error;
    }
	printk (KERN_ALERT "[%s OK]\n", __FUNCTION__);  

#ifdef OPTICAL_SENSOR_DEBUG
	optical_set_enable(indio_dev, true);
#endif

    return err;

st_press_device_register_error:
    if (adata->get_irq_data_ready(indio_dev) > 0)
    {
      //  st_sensors_deallocate_trigger(indio_dev); //cho nay VungGV
    	//Vung them
    	iio_trigger_unregister(adata->trig);
    	free_irq(adata->get_irq_data_ready(indio_dev), adata->trig);
    	iio_free_trigger(adata->trig);
    }
st_press_probe_trigger_error:
    if (adata->get_irq_data_ready(indio_dev) > 0)
    {
        optical_deallocate_ring(indio_dev);
    }
st_press_common_probe_error:
    return err;
}
static void optical_common_remove(struct iio_dev *indio_dev)
{
    struct optical_data *adata = iio_priv(indio_dev);
    iio_device_unregister(indio_dev);
    if (adata->get_irq_data_ready(indio_dev) > 0) 
	{
        optical_deallocate_trigger(indio_dev);
        optical_deallocate_ring(indio_dev);
    }
    //iio_device_free(indio_dev);
    iio_free_device(indio_dev);
}
static unsigned int optical_i2c_get_irq(struct iio_dev *indio_dev)
{
	struct optical_data *sdata = iio_priv(indio_dev);
	printk("irq day ------------------------%d",to_i2c_client(sdata->dev)->irq);
	return to_i2c_client(sdata->dev)->irq;
}

static int optical_i2c_read_byte(struct optical_transfer_buffer *tb,
				struct device *dev, u8 reg_addr, u8 *res_byte)
{
	int err;

	err = i2c_smbus_read_byte_data(to_i2c_client(dev), reg_addr);
	if (err < 0)
		goto i2c_read_byte_error;

	*res_byte = err & 0xff;

i2c_read_byte_error:
	return err < 0 ? err : 0;
}

static int optical_i2c_read_multiple_byte(
		struct optical_transfer_buffer *tb, struct device *dev,
			u8 reg_addr, int len, u8 *data, bool multiread_bit)
{
	if (multiread_bit)
		reg_addr |= OPTICAL_I2C_MULTIREAD;

	return i2c_smbus_read_i2c_block_data(to_i2c_client(dev),
							reg_addr, len, data);
}

static int optical_i2c_write_byte(struct optical_transfer_buffer *tb,
				struct device *dev, u8 reg_addr, u8 data)
{
	return i2c_smbus_write_byte_data(to_i2c_client(dev), reg_addr, data);
}

static const struct optical_transfer_function optical_tf_i2c = {
	.read_byte = optical_i2c_read_byte,
	.write_byte = optical_i2c_write_byte,
	.read_multiple_byte = optical_i2c_read_multiple_byte,
};


static void optical_i2c_configure(struct iio_dev *indio_dev,
		struct i2c_client *client, struct optical_data *sdata)
{
	int rc = 0;
	i2c_set_clientdata(client, indio_dev);
	indio_dev->dev.parent = &client->dev;
	indio_dev->name = client->name;

    sdata->tf = &optical_tf_i2c;
    sdata->get_irq_data_ready = optical_i2c_get_irq;
    sdata->vdd_io = regulator_get(&client->dev, "vdd_io");

    if (IS_ERR(sdata->vdd_io))
     {
        rc = PTR_ERR(sdata->vdd_io);
        dev_err(&client->dev, "regulator get failed vdd_io, %d\n", rc);
        sdata->vdd_io = NULL;
    }

    // Get information regarding the analog power to the I2C chip
    sdata->vdd_ana = regulator_get(&client->dev, "vdd_ana");

    if (IS_ERR(sdata->vdd_ana))
    {
        rc = PTR_ERR(sdata->vdd_ana);
        dev_err(&client->dev, "regulator get failed vdd_ana, %d\n", rc);
        sdata->vdd_ana = NULL;
        return;
    }
}

static int optical_i2c_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{	

    struct iio_dev *indio_dev = NULL;
    struct optical_data *adata = NULL;
    int err = 0;
 	printk (KERN_ALERT "[%s]\n", __FUNCTION__);  
    //indio_dev = iio_device_alloc(sizeof(*adata));//vungGV
 	indio_dev=iio_allocate_device(sizeof(*adata));

    if (NULL != indio_dev)
    {
        adata = iio_priv(indio_dev);
    	//adata=indio_dev->dev_data;
    	printk("vungv optical_i2c_probe");
        adata->dev = &client->dev;
        optical_i2c_configure(indio_dev, client, adata);
        printk (KERN_ALERT "\t %s - Enabled VDD_IO", __FUNCTION__);
        if (adata->vdd_io)
        {
            err = regulator_enable(adata->vdd_io);//VungGV
            if (!err)
            {
                mdelay(3);
            }
            else
            {
                dev_err(&client->dev, "error enabling vdd_io, %d\n", err);
            }
        }

        printk (KERN_ALERT "\t %s - Enabled VDD_ANA", __FUNCTION__);
        if (adata->vdd_ana && !err)
        {
            err = regulator_enable(adata->vdd_ana);
            if (!err)
            {
                mdelay(3);
            }
            else
            {
                dev_err(&client->dev, "error enabling vdd_ana, %d\n", err);
            }
        }


        if (!err)
        {
            printk (KERN_ALERT "\t %s - Test PRESS Chip", __FUNCTION__);
            err = optical_common_probe(indio_dev);
            if (0 <= err)
            {
                return 0;
            }
        }
    }
    else
    {

        err = -ENOMEM;
    }


/*
    if (adata && adata->vdd_io)
    {
        regulator_disable(adata->vdd_io);
    }
*/
/*
    if (adata && adata->vdd_ana)
    {
        regulator_disable(adata->vdd_ana);
    }
*/    
    if (indio_dev)
    {
        //iio_device_free(indio_dev);
    	iio_free_device(indio_dev);
    }
 
    return err;
}

static int optical_i2c_remove(struct i2c_client *client)
{
	printk (KERN_ALERT "[%s] optical_i2c_remove\n", __FUNCTION__);
    optical_common_remove(i2c_get_clientdata(client));
    return 0;
}

static const struct i2c_device_id optical_id_table[] = {
    { OPTICAL_DEV_NAME},
    {},
};
MODULE_DEVICE_TABLE(i2c, optical_id_table);

/*
static struct of_device_id optical_match_table[] = {
    { .compatible = "sychip,lt1ph03", },
    { },
};
*/
//  .of_match_table = optical_match_table,
static struct i2c_driver optical_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "lt1ph03",
    },
    .probe = optical_i2c_probe,
    .remove = __devexit_p(optical_i2c_remove),
    .id_table = optical_id_table,
};

static int __init lt1ph03_init(void)
{
	return i2c_add_driver(&optical_driver);
}
static void __exit lt1ph03_exit(void)
{
	i2c_del_driver(&optical_driver);
}


module_init(lt1ph03_init);
module_exit(lt1ph03_exit);

MODULE_AUTHOR("hli@sychip.com.cn");
MODULE_DESCRIPTION("Murata lt1ph03 i2c driver");
MODULE_LICENSE("GPL v2");




