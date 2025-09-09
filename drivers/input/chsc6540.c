/****************************************************************************
 * drivers/input/chsc6540.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h> /* include/debug.h*/ 

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/input/touchscreen.h>
#include <nuttx/input/chsc6540.h>
#include <arch/board/board.h>


/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define CHSC6540_REG_TOUCHDATA 0x02
#define CHSC6540_REG_CHIPID    0xA7
#define CHSC6540_I2C_RETRIES   2

#define CHSC6540_EVENT_DOWN      0
#define CHSC6540_EVENT_UP        1
#define CHSC6540_EVENT_CONTACT   2
#define CHSC6540_EVENT_NONE      3


/****************************************************************************
 * Private Types
 ****************************************************************************/

struct chsc6540_dev_s
{
	/* I2C bus and address for device. */
	struct i2c_master_s *i2c;
	uint8_t addr;

	struct touch_lowerhalf_s lower;
	char buffer[SIZEOF_TOUCH_SAMPLE_S(2)];
};

/* Last event, last ID and last valid touch coordinates */

static uint8_t  last_event = 0xff;
static uint8_t  last_id    = 0xff;
static uint16_t last_x     = 0xffff;
static uint16_t last_y     = 0xffff;



/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int chsc6540_i2c_read(FAR struct chsc6540_dev_s *dev, uint8_t reg, uint8_t *buf, size_t buflen);
static int chsc6540_get_touch_data(FAR struct chsc6540_dev_s *dev, FAR void *buf);
static int chsc6540_isr_handler(int irq,FAR void *context, FAR void *arg);




/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: chsc6540_i2c_read
 *
 * Description:
 *   Read from I2C device.
 *
 ****************************************************************************/

static int chsc6540_i2c_read(FAR struct chsc6540_dev_s *dev,uint8_t reg,uint8_t *buf,size_t buflen)
{
	int ret = 0;
	int retries;
	struct i2c_msg_s msgv[2] =
	{
		{
			.frequency = CHSC6540_I2C_FREQ,
			.addr      = dev->addr,
			.flags     = 0,
			.buffer    = &reg,
			.length    = 1
		},
		{
			.frequency = CHSC6540_I2C_FREQ,
			.addr      = dev->addr,
			.flags     = I2C_M_READ,
			.buffer    = buf,
			.length    = buflen
		}
	};
	for (retries=0;retries<CHSC6540_I2C_RETRIES;retries++)
	{
		iwarn("I2C_TRANSFER\n");
		ret=I2C_TRANSFER(dev->i2c,msgv,2);
		if (ret == -ENXIO) 
		{
			/* -ENXIO is returned when getting NACK from response.
			 * Keep trying.
			 */
			iwarn("I2C NACK\n");
			continue;
		} else if (ret >= 0) 
		{
			/* Success! */
			return 0;
		} else 
		{
			/* Some other error. Try to reset I2C bus and keep trying. */
			iwarn("I2C error\n");
#ifdef CONFIG_I2C_RESET
			if (retries == CHSC6540_I2C_RETRIES - 1)
			{
				break;
			}

			ret = I2C_RESET(dev->i2c);
			if (ret < 0)
			{
				iinfo("I2C_RESET failed: %d\n", ret);
				return ret;
			}
#endif
		}
	}

	/* Failed to read sensor. */
	return ret;
}



/****************************************************************************
 * Name: chsc6540_get_touch_data
 *
 * Description:
 *   Read Touch Data over I2C.
 *   touch_sample_s
 *   	 int npoints : The number of touch points in point[]
 *   	 struct touch_point_s point[1] : Actual dimension is npoints
 *   touch_point_s
 *   	 uint8_t  id : Unique identifies contact; Same in all reports for the contact
 *   	 uint8_t  flags : See TOUCH_* definitions above
 *   	 	#define TOUCH_DOWN : A new touch contact is established
 *   	 	#define TOUCH_MOVE : Movement occurred with previously reported contact
 *   	 	#define TOUCH_UP  : The touch contact was lost
 *   	 	#define TOUCH_ID_VALID : Touch ID is certain
 *   	 	#define TOUCH_POS_VALID : Hardware provided a valid X/Y position
 *   	 	#define TOUCH_PRESSURE_VALID (1 << 5): Hardware provided a valid pressure
 *   	 	#define TOUCH_SIZE_VALID     (1 << 6) : Hardware provided a valid H/W contact size
 *   	 	#define TOUCH_GESTURE_VALID  (1 << 7) : Hardware provided a valid gesture
 *   	 int16_t  x : Coordinate of the touch point (uncalibrated)
 *   	 int16_t  y : Y coordinate of the touch point (uncalibrated)
 *   	 int16_t  h : Height of touch point (uncalibrated)
 *   	 int16_t  w : Width of touch point (uncalibrated)
 *   	 uint16_t gesture : Gesture of touchscreen contact
 *   	 uint16_t pressure : Touch pressure
 *   	 uint64_t timestamp : Touch event time stamp, in microseconds
 *   event
 *   	 0 = Touch Down
 *   	 1 = Touch Up
 *   	 2 = Contact
 *   p0f
 *   	The chip keeps track of fingers, each touch has a finger ID of 0
 *   	or 1. So when there are two touches in point[0] and point[1] and then one
 *   	goes away, point0finger allows you to see which touch is left in
 *   	point[0].
 ****************************************************************************/

static int chsc6540_get_touch_data(FAR struct chsc6540_dev_s *dev, FAR void *buf)
{
	char buffer[SIZEOF_TOUCH_SAMPLE_S(2)];
	struct touch_sample_s *data;
	uint8_t readbuf[11];
	uint8_t pts_nb_u8 = 0; // The detected point number, 1-2 is valid.
	int ret;
	bool valid;

	valid=true;

	data=(FAR struct touch_sample_s *)buffer;
	memset(data,0,SIZEOF_TOUCH_SAMPLE_S(2));
	data->npoints=2;
	data->point[0].flags=CHSC6540_EVENT_UP;
	data->point[1].flags=CHSC6540_EVENT_UP;

	/* Read the raw touch data. */
	iinfo("\n");
	ret=chsc6540_i2c_read(dev,CHSC6540_REG_TOUCHDATA,readbuf,sizeof(readbuf));
	if (ret < 0)
	{
		iinfo("Read touch data failed\n");
		return ret;
	}

	/* Interpret the raw touch data. */
	pts_nb_u8=readbuf[0];
	if (pts_nb_u8>2) return -EINVAL;
	if (pts_nb_u8) {
		// Read the data. Never mind trying to read the "weight" and
		// "size" properties or using the built-in gestures: they
		// are always set to zero.
		data->point[0].x=((readbuf[1]<<8)|readbuf[2])&0x0fff; // 1st touch X Position over 11 bits
		data->point[0].y=((readbuf[3]<<8)|readbuf[4])&0x0fff; // Touch X Position over 11 bits
		data->point[0].id=(readbuf[3]>>4)?1:0;// Touch ID of 1st Touch Point, this value is 0x0F when ID is invalid
		data->point[0].flags=readbuf[1]>>6; // 1st Event Flag - 00b: Press Down - 01b: LiftUp - 10b: Contact - 11b: No event
	}

	if ((data->point[0].x>=CHSC6540_W)||(data->point[0].y>=CHSC6540_H)) {
		iwarn("Invalid touch data: id=%d, touch=%d, x=%d, y=%d\n", data->point[0].id,pts_nb_u8,data->point[0].x,data->point[0].y);
		if (last_event==0xff)  /* Quit if we have no last valid coordinates. */
		{
			ierr("Can't return touch data: id=%d, touch=%d, x=%d, y=%d\n",data->point[0].id,pts_nb_u8,data->point[0].x,data->point[0].y);
			return -EINVAL;
		}
		data->point[0].flags=CHSC6540_EVENT_UP;  /* Change to Touch Up Event to prevent looping. */
		valid=false;
		data->point[0].id =last_id;
		data->point[0].x  =last_x;
		data->point[0].y  =last_y;
	}

	if (pts_nb_u8==2) {
		data->point[1].x=((readbuf[7]<<8)|readbuf[8])&0x0fff; // 2nd Touch X Position
		data->point[1].y=((readbuf[9]<<8)|readbuf[10])&0x0fff;// 2nd Touch Y Position
		data->point[1].id=(readbuf[9]>>4)?1:0; // 2nd Touch ID
		data->point[1].flags=readbuf[7]>>6; // 2nd Event Flag
	}

	if (data->point[1].x>=CHSC6540_W||data->point[1].y>=CHSC6540_H) {
		iwarn("Invalid touch data: id=%d, touch=%d, x=%d, y=%d\n",data->point[1].id,pts_nb_u8,data->point[1].x,data->point[1].y);
		if (last_event==0xff)  /* Quit if we have no last valid coordinates. */
		{
			ierr("Can't return touch data: id=%d, touch=%d, x=%d, y=%d\n",data->point[1].id,pts_nb_u8,data->point[1].x,data->point[1].y);
			return -EINVAL;
		}
		data->point[1].flags=CHSC6540_EVENT_UP;  /* Change to Touch Up Event to prevent looping. */
		valid=false;
		data->point[1].id =last_id;
		data->point[1].x  =last_x;
		data->point[1].y  =last_y;
	}

	/* Remember the last valid touch data. Keep only fisrt touch */
	last_event = data->point[0].flags;
	last_id    = data->point[0].id;
	last_x     = data->point[0].x;
	last_y     = data->point[0].y;

	/* Set the touch flags. */
	if (data->point[0].flags==CHSC6540_EVENT_DOWN)  /* Touch Down */
	{
		iinfo("DOWN: id=%d, pts=%d, x=%d, y=%d\n",data->point[0].id, pts_nb_u8,data->point[0].x,data->point[0].y);
		if (valid){
			data->point[0].flags=TOUCH_DOWN|TOUCH_ID_VALID|TOUCH_POS_VALID;
		}
		else {
			data->point[0].flags=TOUCH_DOWN|TOUCH_ID_VALID;
		}
	} else if (data->point[0].flags==CHSC6540_EVENT_UP)  /* Touch Up */
	{
		iinfo("UP: id=%d, pts=%d, x=%d, y=%d\n",data->point[0].id,pts_nb_u8,data->point[0].x,data->point[0].y);
		if (valid){
			data->point[0].flags=TOUCH_UP|TOUCH_ID_VALID|TOUCH_POS_VALID;
		}
		else {
			data->point[0].flags=TOUCH_UP|TOUCH_ID_VALID;
		}
	} else if (data->point[0].flags==CHSC6540_EVENT_CONTACT)  /* Contact */
	{
		iinfo("CONTACT: id=%d, pts=%d, x=%d, y=%d\n",data->point[0].id, pts_nb_u8,data->point[0].x,data->point[0].y);
		if (valid)  /* Touch coordinates were valid. */
		{
			data->point[0].flags=TOUCH_MOVE|TOUCH_ID_VALID|TOUCH_POS_VALID;
		}
		else {
			data->point[0].flags=TOUCH_MOVE|TOUCH_ID_VALID;
		}
	} else  /* Reject Unknown Event */
	{
		iwarn("UNKNOWN EVENT: event=%d, id=%d, pts=%d, x=%d, y=%d\n",data->point[0].flags,data->point[0].id, pts_nb_u8,data->point[0].x,data->point[0].y);
		return -EINVAL;
	}

	/* Return the touch data. */
	data->npoints=1;
	memcpy(buf,data,SIZEOF_TOUCH_SAMPLE_S(1));

	iwarn("  id:      %d\n",   data->point[0].id);
	iwarn("  flags:   %02x\n", data->point[0].flags);
	iwarn("  x:       %d\n",   data->point[0].x);
	iwarn("  y:       %d\n",   data->point[0].y);

	return sizeof(data);
}





/****************************************************************************
 * Name: chsc6540_isr_handler
 *
 * Description:
 *   Handle GPIO Interrupt triggered by touch.
 *
 ****************************************************************************/
static int chsc6540_isr_handler(int irq,FAR void *context, FAR void *arg)
{
	FAR struct chsc6540_dev_s *priv=(FAR struct chsc6540_dev_s *)arg;

	chsc6540_get_touch_data(priv,priv->buffer);
	//	touch_event(priv->lower.priv,(FAR struct touch_sample_s *)priv->buffer);

	return 0;
}




/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: chsc6540_register
 *
 * Description:
 *   Register the chsc6540 device (e.g. /dev/input0).
 *   Config
 *   	xmin : 0
 *   	xmax : 319
 *   	ymin : 0
 *   	ymax : 239
 *
 ****************************************************************************/

int chsc6540_register(FAR const char *devpath,FAR struct i2c_master_s *i2c_dev,uint8_t i2c_devaddr,int16_t irq_i16)
{
	struct chsc6540_dev_s *priv;// Static create with kmm_zalloc
	int ret = 0;

	/*
	 * Allocate device private structure.
	 * Create once in heap, never destructed.
	 * Parameter of the handler
	 */
	iinfo("path=%s, addr=%d\n",devpath,i2c_devaddr);
	priv=kmm_zalloc(sizeof(struct chsc6540_dev_s));
	if (!priv)
	{
		ierr("Memory allocation failed\n");
		return -ENOMEM;
	}
	
	if (i2c_dev==NULL)
	{
		ierr("ERROR: i2c_dev NULL\n");
		return -ENOMEM;
	}

	/* Setup device structure. */
	memset((void*)priv,0,sizeof(struct chsc6540_dev_s));
	priv->addr=i2c_devaddr;
	priv->i2c=i2c_dev;
	priv->lower.maxpoint = CHSC6540_MAXPOINT; /* Maximal point supported by the touchscreen */

	/* 
	 * Attach GPIO interrupt handler. 
	 *    priv : will be the argument for handler
	 */
	ret=irq_attach(irq_i16,chsc6540_isr_handler,priv);
	if (ret<0)
	{
		kmm_free(priv);
		ierr("Attach interrupt failed\n");
		return -EIO;
	}

	/*
	 *   lower     - A pointer of lower half instance.
	 *   path      - The path of touchscreen device. such as "/dev/input0"
	 *   nums      - Number of the touch points structure.
	 */
	touch_register(&(priv->lower),devpath,CHSC6540_MAXPOINT);

	iinfo("Driver registered\n");
	return 0;
}

