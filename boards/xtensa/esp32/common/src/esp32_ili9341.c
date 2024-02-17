/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_ili9341.c
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
#include <inttypes.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/lcd/ili9341.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/spi/spi.h>


#include <nuttx/i2c/i2c_master.h>
#include "esp32_board_i2c.h"
#include "esp32_i2c.h"

#include <arch/board/board.h>

#include "esp32_gpio.h"
#include "esp32_spi.h"
#include "hardware/esp32_gpio_sigmap.h"
#include "hardware/esp32_spi.h"

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

/* HBL 100523
#ifndef DISPLAY_RST
#  error "DISPLAY_RST must be defined in board.h!"
#endif
 */

#ifndef DISPLAY_DC
#  error "DISPLAY_DC must be defined in board.h!"
#endif
#ifndef DISPLAY_SPI
#  error "DISPLAY_SPI must be defined in board.h!"
#endif

#ifdef CONFIG_ESP32_LCD_OVERCLOCK
#  define ILI9341_SPI_MAXFREQUENCY 40*1000*1000
#else
#  define ILI9341_SPI_MAXFREQUENCY 10*1000*1000
#endif

#ifndef CONFIG_SPI_CMDDATA
#  error "The ILI9341 driver requires CONFIG_SPI_CMDATA in the configuration"
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

struct ili93414ws_lcd_s
{
	struct ili9341_lcd_s dev;
	struct spi_dev_s *spi;
	struct i2c_master_s *i2c;
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static void esp32_ili93414ws_select(struct ili9341_lcd_s *lcd);
static void esp32_ili93414ws_deselect(struct ili9341_lcd_s *lcd);
static int esp32_ili93414ws_backlight(struct ili9341_lcd_s *lcd,
		int level);
static int esp32_ili93414ws_sendcmd(struct ili9341_lcd_s *lcd,
		const uint8_t cmd);
static int esp32_ili93414ws_sendparam(struct ili9341_lcd_s *lcd,
		const uint8_t param);
static int esp32_ili93414ws_sendgram(struct ili9341_lcd_s *lcd,
		const uint16_t *wd, uint32_t nwords);
static int esp32_ili93414ws_recvparam(struct ili9341_lcd_s *lcd,
		uint8_t *param);
static int esp32_ili93414ws_recvgram(struct ili9341_lcd_s *lcd,
		uint16_t *wd, uint32_t nwords);
int writeRegister8(FAR struct i2c_master_s *i2c,FAR struct i2c_config_s *config,uint8_t reg, uint8_t data, uint8_t mask);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ili93414ws_lcd_s g_lcddev;
static struct lcd_dev_s *g_lcd = NULL;
#define ILI9341_SPI_FREQUENCY_16 16*1000*1000
#define ILI9341_SPI_FREQUENCY_40 40*1000*1000

/****************************************************************************
 * Private Functions
 ****************************************************************************/
int writeRegister8(FAR struct i2c_master_s *i2c,FAR struct i2c_config_s *config,uint8_t reg, uint8_t data, uint8_t mask)
{
	int ret=-1;
	FAR uint8_t tmp[2]={reg,data};
	if (mask)
	{
		ret=i2c_writeread(i2c,config,&reg,1,&tmp[1],1);
		if (ret<0) {
			lcderr("i2c_writeread failed: %d\n",ret);
			return ret;
		}
		tmp[1]=(tmp[1]&mask)|data;
	}
	return i2c_write(i2c,config,tmp,2);
}

int bitOn(FAR struct i2c_master_s *i2c,FAR struct i2c_config_s *config, uint8_t reg, uint8_t bit)
{
  return writeRegister8(i2c, config, reg, bit, ~0);
}

int bitOff(FAR struct i2c_master_s *i2c,FAR struct i2c_config_s *config, uint8_t reg, uint8_t bit)
{
  return writeRegister8(i2c, config, reg, 0, ~bit);
}

/****************************************************************************
 * Name: esp32_ili93414ws_select
 *
 * Description:
 *   Select the SPI, lock and reconfigure if necessary
 *
 * Input Parameters:
 *   lcd  - Reference to the public driver structure
 *
 ****************************************************************************/

static void esp32_ili93414ws_select(struct ili9341_lcd_s *lcd)
{
	struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

	SPI_LOCK(priv->spi, true);
	SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
}

/****************************************************************************
 * Name: esp32_ili93414ws_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   lcd  - Reference to the public driver structure
 *
 ****************************************************************************/

static void esp32_ili93414ws_deselect(struct ili9341_lcd_s *lcd)
{
	struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

	SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
	SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: esp32_ili93414ws_backlight
 *
 * Description:
 *   Set the backlight level of the connected display.
 *   NOTE: Currently this function either sets the brightness to the maximum
 *         level (level > 0) or turns the display off (level == 0). Although
 *         the ILI9341 chip provides an interface for configuring the
 *         backlight level via WRITE_DISPLAY_BRIGHTNESS (0x51), it depends on
 *         the actual circuit of the display device. Usually the backlight
 *         pins are hardwired to Vcc, making the backlight level setting
 *         effectless.
 *
 * Input Parameters:
 *   lcd   - Reference to the public driver structure
 *   level - Backlight level
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/
/*
static int esp32_ili93414ws_backlight(struct ili9341_lcd_s *lcd,
		int level)
{
	if (level > 0)
	{
		lcd->sendcmd(lcd, ILI9341_WRITE_CTRL_DISPLAY);
		lcd->sendparam(lcd, 0x24);
	}
	else
	{
		lcd->sendcmd(lcd, ILI9341_WRITE_CTRL_DISPLAY);
		lcd->sendparam(lcd, 0x0);
	}

	return OK;
}
*/

static int esp32_ili93414ws_backlight(struct ili9341_lcd_s *lcd,
		int level)
{
	struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;
	struct i2c_config_s config;
	int ret=-1;

	// Set up the I2C configuration
	config.frequency = ILI9341_AXP_I2C_FREQ;
	config.address   = ILI9341_AXP_I2C_ADDR;
	config.addrlen   = 7;

	if (level)
	{
		if (level > 4)
		{
			level = (level / 24) + 5;
		}
		// LDO3 enable
		ret=bitOn(priv->i2c,&config,0x12,0x08);
		if (ret<0) {
			lcderr("LDO3 enable: %d\n",ret);
		}
	}else
	{
		// LDO3 disable
		ret=bitOff(priv->i2c,&config,0x12,0x08);
		if (ret<0) {
			lcderr("LDO3 disable: %d\n",ret);
		}
	}
	// Brightness
	ret=writeRegister8(priv->i2c,&config,0x28,level,0xF0);
	if (ret<0) {
		lcderr("Brightness: %d\n",ret);
	}

	return OK;
}

/*
void setBrightness(std::uint8_t brightness) override
  {
    if (brightness)
    {
      if (brightness > 4)
      {
        brightness = (brightness / 24) + 5;
      }
      lgfx::i2c::bitOn(axp_i2c_port, axp_i2c_addr, 0x12, 0x08, axp_i2c_freq); // LDO3 enable
    }
    else
    {
      lgfx::i2c::bitOff(axp_i2c_port, axp_i2c_addr, 0x12, 0x08, axp_i2c_freq); // LDO3 disable
    }
    lgfx::i2c::writeRegister8(axp_i2c_port, axp_i2c_addr, 0x28, brightness, 0xF0, axp_i2c_freq);
  }
};
*/

/****************************************************************************
 * Name: esp32_ili93414ws_sendcmd
 *
 * Description:
 *   Send a command to the lcd driver.
 *
 * Input Parameters:
 *   lcd  - Reference to the ili9341_lcd_s driver structure
 *   cmd  - command to send
 *
 * Returned Value:
 *   On success - OK
 *
 ****************************************************************************/

static int esp32_ili93414ws_sendcmd(struct ili9341_lcd_s *lcd,
                                    const uint8_t cmd)
{
  struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

  lcdinfo("%02x\n", cmd);

  SPI_SETBITS(priv->spi, 8);

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(priv->spi, cmd);
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);

  return OK;
}

/****************************************************************************
 * Name: esp32_ili93414ws_sendparam
 *
 * Description:
 *   Send a parameter to the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - Parameter to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int esp32_ili93414ws_sendparam(struct ili9341_lcd_s *lcd,
                                      const uint8_t param)
{
  struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

  lcdinfo("param=%04x\n", param);

  SPI_SETBITS(priv->spi, 8);

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_SEND(priv->spi, param);

  return OK;
}

/****************************************************************************
 * Name: esp32_ili93414ws_sendgram
 *
 * Description:
 *   Send a number of pixel words to the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   wd     - Reference to the words to send
 *   nwords - Number of words to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int esp32_ili93414ws_sendgram(struct ili9341_lcd_s *lcd,
                                     const uint16_t *wd, uint32_t nwords)
{
  struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

  lcdinfo("lcd:%p, wd=%p, nwords=%" PRIu32 "\n", lcd, wd, nwords);

  SPI_SETBITS(priv->spi, 16);
  // HBL SPI_SNDBLOCK(priv->spi, wd, nwords);
  SPI_SNDBLOCK(priv->spi, wd, nwords);

  return OK;
}

/****************************************************************************
 * Name: esp32_ili93414ws_recvparam
 *
 * Description:
 *   Receive a parameter from the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - Reference to where parameter is received
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int esp32_ili93414ws_recvparam(struct ili9341_lcd_s *lcd,
                                      uint8_t *param)
{
  struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

  SPI_SETBITS(priv->spi, 8);

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);

  *param = (uint8_t)(SPI_SEND(priv->spi, (uintptr_t)param) & 0xff);

  return OK;
}

/****************************************************************************
 * Name: esp32_ili93414ws_recvgram
 *
 * Description:
 *   Receive pixel words from the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the public driver structure
 *   wd     - Reference to where the pixel words are received
 *   nwords - Number of pixel words to receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/
/*
_pin_level(pin_cs, true);
  bus->writeCommand(0, 8);
  bus->wait();
  _pin_level(pin_cs, false);
  bus->writeCommand(cmd, 8);
  if (dummy_read_bit) bus->writeData(0, dummy_read_bit);  // dummy read bit
  bus->beginRead();
  std::uint32_t res = bus->readData(32);
  bus->endTransaction();
  _pin_level(pin_cs, true);
  */
static int esp32_ili93414ws_recvgram(struct ili9341_lcd_s *lcd,
                                     uint16_t *wd, uint32_t nwords)
{
  struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

  lcdinfo("wd=%p, nwords=%" PRIu32 "\n", wd, nwords);

  SPI_SETBITS(priv->spi, 16);
  SPI_RECVBLOCK(priv->spi, wd, nwords);

  return OK;
};


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware. The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with
 *   the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
	struct ili93414ws_lcd_s *priv = &g_lcddev;
	struct spi_dev_s *spi;
	struct i2c_master_s *i2c;
	struct i2c_config_s config;
	FAR uint8_t wbuffer;
	FAR uint8_t rbuffer;
	int ret=-1;
	lcdinfo("Initializing LCD\n");

	if (g_lcd == NULL)
	{
		lcdinfo("esp32_spibus_initialize\n");
		/*
		 * SPI
		 */
		spi = esp32_spibus_initialize(DISPLAY_SPI);

		if (!spi)
		{
			lcderr("Failed to initialize SPI bus.\n");
			return -ENODEV;
		}

		priv->spi=spi;

		/* Initialize non-SPI GPIOs */
		lcdinfo("Initializing DISPLAY_DC\n");
		esp32_configgpio(DISPLAY_DC, OUTPUT_FUNCTION_3);
		esp32_gpio_matrix_out(DISPLAY_DC, SIG_GPIO_OUT_IDX, 0, 0);

		/*
		 * I2C
		 */
		i2c = esp32_i2cbus_initialize(ILI9341_I2C_NUM);
		if (i2c == NULL)
		{
			lcderr("ERROR: Failed to initialize I2C%d\n",ILI9341_I2C_NUM);
		}
		priv->i2c=i2c;

		// Set up the I2C configuration
		config.frequency = ILI9341_AXP_I2C_FREQ;
		config.address   = ILI9341_AXP_I2C_ADDR;
		config.addrlen   = 7;

		wbuffer=0x03;
		rbuffer=0x00;
		ret=i2c_writeread(i2c,&config,&wbuffer,1,&rbuffer,1);
		if (ret < 0){
			lcderr("i2c_read failed: %d\n", ret);
			return -ENODEV;
		}

		if (rbuffer!=0x03) // AXP192 found
		{
			lcderr("AXP192not found: %d\n", rbuffer);
			return -ENODEV;

		}

		lcdinfo("AXP192 found\n");
		// GPIO4 enable
		ret=writeRegister8(i2c,&config,0x95,0x84,0x72);
		if (ret<0) {
			lcderr("GPIO4 enable failed: %d\n",ret);
		}

		// set LDO2 3300mv - LCD PWR
		ret=writeRegister8(i2c,&config,0x28,0xF0,~0);
		if (ret<0) {
			lcderr("LCD PWR failed: %d\n",ret);
		}
		// LDO2 enable
		ret=writeRegister8(i2c,&config,0x12,0x04,~0);
		if (ret<0) {
			lcderr("LDO2 enable failed: %d\n",ret);
		}
		// GPIO4 enable
		ret=writeRegister8(i2c,&config,0x95,0x84,0x72);
		if (ret<0) {
			lcderr("GPIO4 enable failed: %d\n",ret);
		}
		// GPIO4 LOW (LCD RST)
		ret=writeRegister8(i2c,&config,0x96,0,~0x02);
		if (ret<0) {
			lcderr("LCD RST L failed: %d\n",ret);
		}
		// GPIO4 HIGH (LCD RST)
		ret=writeRegister8(i2c,&config,0x96,0x02,~0);
		if (ret<0) {
			lcderr("LCD RST H failed: %d\n",ret);
		}
		up_udelay(128); // AXP LCD

		/*
		 * Touch
		 */
		// GPIO1 OpenDrain
		ret=writeRegister8(i2c,&config,0x92,0,0xF8);
		if (ret<0) {
			lcderr("GPIO1 OpenDrain: %d\n",ret);
		}

		// GPIO1 LOW (TOUCH RST)
		ret=writeRegister8(i2c,&config,0x94,0,~0x02);
		if (ret<0) {
			lcderr("GPIO1 OpenDrain: %d\n",ret);
		}

		// GPIO1 HIGH (TOUCH RST)
		ret=writeRegister8(i2c,&config,0x94,0x02,~0);
		if (ret<0) {
			lcderr("GPIO1 OpenDrain: %d\n",ret);
		}

		/*
		 * Audio NS4168
		 */
		ret=writeRegister8(i2c,&config,0x94,0x04,~0);
		if (ret<0) {
			lcderr("NS4168 - AXP192 IO2: %d\n",ret);
		}

		/* Configure SPI */
		SPI_SETMODE(priv->spi, SPIDEV_MODE0);
		SPI_SETBITS(priv->spi, 8);
		SPI_HWFEATURES(priv->spi, 0);
		SPI_SETFREQUENCY(priv->spi,ILI9341_SPI_FREQUENCY_40);

		/* Initialize ILI9341 driver with necessary methods */
		priv->dev.select      = esp32_ili93414ws_select;
		priv->dev.deselect    = esp32_ili93414ws_deselect;
		priv->dev.sendcmd     = esp32_ili93414ws_sendcmd;
		priv->dev.sendparam   = esp32_ili93414ws_sendparam;
		priv->dev.recvparam   = esp32_ili93414ws_recvparam;
		priv->dev.sendgram    = esp32_ili93414ws_sendgram;
		priv->dev.recvgram    = esp32_ili93414ws_recvgram;
		priv->dev.backlight   = esp32_ili93414ws_backlight;

		g_lcd = ili9341_initialize(&priv->dev, 0);

		if (g_lcd != NULL)
		{
			/* Turn the LCD on at 100% power */
			g_lcd->setpower(g_lcd, CONFIG_LCD_MAXPOWER);

			esp32_ili93414ws_backlight(&priv->dev,200);//HBL
		}
	}

return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 *
 * Description:
 *   Return a reference to the LCD object for the specified LCD. This allows
 *   support for multiple LCD devices.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
	if (lcddev == 0)
	{
		return g_lcd;
	}

	return NULL;
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support.
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
	lcdinfo("Terminating LCD\n");


	if (g_lcd != NULL)
	{
		/* Turn the display off */

		g_lcd->setpower(g_lcd, 0);

		g_lcd = NULL;
	}
}
