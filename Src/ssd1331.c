
#include "hal.h"
#include "ssd1331.h"

void ms_delay(unsigned ms)
{
	unsigned x;
	while (ms > 0) {
		x=2*5971;
		while (x > 0) {
			__asm("nop");
			x--;
		}
		ms--;
	}
}

void XSPI_SendData(uint8_t data)
{
	GPIO_SetBit(GPIO_PORT, CE0_GPIO, 0);

	if (SPI_SendData(SPI_PORT, (uint32_t)&data, 1)) {
        while (1);
    }

	GPIO_SetBit(GPIO_PORT, CE0_GPIO, 1);
}

void XSPI_SendDataSize(uint8_t *data, int size)
{
	GPIO_SetBit(GPIO_PORT, CE0_GPIO, 0);

	if (SPI_SendData(SPI_PORT, (uint32_t)data, size*2)) {
        while (1);
    }

    GPIO_SetBit(GPIO_PORT, CE0_GPIO, 1);
}

void sendCommand(uint8_t cmd)
{
	GPIO_SetBit(GPIO_PORT, DC_GPIO, 0);
	XSPI_SendData(cmd);
}

void sendData(uint8_t data)
{
	GPIO_SetBit(GPIO_PORT, DC_GPIO, 1);
	XSPI_SendData(data);
}

void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {

  uint8_t x1 = x;
  uint8_t y1 = y;
  if (x1 > 95) x1 = 95;
  if (y1 > 63) y1 = 63;

  uint8_t x2 = (x+w-1);
  uint8_t y2 = (y+h-1);
  if (x2 > 95) x2 = 95;
  if (y2 > 63) y2 = 63;

  if (x1 > x2) {
    uint8_t t = x2;
    x2 = x1;
    x1 = t;
  }
  if (y1 > y2) {
    uint8_t t = y2;
    y2 = y1;
    y1 = t;
  }

  sendCommand(0x15); // Column addr set
  sendCommand(x1);
  sendCommand(x2);

  sendCommand(0x75); // Column addr set
  sendCommand(y1);
  sendCommand(y2);
}

void SSD1331_Init(void) {
	GPIO_SetBit(GPIO_PORT, RS_GPIO, 1);
    ms_delay(10);
	GPIO_SetBit(GPIO_PORT, RS_GPIO, 0);
    ms_delay(10);
	GPIO_SetBit(GPIO_PORT, RS_GPIO, 1);
    ms_delay(10);

    // Initialization Sequence
    sendCommand(SSD1331_CMD_DISPLAYOFF);  	// 0xAE
   sendCommand(SSD1331_CMD_SETREMAP); 	// 0xA0
#if defined SSD1331_COLORORDER_RGB
    sendCommand(0x72);				// RGB Color
#else
    sendCommand(0x76);				// BGR Color
#endif
    sendCommand(SSD1331_CMD_STARTLINE); 	// 0xA1
    sendCommand(0x0);
    sendCommand(SSD1331_CMD_DISPLAYOFFSET); 	// 0xA2
    sendCommand(0x0);
    sendCommand(SSD1331_CMD_NORMALDISPLAY);  	// 0xA4
    sendCommand(SSD1331_CMD_SETMULTIPLEX); 	// 0xA8
    sendCommand(0x3F);  			// 0x3F 1/64 duty
    sendCommand(SSD1331_CMD_SETMASTER);  	// 0xAD
    sendCommand(0x8E);
    sendCommand(SSD1331_CMD_POWERMODE);  	// 0xB0
    sendCommand(0x0B);
    sendCommand(SSD1331_CMD_PRECHARGE);  	// 0xB1
    sendCommand(0x31);
    sendCommand(SSD1331_CMD_CLOCKDIV);  	// 0xB3
    sendCommand(0xF0);  // 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    sendCommand(SSD1331_CMD_PRECHARGEA);  	// 0x8A
    sendCommand(0x64);
    sendCommand(SSD1331_CMD_PRECHARGEB);  	// 0x8B
    sendCommand(0x78);
    sendCommand(SSD1331_CMD_PRECHARGEC);  	// 0x8C
    sendCommand(0x64);
    sendCommand(SSD1331_CMD_PRECHARGELEVEL);  	// 0xBB
    sendCommand(0x3A);
    sendCommand(SSD1331_CMD_VCOMH);  		// 0xBE
    sendCommand(0x3E);
    sendCommand(SSD1331_CMD_MASTERCURRENT);  	// 0x87
    sendCommand(0x06);
    sendCommand(SSD1331_CMD_CONTRASTA);  	// 0x81
    sendCommand(0x91);
    sendCommand(SSD1331_CMD_CONTRASTB);  	// 0x82
    sendCommand(0x50);
    sendCommand(SSD1331_CMD_CONTRASTC);  	// 0x83
    sendCommand(0x7D);
    sendCommand(SSD1331_CMD_DISPLAYON);	//--turn on oled panel

}

uint16_t Color16(uint8_t r, uint8_t g, uint8_t b)
{
	uint16_t color;

	color  = ((uint16_t)(r >> 3) & 0x1F) << 11;
	color |= ((uint16_t)(g >> 2) & 0x3F) << 5;
	color |= ((uint16_t)(b >> 3) & 0x1F);

	return color;
}

void SSD1331_DrawPixels(int x, int y, int w, int h, uint8_t *data) {
    setAddrWindow(x, y, w, h);

    GPIO_SetBit(GPIO_PORT, DC_GPIO, 1);

    // for (int i = 0; i < 2*w*h; i++)
    //     XSPI_SendData(data[i]);
    XSPI_SendDataSize(data, w*h);
}