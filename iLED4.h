#ifndef __iLED4_H__
#define __iLED4_H__

#include <Wire.h>
#if (ARDUINO >= 100)
#include <Arduino.h>
#define WireSend(x) Wire.write(x)
#define WireSend2(x,size_num) Wire.write(x,size_num)
#define WireReceive() Wire.read()
#else
#include <WProgram.h>
#define WireSend(x) Wire.send(x)
#define WireSend2(x,size_num) Wire.send(x,size_num)
#define WireReceive(x) Wire.receive(x)
#endif

#define iLED_BLINK_CMD 0x80		 ///< I2C register for BLINK setting
#define iLED_BLINK_DISPLAYON 0x01 ///< I2C value for steady on
#define iLED_BLINK_OFF 0			 ///< I2C value for steady off
#define iLED_BLINK_2HZ 1			 ///< I2C value for 2 Hz blink
#define iLED_BLINK_1HZ 2			 ///< I2C value for 1 Hz blink
#define iLED_BLINK_HALFHZ 3		 ///< I2C value for 0.5 Hz blink

#define iLED_CMD_BRIGHTNESS 0xE0 ///< I2C register for BRIGHTNESS setting

#define SEVENSEG_DIGITS 5 ///< # Digits in 7-seg displays, plus NUL end

class iLED4
{
private:
	uint8_t position; ///< Current print position, 0-3
	uint8_t iled_addr = 0;
	void writeColon(void);
public:
	// constructor
	iLED4(uint8_t _addr = 0x70);
	// destructor
	~iLED4();
	// method
	void begin();
	void setBrightness(uint8_t b);
	void blinkRate(uint8_t b);
	void clear(void);

	uint16_t displaybuffer[8]; ///< Raw display data
	// size_t write(uint8_t c);

	void writeDigitRaw(uint8_t x, uint8_t bitmask);
	void writeDigitNum(uint8_t x, uint8_t num, bool dot = false);
	void drawColon(bool state);
	void printNumber(long n, uint8_t base = DEC);
	void printFloat(double n, uint8_t fracDigits = 2, uint8_t base = DEC);
	void printError(void);
	void showDotPoint(uint8_t x, bool show);
	void writeDisplay(void);
	void turn_on() ;
	void turn_off() ;
};

#endif