#ifndef __iLED4_CPP__
#define __iLED4_CPP__

#include "iLED4.h"

static const uint8_t numbertable[] = {
	0x3F, /* 0 */
	0x06, /* 1 */
	0x5B, /* 2 */
	0x4F, /* 3 */
	0x66, /* 4 */
	0x6D, /* 5 */
	0x7D, /* 6 */
	0x07, /* 7 */
	0x7F, /* 8 */
	0x6F, /* 9 */
	0x77, /* a */
	0x7C, /* b */
	0x39, /* C */
	0x5E, /* d */
	0x79, /* E */
	0x71, /* F */
};

iLED4::iLED4(uint8_t _addr)
{
	iled_addr = _addr;
}
iLED4::~iLED4(){}
// --------------------------------------

// Method
void iLED4::setBrightness(uint8_t b)
{
	if (b > 15)
		b = 15;

	uint8_t cmd = iLED_CMD_BRIGHTNESS | b;
	Wire.begin(4, 5);
	Wire.beginTransmission(iled_addr);
	WireSend(cmd);
	Wire.endTransmission();
	writeDisplay();
}

void iLED4::blinkRate(uint8_t b)
{
	if (b > 3)
		b = 0; // turn off if not sure

	uint8_t cmd = iLED_BLINK_CMD | iLED_BLINK_DISPLAYON | (b << 1);
	Wire.begin(4, 5);
	Wire.beginTransmission(iled_addr);
	WireSend(cmd);
	Wire.endTransmission();
	writeDisplay();
}

void iLED4::begin()
{
	uint8_t cmd = 0x21;
	Wire.begin(4, 5);
	Wire.beginTransmission(iled_addr);
	WireSend(cmd);
	Wire.endTransmission();

	blinkRate(iLED_BLINK_OFF);

	setBrightness(15); // max brightness
	clear();
	writeDisplay();
}

void iLED4::writeDisplay(void)
{
	uint8_t data[1 + 16];
	data[0] = 0x00;
	uint8_t inx = 1;
	for (uint8_t i = 0; i < 8; i++)
	{
		data[inx++] = displaybuffer[i] & 0xFF;
		data[inx++] = displaybuffer[i] >> 8;
	}
	Wire.begin(4, 5);
	Wire.beginTransmission(iled_addr);
	WireSend2(data, 1 + 16);
	Wire.endTransmission();
}

void iLED4::clear(void)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		displaybuffer[i] = 0;
	}
	writeDisplay();
}

/*size_t iLED4::write(uint8_t c)
{

	uint8_t r = 0;

	if (c == '\n')
		position = 0;
	if (c == '\r')
		position = 0;

	if ((c >= '0') && (c <= '9'))
	{
		writeDigitNum(position, c - '0');
		r = 1;
	}

	position++;
	if (position == 2)
		position++;

	return r;
}*/

void iLED4::writeDigitRaw(uint8_t d, uint8_t bitmask)
{
	if (d > 4)
		return;
	displaybuffer[d] = bitmask;
	writeDisplay();
}

void iLED4::drawColon(bool state)
{
	if (state)
		displaybuffer[4] = 0x1;
	else
		displaybuffer[4] = 0;
	writeColon();
	writeDisplay();
}

void iLED4::writeColon(void)
{
	uint8_t buff[3] = {0x08, (uint8_t)(displaybuffer[4] & 0xFF), (uint8_t)(displaybuffer[4] >> 8)};
	Wire.begin(4, 5);
	Wire.beginTransmission(iled_addr);
	WireSend2(buff, 3);
	Wire.endTransmission();
	writeDisplay();
}

void iLED4::writeDigitNum(uint8_t d, uint8_t num, bool dot)
{
	if (d > 4)
		return;

	writeDigitRaw(d, numbertable[num] | (dot << 7));
	
	writeDisplay();
}

void iLED4::printNumber(long n, uint8_t base)
{
	printFloat(n, 0, base);
}

void iLED4::printFloat(double n, uint8_t fracDigits, uint8_t base)
{
	uint8_t numericDigits = 4; // available digits on display
	bool isNegative = false;   // true if the number is negative

	// is the number negative?
	if (n < 0)
	{
		isNegative = true; // need to draw sign later
		--numericDigits;   // the sign will take up one digit
		n *= -1;		   // pretend the number is positive
	}

	// calculate the factor required to shift all fractional digits
	// into the integer part of the number
	double toIntFactor = 1.0;
	for (int i = 0; i < fracDigits; ++i)
		toIntFactor *= base;

	// create integer containing digits to display by applying
	// shifting factor and rounding adjustment
	uint32_t displayNumber = n * toIntFactor + 0.5;

	// calculate upper bound on displayNumber given
	// available digits on display
	uint32_t tooBig = 1;
	for (int i = 0; i < numericDigits; ++i)
		tooBig *= base;

	// if displayNumber is too large, try fewer fractional digits
	while (displayNumber >= tooBig)
	{
		--fracDigits;
		toIntFactor /= base;
		displayNumber = n * toIntFactor + 0.5;
	}

	// did toIntFactor shift the decimal off the display?
	if (toIntFactor < 1)
	{
		printError();
	}
	else
	{
		// otherwise, display the number
		int8_t displayPos = 3;

		if (displayNumber) // if displayNumber is not 0
		{
			for (uint8_t i = 0; displayNumber || i <= fracDigits; ++i)
			{
				bool displayDecimal = (fracDigits != 0 && i == fracDigits);
				writeDigitNum(displayPos--, displayNumber % base, displayDecimal);
				/*if (displayPos == 2)
					writeDigitRaw(displayPos--, 0x00);*/
				displayNumber /= base;
			}
		}
		else
		{
			writeDigitNum(displayPos--, 0, false);
		}

		// display negative sign if negative
		if (isNegative)
			writeDigitRaw(displayPos--, 0x40);

		// clear remaining display positions
		while (displayPos >= 0)
			writeDigitRaw(displayPos--, 0x00);
	}
	
	writeDisplay();
}

void iLED4::printError(void)
{
	for (uint8_t i = 0; i < SEVENSEG_DIGITS; ++i)
	{
		writeDigitRaw(i, (i == 2 ? 0x00 : 0x40));
	}
	
	writeDisplay();
}

void iLED4::showDotPoint(uint8_t x, bool show)
{
	if (x > 4)
		return;
	if (x == 4)
	{
		drawColon(show);
		return;
	}

	if (show)
	{
		displaybuffer[x] |= (1 << 7);
	}
	else
	{
		displaybuffer[x] &= ~(1 << 7);
	}
	
	writeDisplay();
}

void iLED4::turn_on()
{
	uint8_t cmd = 0x81;
	Wire.begin(4, 5);
	Wire.beginTransmission(iled_addr);
	WireSend(cmd);
	Wire.endTransmission();
}

void iLED4::turn_off()
{
	uint8_t cmd = 0x80;
	Wire.begin(4, 5);
	Wire.beginTransmission(iled_addr);
	WireSend(cmd);
	Wire.endTransmission();
}

#endif