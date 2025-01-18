#include "iRGBLED.h"
//================== Neopixel ===============================//
iRGBLED::~iRGBLED() {}
iRGBLED::iRGBLED(int16_t pin, uint16_t num)
{
    iRGBLEDNum = num;
    iRGBLEDPin = pin;
}
void iRGBLED::begin()
{
#if defined(_KB_H_)
    if (iRGBLEDPin == 0 ||
        (iRGBLEDPin != 32 && iRGBLEDPin != 33 && iRGBLEDPin != 34 &&
         iRGBLEDPin != 35 && iRGBLEDPin != 18 && iRGBLEDPin != 19 && iRGBLEDPin != 23
         && iRGBLEDPin != 26 && iRGBLEDPin != 27))
        return;
#endif
    neopixel.updateLength(iRGBLEDNum);
    neopixel.setPin(iRGBLEDPin);
    neopixel.begin();
    neopixel.clear();
    neopixel.show();
    isiRGBLEDBegun = true;
}
void iRGBLED::iRGBLEDPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
{
    if (!isiRGBLEDBegun)
        return;
    neopixel.setPixelColor(n, r, g, b);
    neopixel.show();
}
void iRGBLED::iRGBLEDPixelColor(uint16_t n, uint32_t color)
{
    if (!isiRGBLEDBegun)
        return;
    neopixel.setPixelColor(n, color);
    neopixel.show();
}
void iRGBLED::iRGBLEDFill(uint8_t r, uint8_t g, uint8_t b)
{
    if (!isiRGBLEDBegun)
        return;
    uint32_t result = iRGBColor(r, g, b);
    neopixel.fill(result, 0, iRGBLEDNum);
    neopixel.show();
}
void iRGBLED::iRGBLEDFill(uint32_t color)
{
    if (!isiRGBLEDBegun)
        return;
    neopixel.fill(color, 0, iRGBLEDNum);
    neopixel.show();
}
void iRGBLED::iRGBLEDBrightness(uint8_t brightness)
{
    if (!isiRGBLEDBegun)
        return;
    neopixel.setBrightness(brightness);
    neopixel.show();
}
void iRGBLED::iRGBLEDClear()
{
    if (!isiRGBLEDBegun)
        return;
    neopixel.clear();
    neopixel.show();
}
uint32_t iRGBLED::iRGBLEDColorWheel(uint8_t WheelPos)
{
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85)
    {
        return iRGBColor(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170)
    {
        WheelPos -= 85;
        return iRGBColor(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return iRGBColor(WheelPos * 3, 255 - WheelPos * 3, 0);
}
void iRGBLED::iRGBLEDRainbow(uint16_t first_hue, int8_t reps, uint8_t saturation, uint8_t brightness, bool gammify)
{
    neopixel.rainbow(first_hue, reps, saturation, brightness, gammify);
}
void iRGBLED::iRGBLEDRainbowTime(uint32_t ms)
{
    for (uint16_t j = 0; j < 256; j++)
    {
        for (uint16_t i = 0; i < iRGBLEDNum; i++)
        {
            neopixel.setPixelColor(i, iRGBLEDColorWheel((i + j) & 255));
        }
        neopixel.show();
        delay(ms);
    }
}
void iRGBLED::iRGBLEDRainbowCycleTime(uint32_t ms)
{
    for (uint16_t m = 0; m < 256 * 5; m++)
    { // 5 cycles of all colors on wheel
        for (uint16_t k = 0; k < iRGBLEDNum; k++)
        {
            neopixel.setPixelColor(k, iRGBLEDColorWheel(((k * 256 / iRGBLEDNum) + m) & 255));
        }
        neopixel.show();
        delay(ms);
    }
}