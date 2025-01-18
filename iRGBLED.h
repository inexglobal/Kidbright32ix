
#include "esp_system.h"
#include <Arduino.h>
#include "Adafruit_Neopixel.h" // Neopixel display
class iRGBLED
{
public:
    //============ Neopixel (NO!!! It's called "iRGBLED") ====================//
    iRGBLED(int16_t pin, uint16_t num);
    ~iRGBLED();
    void begin();
    void iRGBLEDPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
    void iRGBLEDPixelColor(uint16_t n, uint32_t color);
    void iRGBLEDFill(uint8_t r, uint8_t g, uint8_t b);
    void iRGBLEDFill(uint32_t color);
    void iRGBLEDBrightness(uint8_t brightness);
    void iRGBLEDClear();
    static uint32_t iRGBColor(uint8_t r, uint8_t g, uint8_t b)
    {
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }
    uint32_t iRGBLEDColorWheel(uint8_t WheelPos);
    void iRGBLEDRainbow(uint16_t first_hue, int8_t reps, uint8_t saturation, uint8_t brightness, bool gammify);
    void iRGBLEDRainbowTime(uint32_t ms);
    void iRGBLEDRainbowCycleTime(uint32_t ms);
    Adafruit_NeoPixel neopixel = Adafruit_NeoPixel(iRGBLEDNum, iRGBLEDPin, NEO_GRB + NEO_KHZ800);

private:
    uint16_t iRGBLEDNum = 0;
    int16_t iRGBLEDPin = 0;
    bool isiRGBLEDBegun = false;
};