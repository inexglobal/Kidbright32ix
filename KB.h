#ifndef KIDBRIGHT_V15I_BOARD_H
#define KIDBRIGHT_V15I_BOARD_H
#include "esp_system.h"
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
// #include "pins_kb.h"
#include "vector"
#include "Adafruit_GFX.h"		  // DotMatrix display
#include "Adafruit_LEDBackpack.h" // DotMatrix display

#include "freeRTOS\freeRTOS.h"
#include "freeRTOS\task.h"

#include "driver/adc.h"
#include "time.h" // Library Internet time

// #include "Servo.h"
// #define SERVO_ALL 3

#define TONE_CHANNEL 1

#define MATRIX_ADDR 0x70
//============ Sensor LDR Definition ====================//
#define MAX_LDR_VALUE 3400 // calibated value
#define AVG_SAMPLE 20	   // 100 sample
#define SMOOTH_DELAY 1	   //  ms delay
//============ RTC MCP9741x Definition ====================//
#define MCP7941x_EEPROM_I2C_ADDR 0x57
#define MAC_LOCATION 0xF2 // Starts at 0xF0 but we are only interested in 6 bytes.
#define MCP7941x_RTC_I2C_ADDR 0x6F
#define RTC_LOCATION 0x00

//======== Accelerometer & Magnetic Sensor ===========//
// #include "LSM303AGR_ACC_Sensor.h"
// #include "LSM303AGR_MAG_Sensor.h"

#define ACC_RANGE_2G 2.0f
#define ACC_RANGE_4G 4.0f
#define ACC_RANGE_8G 8.0f
#define ACC_RANGE_16G 16.0f

#define DEV_I2C Wire1 // Or Wire
#define SHAKE_LEVEL_BASE 2000
#define FREE_FALL_LEVEL_BASE 300
//======== Analog Channel ===========//
#define IN1 1
#define IN2 2
#define IN3 3
#define IN4 4

#define sound tone

#define TYPE_PLOT 0
#define TYPE_UNPLOT 1
#define TYPE_TOGGLE 2
#define TYPE_BARGRAPH 3

#define TYPE_SHIFT_LEFT 0
#define TYPE_SHIFT_RIGHT 1
/*
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} LSM303AGR_raw;

typedef struct {
	LSM303AGR_raw raw;
	float x;
	float y;
	float z;
} LSM303AGR_sensor;

*/

const uint8_t font8x4[16 + 1][4] = {
	{0x00, 0x7c, 0x44, 0x7c}, // 0
	{0x00, 0x00, 0x7c, 0x00}, // 1
	{0x00, 0x5c, 0x54, 0x74}, // 2
	{0x00, 0x54, 0x54, 0x7c}, // 3
	{0x00, 0x70, 0x10, 0x7c}, // 4
	{0x00, 0x74, 0x54, 0x5c}, // 5
	{0x00, 0x7c, 0x54, 0x5c}, // 6
	{0x00, 0x40, 0x40, 0x7c}, // 7
	{0x00, 0x7c, 0x54, 0x7c}, // 8
	{0x00, 0x74, 0x54, 0x7c}, // 9
	{0x00, 0x3c, 0x50, 0x3c}, // A
	{0x00, 0x7c, 0x54, 0x38}, // B
	{0x00, 0x38, 0x44, 0x44}, // C
	{0x00, 0x7c, 0x44, 0x38}, // D
	{0x00, 0x7c, 0x54, 0x54}, // E
	{0x00, 0x7c, 0x50, 0x50}, // F
	{0x00, 0x10, 0x10, 0x10}  // -
};
//============== Image Data ================//
const uint8_t IMG_ARROW_LEFT[] =
	{0x00,
	 0x08,
	 0x10,
	 0x20,
	 0x7F,
	 0x20,
	 0x10,
	 0x08};
const uint8_t IMG_ARROW_RIGHT[] =
	{0x00,
	 0x08,
	 0x04,
	 0x02,
	 0x7F,
	 0x02,
	 0x04,
	 0x08};
const uint8_t IMG_ARROW_UP[] =
	{0x00,
	 0x08,
	 0x1C,
	 0x2A,
	 0x49,
	 0x08,
	 0x08,
	 0x08};
const uint8_t IMG_ARROW_DOWN[] =
	{0x00,
	 0x08,
	 0x08,
	 0x08,
	 0x49,
	 0x2A,
	 0x1C,
	 0x08};
/*
#include <stdint.h>

#define EXTERNAL_NUM_INTERRUPTS 16
#define NUM_DIGITAL_PINS        40
#define NUM_ANALOG_INPUTS       16

#define analogInputToDigitalPin(p)  (((p)<20)?(esp32_adc2gpio[(p)]):-1)
#define digitalPinToInterrupt(p)    (((p)<40)?(p):-1)
#define digitalPinHasPWM(p)         (p < 34)
/
 const uint8_t TX = 1;
 const uint8_t RX = 3;

const uint8_t SDA = 21;
const uint8_t SCL = 22;

 const uint8_t SS    = 5;
 const uint8_t MOSI  = 23;
 const uint8_t MISO  = 19;
 const uint8_t SCK   = 18;

 const uint8_t A0 = 36;
 const uint8_t A3 = 39;
 const uint8_t A4 = 32;
 const uint8_t A5 = 33;
 const uint8_t A6 = 34;
 const uint8_t A7 = 35;
 const uint8_t A10 = 4;
 const uint8_t A11 = 0;
 const uint8_t A12 = 2;
 const uint8_t A13 = 15;
 const uint8_t A14 = 13;
 const uint8_t A15 = 12;
 const uint8_t A16 = 14;
 const uint8_t A17 = 27;
 const uint8_t A18 = 25;
 const uint8_t A19 = 26;

 const uint8_t T0 = 4;
 const uint8_t T1 = 0;
 const uint8_t T2 = 2;
 const uint8_t T3 = 15;
 const uint8_t T4 = 13;
 const uint8_t T5 = 12;
 const uint8_t T6 = 14;
 const uint8_t T7 = 27;
 const uint8_t T8 = 33;
 const uint8_t T9 = 32;

 const uint8_t DAC1 = 25;
 const uint8_t DAC2 = 26;
*/

const uint8_t KB_LDR_PIN = 36;

const uint8_t KB_BUTTON1 = 16;
const uint8_t KB_BUTTON2 = 14;

const uint8_t KB_LED_BT = 17;
const uint8_t KB_LED_WIFI = 2;
const uint8_t KB_LED_NTP = 15;
const uint8_t KB_LED_IOT = 12;

const uint8_t KB_BUZZER = 13;

const uint8_t KB_INPUT1 = 32;
const uint8_t KB_INPUT2 = 33;
const uint8_t KB_INPUT3 = 34;
const uint8_t KB_INPUT4 = 35;

const uint8_t KB_OUTPUT1 = 26;
const uint8_t KB_OUTPUT2 = 27;
const uint8_t KB_USB = 25;

const uint8_t KB_SDA0 = 21;
const uint8_t KB_SCL0 = 22;

const uint8_t KB_SDA1 = 4;
const uint8_t KB_SCL1 = 5;

const uint8_t KB_GPIO18 = 18;
const uint8_t KB_GPIO19 = 19;
const uint8_t KB_GPIO23 = 23;

class KB
{
public:
	KB();
	//======== I/O ===========//
	void begin(void);
	bool S1();
	bool S2();
	void waitS1();
	void waitS2();
	bool in1();
	bool in2();
	bool in3();
	bool in4();
	int analog(int ch);
	void usbOUT(bool st);
	void usbToggle();
	void out1(bool dat);
	void out2(bool dat);
	void ledWIFI(bool state);
	void ledIOT(bool state);
	//======== Music ===========//
	void beep();
	void tone(unsigned int frequency, unsigned long duration = 0);
	void noTone();
	void song(std::vector<int> notes, int duration);
	//======== DotMatrix display ===========//
	bool getPixel(int16_t x, int16_t y);
	void display(double x1, double y1, int TYPE);
	void moveDisplay(int TYPE);
	void clearDisplay();
	void setBrightness(uint8_t b);
	void scroll(String text, int lapTime = 50);
	void scroll(long number, int lapTime = 50);
	void scroll(int number, int lapTime = 50);
	void scroll(double number, int lapTime = 50);
	void print(String text, int x = 0, int y = 0);
	void print(int number, int x = 0, int y = 0);
	void print(float number, int x = 0, int y = 0);
	void printNumber(double n, int base = 10);
	void printTime(unsigned int n1, unsigned int n2, bool colon);
	void drawImage(int x, int y, const uint8_t *img, uint16_t on = 1);
	void drawPixel(int16_t x, int16_t y, uint16_t color);
	void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
	void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
	void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
	void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
	void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
	void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	//======== LDR Sensor ===========//
	uint16_t light();
	//======== Temperature Sensor ===========//
	float temperature();
	//============ RTC MCP9741x ====================//
	byte getHour();
	byte getMinute();
	byte getSecond();
	byte getDay();
	byte getMonth();
	byte getYear();
	void scrollDate();
	void scrollTime();
	void scrollDateTime();
	void setDate(byte dayOfMonth, byte month, byte year);
	void setTime(byte hour, byte minute, byte second);
	byte getDayofWeek();
	//============ Internet Time from WIFI connected ====================//
	bool syncTime(char *ssid = "", char *password = "", unsigned int timeout = 10);
	/*
	//======== Accelerometer Sensor ===========//
	void setAccRange(float range);
	int32_t ax();
	int32_t ay();
	int32_t az();
	long aStrength();
	bool setShakeLevel(unsigned int level);
	bool isShake();
	bool isScreenUp();
	bool isScreenDown();
	bool isBoardUp();
	bool isBoardDown();
	bool isTiltLeft();
	bool isTiltRight();
	bool isFreeFall();
	int roll();
	int pitch();
	//======== Compass Sensor ======================//
	void cCalibrate();
	int heading();
	int32_t cx();
	int32_t cy();
	int32_t cz();
	long cStrength();
	*/
	//================= Servo ======================//
	// void servo(int ch,int pos);
	// void servoStop(int ch);
	//======== DotMatrix display ===========//
	Adafruit_8x16minimatrix matrix = Adafruit_8x16minimatrix();

protected:
	uint16_t readState;

	//======== Accelerometer & Magnetic Sensor ===========//
	// int compass_heading;
	// LSM303AGR_ACC_Sensor *Acc;
	// LSM303AGR_MAG_Sensor *Mag;
	unsigned long acc_shake_level = SHAKE_LEVEL_BASE;
	unsigned long acc_free_fall_level = FREE_FALL_LEVEL_BASE;
	// LSM303AGR_raw my_mag_min;
	// LSM303AGR_raw my_mag_max;
	//======== Servo ===========//
	// Servo Servo1;
	// Servo Servo2;

private:
	//======== DotMatrix display ===========//
	void displayBegin(void);
	//======== Accelerometer & Magnetic Sensor ===========//
	// int angleXY();
	// void set_mag_min_max(LSM303AGR_raw mag_min, LSM303AGR_raw mag_max);
	// void calibrate_mag(void);

	//============ RTC MCP9741x ====================//
	byte _second, _minute, _hour, _dayOfWeek, _dayOfMonth, _month, _year;
	byte decToBcd(byte val);
	byte bcdToDec(byte val);

	void getMacAddress(byte *mac_address);
	void unlockUniqueID();
	void writeMacAddress(byte *mac_address);
	void getDateTime(byte *second, byte *minute, byte *hour, byte *dayOfWeek, byte *dayOfMonth, byte *month, byte *year);
	void enableClock();
	void disableClock();
	void enableBattery();
	//============ Internet Time ====================//
	int getNetYear(void);
	int getNetMonth(void);
	int getNetDay(void);
	int getNetDayOfWeek(void);
	int getNetHour(void);
	int getNetMinute(void);
	int getNetSecond(void);
	void printLocalTime();

	void setSramByte(byte location, byte data);
	byte getSramByte(byte location);
	void setSramWord(byte location, int data);
	int getSramWord(byte location);
};

#endif
