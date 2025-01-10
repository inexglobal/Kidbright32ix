#include "KB.h"


//============ RTC MCP9741x Definition ====================//
#if (ARDUINO >= 100)
#include <Arduino.h>
#define WireSend(x) Wire.write(x)
#define WireReceive() Wire.read()
#else
#include <WProgram.h>
#define WireSend(x) Wire.send(x)
#define WireReceive(x) Wire.receive(x)
#endif
//============ Internet time Definition ====================//
#include "time.h"
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600*7;// GTM+7 for Bangkok Hanoi Jakata
const int   daylightOffset_sec = 0;
uint8_t buffDisplay[16]; // 16x8
uint8_t newbuffDisplay[16]; // 16x8
 //KB Kidbright = new KB();
KB::KB()
{
}
void KB::begin(void) {
    pinMode(KB_BUTTON1, INPUT_PULLUP);
    pinMode(KB_BUTTON2, INPUT_PULLUP);
	pinMode(KB_INPUT1, INPUT_PULLUP);
	pinMode(KB_INPUT2, INPUT_PULLUP);
	pinMode(KB_INPUT3, INPUT_PULLUP);
	pinMode(KB_INPUT4, INPUT_PULLUP);
    pinMode(KB_OUTPUT1, OUTPUT);
    pinMode(KB_OUTPUT2, OUTPUT);
    pinMode(KB_USB, OUTPUT);	// USB Output
	usbOUT(0);	// Initial USB OFF


    pinMode(KB_LED_BT, OUTPUT);
    pinMode(KB_LED_WIFI, OUTPUT);
    pinMode(KB_LED_NTP, OUTPUT);
    pinMode(KB_LED_IOT, OUTPUT);

    digitalWrite(KB_LED_BT, HIGH);
    digitalWrite(KB_LED_WIFI, HIGH);
    digitalWrite(KB_LED_NTP, HIGH);
    digitalWrite(KB_LED_IOT, HIGH);
	displayBegin();		// Initial for DotMatrix display
	analogReadResolution(12);
	//analogSetPinAttenuation(KB_LDR_PIN, ADC_0db);	// Initial for LDR Sensor 
	//Wire.begin(4, 5);	// Initial for Temperature Sensor LM73
	//Wire.begin();		// Initial for RTC 7941x
	//======== Accelerometer & Magnetic Sensor ===========//
	//DEV_I2C.begin(4, 5);
  /*
   //DEV_I2C.begin();
  // Initlialize components.
	Acc = new LSM303AGR_ACC_Sensor(&DEV_I2C);
	//Acc->SetFS(LSM303AGR_ACC_FS_2G);
	setAccRange(ACC_RANGE_16G);
	Acc->Enable();
	Acc->EnableTemperatureSensor();
	Mag = new LSM303AGR_MAG_Sensor(&DEV_I2C);
	Mag->Enable();
	// For calibrate compass sensor	
	int my_mag_x=abs(getSramWord(10)-getSramWord(13));
	if((my_mag_x>500)&&(my_mag_x<1000)){
	  my_mag_min = {getSramWord(10),getSramWord(11),getSramWord(12)};
      my_mag_max = {getSramWord(13),getSramWord(14),getSramWord(15)};
	}else{
	my_mag_min = {-567,-384,94};
    my_mag_max = {169,402,472};
  }
  */
  enableBattery();
  enableClock();
}
/*
void KB::setAccRange(float range)
{
	Acc->SetFS(range);
}

void KB::servo(int ch,int pos)
{
	if(ch==1)
	{
		if(!Servo1.attached())
			Servo1.attach(15);
		Servo1.write(pos);
	}
	else if(ch==2)
	{
		if(!Servo2.attached())
			Servo2.attach(17);
		Servo2.write(pos);
	}
}
void KB::servoStop(int ch)
{
	if(ch==1){
		Servo1.writeMicroseconds(-1);
	}
	else if(ch==2){
		Servo2.writeMicroseconds(-1);
	}
	else if(ch==3)
	{
		Servo1.writeMicroseconds(-1);
		Servo2.writeMicroseconds(-1);
	}
}
*/
bool KB::S1()
{
	return (digitalRead(KB_BUTTON1));
}
bool KB::S2()
{
	return (digitalRead(KB_BUTTON2));
}
bool KB::in1()
{
	return (digitalRead(KB_INPUT1));
}
bool KB::in2()
{
	return (digitalRead(KB_INPUT2));
}
bool KB::in3()
{
	return (digitalRead(KB_INPUT3));
}
bool KB::in4()
{
	return (digitalRead(KB_INPUT4));
}
int KB::analog(int ch)
{
	if(ch>=1 && ch<=4)
		return (analogRead((KB_INPUT1 -1)+ch));
	else
		return -1;	
}
void KB::waitS1()
{
	drawImage(0,0,IMG_ARROW_DOWN,1);
	while(digitalRead(KB_BUTTON1));
	while(!digitalRead(KB_BUTTON1));
	beep();
	drawImage(0,0,IMG_ARROW_DOWN,0);
}
void KB::waitS2()
{
	drawImage(8,0,IMG_ARROW_DOWN,1);
	while(digitalRead(KB_BUTTON2));
	while(!digitalRead(KB_BUTTON2));
	beep();
	drawImage(8,0,IMG_ARROW_DOWN,0);
}
void KB::usbOUT(bool st)
{
	digitalWrite(KB_USB,!st);
}
void KB::usbToggle()
{
	digitalWrite(KB_USB,!digitalRead(KB_USB));
}
void KB::out1(bool dat)
{
	digitalWrite(KB_OUTPUT1,dat);
}
void KB::out2(bool dat)
{
	digitalWrite(KB_OUTPUT2,dat);
}
void KB::ledWIFI(bool state) {
    digitalWrite(KB_LED_WIFI,!state);
}
void KB::ledIOT(bool state) {
    digitalWrite(KB_LED_IOT,!state);
}
//====================== Music ==========================//
void KB::tone(unsigned int frequency, unsigned long duration)
{
	 ledcAttach(KB_BUZZER,1000, 8);
	 ledcWrite(KB_BUZZER, frequency);
    //ledcAttachPin(KB_BUZZER, TONE_CHANNEL);
   // ledcWriteTone(TONE_CHANNEL, frequency);
    if (duration) {
        delay(duration);
        noTone();
    }
 
}
void KB::beep()
{
    tone(500,100);
}
void KB::noTone()
{
    //ledcDetachPin(KB_BUZZER);
    //ledcWrite(TONE_CHANNEL, 0);
	//ledcWrite(TONE_CHANNEL, 0);
	 ledcWrite(KB_BUZZER, 0);
}
void KB::song(std::vector<int>notes,int duration)
{
    for(int freq : notes){
        if(freq == -1){
            noTone();
            delay(duration);
        }else{
            tone(freq,duration);
        }
    }
}
//======== DotMatrix display ===========//
void KB::displayBegin(void) {
    Wire1.begin(21,22);
    matrix.begin(MATRIX_ADDR,&Wire1);
	matrix.setBrightness(2);
    matrix.setTextColor(LED_ON);
    matrix.setTextSize(1);
    matrix.setRotation(1);
    matrix.setTextWrap(false);
}
void KB::setBrightness(uint8_t b) {
	matrix.setBrightness(b);
}
void KB::clearDisplay() {
	matrix.clear();
    matrix.writeDisplay();
}
void KB::scroll(String text,int lapTime) {
  String msg = text;
  int16_t msg_len = (-1 * (msg.length() * 6));
  for (int16_t x = 16; x >= msg_len; x--) {
     matrix.clear();
     matrix.setCursor(x, 0);
     matrix.print(msg);
     matrix.writeDisplay();
     delay(lapTime);
  }
}
void KB::scroll(int number,int lapTime) {
  String msg = String(number);
  int16_t msg_len = (-1 * (msg.length() * 6));
  for (int16_t x = 16; x >= msg_len; x--) {
     matrix.clear();
     matrix.setCursor(x, 0);
     matrix.print(msg);
     matrix.writeDisplay();
     delay(lapTime);
  }
}
void KB::scroll(long number,int lapTime) {
  String msg = String(number);
  int16_t msg_len = (-1 * (msg.length() * 6));
  for (int16_t x = 16; x >= msg_len; x--) {
     matrix.clear();
     matrix.setCursor(x, 0);
     matrix.print(msg);
     matrix.writeDisplay();
     delay(lapTime);
  }
}
void KB::scroll(double number,int lapTime) {
  String msg = String(number,3);
  int16_t msg_len = (-1 * (msg.length() * 6));
  for (int16_t x = 16; x >= msg_len; x--) {
     matrix.clear();
     matrix.setCursor(x, 0);
     matrix.print(msg);
     matrix.writeDisplay();
     delay(lapTime);
  }
}
void KB::print(String text,int x, int y) {
	matrix.clear();
    matrix.setCursor(x, y);
    matrix.print(text);
    matrix.writeDisplay();
}
void KB::print(int number,int x, int y) {
    matrix.clear();
    matrix.setCursor(x, y);
    matrix.print(String(number));
    matrix.writeDisplay();
}
void KB::print(float number,int x, int y) {
    matrix.clear();
    matrix.setCursor(x, y);
    matrix.print(String(number));
    matrix.writeDisplay();
}
void KB::printNumber(double n,int base){
	memset(buffDisplay, 0, sizeof(buffDisplay));
	char strBuff[20];
	memset(strBuff, 0, 20);
	if (base == 10) {
		sprintf(strBuff, "%f", n);
		ESP_LOGI("iTerminal", "%s", strBuff);
		while(1) {
			int n = strlen(strBuff);
			if (strBuff[n - 1] == '0') {
				strBuff[n - 1] = 0;
			} else if (strBuff[n - 1] == '.') {
				strBuff[n - 1] = 0;
				break;
			} else {
				break;
			}
		}
		ESP_LOGI("iTerminal", "%s", strBuff);
	} else if (base == 16 || base == 2 || base == 8) {
		itoa(n, strBuff, base);
	}
	
	int len = strlen(strBuff);
	int nextIndex = 0;
	bool hasDot = false;
	for (int i=0;i<len;i++) {
		if (strBuff[i] == '.') {
			hasDot = true;
			break;
		}
	}

	if (len > (hasDot ? 5 : 4)) {
		// Substring
		char newStrBuff[20];
		strcpy(newStrBuff, strBuff);
		memcpy(strBuff, &newStrBuff[len - (hasDot ? 5 : 4)], (hasDot ? 5 : 4));
	}

	if (len < (hasDot ? 5 : 4)) {
		nextIndex = (hasDot ? 5 : 4) - len;
	}
	int i = 0;
	for (;;) {
		char c = strBuff[i];
		i++;
		if (c == '.' && nextIndex > 0) {
			buffDisplay[((nextIndex - 1) * 4) + 3] |= 0x04;
			continue;
		}
		int asciiToIndex = 0;
		if (c >= '0' && c <= '9') {
			asciiToIndex = c - '0';
		} else if (c >= 'a' && c <= 'f') {
			asciiToIndex = c - 'a' + 10;
		} else if (c == '-') {
			asciiToIndex = 16; // -
		}
		memcpy(&buffDisplay[(nextIndex * 4) + 0], font8x4[asciiToIndex], 4);
		nextIndex++;
		if (nextIndex == 4) {
			break;
		}
	}
	
	for(int i=0;i<16;i++){
		newbuffDisplay[i]=buffDisplay[15-i];
	}
	
    matrix.clear();
    matrix.setCursor(0, 0);
    matrix.setRotation(0);
	matrix.drawBitmap(0, 0,newbuffDisplay, 8, 16, 1);
    matrix.writeDisplay();
    //matrix.setRotation(1);   
}
void KB::printTime(unsigned int n1, unsigned int n2, bool colon){
	n1 = n1 % 100;
	n2 = n2 % 100;

	char n1str[5], n2str[5];

	memset(n1str, 0, 5);
	memset(n2str, 0, 5);

	itoa(n1, n1str, 10);
	itoa(n2, n2str, 10);

	if (n2 < 10) {
		n2str[1] = n2str[0];
		n2str[0] = '0';
	}

	memset(buffDisplay, 0, 16);

	if (n1 >= 10) {
		buffDisplay[0] = font8x4[n1str[0] - '0'][1];
		buffDisplay[1] = font8x4[n1str[0] - '0'][2];
		buffDisplay[2] = font8x4[n1str[0] - '0'][3];

		buffDisplay[3] = 0;
		buffDisplay[4] = font8x4[n1str[1] - '0'][1];
		buffDisplay[5] = font8x4[n1str[1] - '0'][2];
		buffDisplay[6] = font8x4[n1str[1] - '0'][3];
	} else {
		buffDisplay[4] = font8x4[n1str[0] - '0'][1];
		buffDisplay[5] = font8x4[n1str[0] - '0'][2];
		buffDisplay[6] = font8x4[n1str[0] - '0'][3];
	}

	// buffDisplay[6] = 0;
	buffDisplay[7] = colon ? 0x28 : 0;
	buffDisplay[8] = 0;

	buffDisplay[9] = font8x4[n2str[0] - '0'][1];
	buffDisplay[10] = font8x4[n2str[0] - '0'][2];
	buffDisplay[11] = font8x4[n2str[0] - '0'][3];

	buffDisplay[12] = 0;
	buffDisplay[13] = font8x4[n2str[1] - '0'][1];
	buffDisplay[14] = font8x4[n2str[1] - '0'][2];
	buffDisplay[15] = font8x4[n2str[1] - '0'][3];
	  for(int i=0;i<16;i++){
	   newbuffDisplay[i]=buffDisplay[15-i];
	  }
    matrix.clear();
    matrix.setCursor(0, 0);
    matrix.setRotation(0);
	matrix.drawBitmap(0, 0,newbuffDisplay, 8, 16, 1);
    matrix.writeDisplay();
}
void KB::drawImage(int x, int y,const uint8_t *img , uint16_t on){
    matrix.clear();
    matrix.setCursor(0, 0);
    matrix.setRotation(1);
	matrix.drawBitmap(x, y, img, 8, 16, on);
    matrix.writeDisplay();
    //matrix.setRotation(1);   
}

void KB::drawPixel(int16_t x, int16_t y, uint16_t color) {
    matrix.setRotation(1);
    matrix.drawPixel(x, y, color);
    matrix.writeDisplay();
}
void KB::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    matrix.setRotation(1);
    matrix.drawLine(x0, y0, x1, y1, color);
    matrix.writeDisplay();
}
void KB::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    matrix.setRotation(1);
    matrix.drawRect(x, y, w, h, color);
    matrix.writeDisplay();
}
void KB::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    matrix.setRotation(1);
    matrix.drawCircle(x0, y0, r, color);
    matrix.writeDisplay();
}
void KB::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    matrix.setRotation(1);
    matrix.fillCircle(x0, y0, r, color);
    matrix.writeDisplay();
}
void KB::drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
    matrix.setRotation(1);
    matrix.drawTriangle(x0, y0, x1, y1, x2, y2, color);
    matrix.writeDisplay();
}
void KB::fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,int16_t x2, int16_t y2, uint16_t color) {
    matrix.setRotation(1);
    matrix.fillTriangle(x0, y0, x1, y1,x2, y2, color);
    matrix.writeDisplay();
}
void KB::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    matrix.setRotation(1);
    matrix.fillRect(x, y, w, h, color);
    matrix.writeDisplay();
}
//================== LDR Sensor ===============================//
uint16_t KB::light() {
	/*
    uint32_t value = 0;
    for(int i=0; i < AVG_SAMPLE; i++){
        value += analogRead(KB_LDR_PIN);
        delay(SMOOTH_DELAY);
    }
    value = value / AVG_SAMPLE;
   // Serial.println(value);
   // if (value > MAX_LDR_VALUE) {
   //     value = MAX_LDR_VALUE;
   // }
    //uint16_t ldr = ((MAX_LDR_VALUE - value) * 100) / MAX_LDR_VALUE;
	*/
	uint16_t ldr = analogRead(KB_LDR_PIN);
    return ldr;
}
//================== Temperature Sensor ===============================//
float KB::temperature() {
    Wire.begin(4, 5);	// Initial for Temperature Sensor LM73
	Wire.beginTransmission(0x4D);
    Wire.write(0x00);
    Wire.endTransmission();

    uint8_t count = Wire.requestFrom(0x4D, 2);
    float temp = 0.0;
    if (count == 2) {
      int buff[2];
      buff[0] = Wire.read();
      buff[1] = Wire.read();
      temp += (int)(buff[0] << 1);
      if (buff[1] & 0b10000000) temp += 1.0;
      if (buff[1] & 0b01000000) temp += 0.5;
      if (buff[1] & 0b00100000) temp += 0.25;
      if (buff[0] & 0b10000000) temp *= -1.0;
    }
    return temp;
}
//================== RTC MCP9741x ===============================//
// Convert normal decimal numbers to binary coded decimal:
byte KB::decToBcd(byte val)
{
  return ((val / 10 * 16) + (val % 10));
}

// Convert binary coded decimal to normal decimal numbers:
byte KB::bcdToDec(byte val)
{
  return ((val / 16 * 10) + (val % 16));
}

// Function to read the mac address from the eeprom:
void KB::getMacAddress(byte *mac_address)
{
  Wire.begin(4, 5);
  Wire.beginTransmission(MCP7941x_EEPROM_I2C_ADDR);
  WireSend(MAC_LOCATION);
  Wire.endTransmission();

  Wire.requestFrom(MCP7941x_EEPROM_I2C_ADDR, 6);

  for (int i = 0; i < 6; i++)
  {
    mac_address[i] = WireReceive();
  }
}

// Unlock the unique id area ready for writing:
void KB::unlockUniqueID()
{
  Wire.begin(4, 5);
  // Write 0x55 to the memory location 0x09 and stop:
  Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
  WireSend(0x09);
  WireSend(0x55);
  Wire.endTransmission();

  // Write 0xAA to the memory location 0x09 and stop:
  Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
  WireSend(0x09);
  WireSend(0xAA);
  Wire.endTransmission();
}

// Unlock the unique id area and write in the mac address:
void KB::writeMacAddress(byte *mac_address)
{
  Wire.begin(4, 5);
  Wire.beginTransmission(MCP7941x_EEPROM_I2C_ADDR);
  WireSend(0xF2);

  for (int i = 0; i < 6; i++)
  {
    WireSend(mac_address[i]);
  }

  Wire.endTransmission();
}

// Set the date/time, set to 24hr and enable the clock:
// (assumes you're passing in valid numbers)
// Get the date/time:
void KB::getDateTime(
    byte *second,
    byte *minute,
    byte *hour,
    byte *dayOfWeek,
    byte *dayOfMonth,
    byte *month,
    byte *year)
{
  Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
  WireSend(RTC_LOCATION);
  Wire.endTransmission();

  Wire.requestFrom(MCP7941x_RTC_I2C_ADDR, 7);

  // A few of these need masks because certain bits are control bits
  *second = bcdToDec(WireReceive() & 0x7f);     // 01111111
  *minute = bcdToDec(WireReceive() & 0x7f);     // 01111111
  *hour = bcdToDec(WireReceive() & 0x3f);       // 00111111
  *dayOfWeek = bcdToDec(WireReceive() & 0x07);  // 01111111
  *dayOfMonth = bcdToDec(WireReceive() & 0x3f); // 00111111
  *month = bcdToDec(WireReceive() & 0x1f);      // 00011111
  *year = bcdToDec(WireReceive());              // 11111111
}

// Enable the clock without changing the date/time:
void KB::enableClock()
{
  // Get the current seconds value as the enable/disable bit is in the same
  // byte of memory as the seconds value:
  Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
  WireSend(RTC_LOCATION);
  Wire.endTransmission();

  Wire.requestFrom(MCP7941x_RTC_I2C_ADDR, 1);

  int second = bcdToDec(WireReceive() & 0x7f); // 01111111

  // Start Clock:
  Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
  WireSend(RTC_LOCATION);
  WireSend(decToBcd(second) | 0x80); // set seconds and enable clock (10000000)
  Wire.endTransmission();
}

// Disable the clock without changing the date/time:
void KB::disableClock()
{
  // Get the current seconds value as the enable/disable bit is in the same
  // byte of memory as the seconds value:
  Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
  WireSend(RTC_LOCATION);
  Wire.endTransmission();

  Wire.requestFrom(MCP7941x_RTC_I2C_ADDR, 1);

  int second = bcdToDec(WireReceive() & 0x7f); // 01111111

  // Start Clock:
  Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
  WireSend(RTC_LOCATION);
  WireSend(decToBcd(second)); // set seconds and disable clock (01111111)
  Wire.endTransmission();
}

// Enable the battery:
void KB::enableBattery()
{
  // Get the current seconds value as the enable/disable bit is in the same
  // byte of memory as the seconds value:
  Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
  WireSend(RTC_LOCATION + 0x03);
  Wire.endTransmission();

  Wire.requestFrom(MCP7941x_RTC_I2C_ADDR, 1);

  int day = bcdToDec(WireReceive() & 0x07); // 00000111

  // Start Clock:
  Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
  WireSend(RTC_LOCATION + 0x03);
  WireSend(decToBcd(day) | 0x08); // set day and enable battery (00001000)
  Wire.endTransmission();
}

// Store byte of data in SRAM:
void KB::setSramByte(byte location, byte data)
{
  if (location >= 0x20 && location <= 0x5f)
  {
    Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
    WireSend(location);
    WireSend(data);
    Wire.endTransmission();
  }
}

// Read byte of data from SRAM:
byte KB::getSramByte(byte location)
{
  if (location >= 0x20 && location <= 0x5f)
  {
    Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
    WireSend(location);
    Wire.endTransmission();

    Wire.requestFrom(MCP7941x_RTC_I2C_ADDR, 1);

    return WireReceive();
  }
}
// Store Word of data in SRAM:
void  KB::setSramWord ( byte location, int data )
{
 if (location >= 0x00 && location <= 0x20)
  {
	byte location_high = 0x20 + (location * 2);
    byte location_low = location_high + 1;
	
    byte data_high = (((uint16_t)data)>>8);
	byte data_low = (((uint16_t)data)&0x00ff);
  
	Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
	WireSend(location_high);
    WireSend(data_high);
    Wire.endTransmission();
	delay(10);
    Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
	WireSend(location_low);
    WireSend(data_low);
    Wire.endTransmission();
	delay(10);
  }
}
// Read word of data from SRAM:
int  KB::getSramWord ( byte location )
{
	
  if (location >= 0x00 && location <= 0x20)
  {
	byte location_high = 0x20 + (location * 2);
    byte location_low = location_high + 1;
	  
	byte data_high = 0x00;
	byte data_low = 0x00;
	
	Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
    WireSend(location_high);
    Wire.endTransmission();
    delay(10);
    Wire.requestFrom(MCP7941x_RTC_I2C_ADDR, 1);
	delay(10);
    data_high = WireReceive();
    delay(10);
	
	Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
    WireSend(location_low);
    Wire.endTransmission();
    delay(10);
    Wire.requestFrom(MCP7941x_RTC_I2C_ADDR, 1);
	delay(10);
    data_low = WireReceive();
    delay(10);
    return  ((int16_t)word(data_high,data_low));
  }
 
}
byte KB::getDayofWeek()
{
  Wire.begin(4, 5);
  getDateTime(&_second, &_minute, &_hour, &_dayOfWeek, &_dayOfMonth, &_month, &_year);
  return _dayOfWeek;
}

byte KB::getHour()
{
  Wire.begin(4, 5);
  getDateTime(&_second, &_minute, &_hour, &_dayOfWeek, &_dayOfMonth, &_month, &_year);
  return _hour;
}

byte KB::getMinute()
{
  Wire.begin(4, 5);
  getDateTime(&_second, &_minute, &_hour, &_dayOfWeek, &_dayOfMonth, &_month, &_year);
  return _minute;
}

byte KB::getSecond()
{
  Wire.begin(4, 5);
  getDateTime(&_second, &_minute, &_hour, &_dayOfWeek, &_dayOfMonth, &_month, &_year);
  return _second;
}

byte KB::getDay()
{
  Wire.begin(4, 5);
  getDateTime(&_second, &_minute, &_hour, &_dayOfWeek, &_dayOfMonth, &_month, &_year);
  return _dayOfMonth;
}

byte KB::getMonth()
{
  Wire.begin(4, 5);
  getDateTime(&_second, &_minute, &_hour, &_dayOfWeek, &_dayOfMonth, &_month, &_year);
  return _month;
}

byte KB::getYear()
{
  Wire.begin(4, 5);
  getDateTime(&_second, &_minute, &_hour, &_dayOfWeek, &_dayOfMonth, &_month, &_year);
  return _year;
}
void KB::scrollTime()
{
	char s[32];
	Wire.begin(4, 5);
	getDateTime(&_second, &_minute, &_hour, &_dayOfWeek, &_dayOfMonth, &_month, &_year);
	sprintf(s,"%02d:%02d",_hour,_minute);
	scroll(String(s));
}
void KB::scrollDate()
{
	char s[32];
	Wire.begin(4, 5);
	getDateTime(&_second, &_minute, &_hour, &_dayOfWeek, &_dayOfMonth, &_month, &_year);
	sprintf(s,"%02d/%02d/%02d",_dayOfMonth,_month,_year);
	scroll(String(s));
}
void KB::scrollDateTime()
{
	char s[32];
	Wire.begin(4, 5);
	getDateTime(&_second, &_minute, &_hour, &_dayOfWeek, &_dayOfMonth, &_month, &_year);
	sprintf(s,"%02d/%02d/%02d %02d:%02d",_dayOfMonth,_month,_year,_hour,_minute);
	scroll(String(s));
}
void KB::setDate(
    byte dayOfMonth, // 1-28/29/30/31
    byte month,      // 1-12
    byte year)       // 0-99
{
  Wire.begin(4, 5);
  Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
  WireSend(RTC_LOCATION+4);

  WireSend(decToBcd(dayOfMonth) & 0x3f);         // set the date in month (00111111)
  WireSend(decToBcd(month) & 0x1f);              // set the month (00011111)
  WireSend(decToBcd(year));                      // set the year (11111111)

  Wire.endTransmission();
}
void KB::setTime(
    byte hour, // 1-28/29/30/31
    byte minute,      // 1-12
    byte second)       // 0-99
{
  Wire.begin(4, 5);
  Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
  WireSend(RTC_LOCATION);

  WireSend(decToBcd(second) & 0x7f);             // set seconds and disable clock (01111111)
  WireSend(decToBcd(minute) & 0x7f);             // set minutes (01111111)
  WireSend(decToBcd(hour) & 0x3f);               // set hours and to 24hr clock (00111111)
 
  Wire.endTransmission();

   //Start Clock:
  Wire.beginTransmission(MCP7941x_RTC_I2C_ADDR);
  WireSend(RTC_LOCATION);
  WireSend(decToBcd(second) | 0x80); // set seconds and enable clock (10000000)
  Wire.endTransmission();
}

//============ Internet Time ====================//
void KB::printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

bool KB::syncTime(char *ssid,char *password,unsigned int timeout)
{
  struct tm timeinfo;  
  //configTime(gmtOffset_sec, 0, ntpServer);
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  for(int i=0;i<timeout*2;i++) 
  {
      if(WiFi.status() == WL_CONNECTED)
	  {
		ledWIFI(1);		// ON LED Wifi status
		Serial.println();
		Serial.println("CONNECTED!");
		//init and get the time
		configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
		if(!getLocalTime(&timeinfo)){
			Serial.println("Failed to obtain time");
			scroll("Fail to Synchronize Time!");
			return false;
		}
		int year = timeinfo.tm_year+1900;
		int month = timeinfo.tm_mon + 1;
		int dayOfWeek = timeinfo.tm_wday;
		int dayOfMonth = timeinfo.tm_mday;
		int hour = timeinfo.tm_hour;
		int min = timeinfo.tm_min;
		int sec = timeinfo.tm_sec;
		setDate(dayOfMonth,month,year%2000);
		setTime(hour,min,sec);
		//disconnect WiFi as it's no longer needed
		WiFi.disconnect(true);
		WiFi.mode(WIFI_OFF);	
		ledIOT(1);		// ON LED IOT status
		ledWIFI(0);		// ON LED WIFI status
		sound(600,100);		// Alarm for Get time success!
		delay(200);
		sound(1200,100);
		ledIOT(0);		// OFF LED IOT status
		scroll("Time sync completed!");
		return true;
	  }
	  digitalWrite(KB_LED_WIFI,!digitalRead(KB_LED_WIFI));	// Blink LED WIFI
	  delay(500);
	  Serial.print(".");
  }
  ledWIFI(0);	// OFF LED Wifi status
  Serial.println();
  Serial.println("Time Out!");
  scroll("Fail to Synchronize Time!");
  return false;
}
int KB::getNetYear()
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        return -1;
    }
    return timeinfo.tm_year + 1900;
}
int KB::getNetMonth()
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        return -1;
    }
    return timeinfo.tm_mon + 1;
}
int KB::getNetDay()
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        return -1;
    }
    return timeinfo.tm_mday;
}
int KB::getNetDayOfWeek()
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        return -1;
    }
    return timeinfo.tm_wday;
}
int KB::getNetHour()
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        return -1;
    }
    return timeinfo.tm_hour;
}
int KB::getNetMinute()
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        return -1;
    }
    return timeinfo.tm_min;
}
int KB::getNetSecond()
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        return -1;
    }
    return timeinfo.tm_sec;
}
/*
int32_t KB::ax()
{
    int32_t acc[3];
	Acc->GetAxes(acc);
	LSM303AGR_sensor a = {acc[0], acc[1], acc[2]};
	return -a.raw.x;
}
int32_t KB::ay()
{
    int32_t acc[3];
	Acc->GetAxes(acc);
	LSM303AGR_sensor a = {acc[0], acc[1], acc[2]};
	return -a.raw.y;
}
int32_t KB::az()
{
    int32_t acc[3];
	Acc->GetAxes(acc);
	LSM303AGR_sensor a = {acc[0], acc[1], acc[2]};
	return -a.raw.z;
	//return -a.z;
}
long KB::aStrength()
{
    int32_t acc[3];
	Acc->GetAxes(acc);
	LSM303AGR_sensor a = {acc[0], acc[1], acc[2]};
	double s = sqrt(pow(a.raw.x, 2) + pow(a.raw.y, 2) + pow(a.raw.z, 2));
	return (long)s;
}
bool KB::setShakeLevel(unsigned int level)
{
	if(level>=1 && level<=8)
	{	
		acc_shake_level = SHAKE_LEVEL_BASE +((level-1)*200);
	}
	
}
bool KB::isShake()
{
	if(aStrength()>acc_shake_level)
		return true;
	else
		return false;
}
bool KB::isScreenUp()
{
	if(az() > 900)
		return true;
	else
		return false;
}	
bool KB::isScreenDown()
{
	if(az() < -900)
		return true;
	else
		return false;
}
bool KB::isBoardDown()
{
	if(ay() > 900)
		return true;
	else
		return false;
}	
bool KB::isBoardUp()
{
	if(ay() < -900)
		return true;
	else
		return false;
}

bool KB::isTiltLeft()
{
	int angle = angleXY();
	if(angle>=-24 && angle<=85 && isScreenUp()==false && isBoardUp()==false && isScreenDown()==false)
		return true;
	else
		return false;
}
bool KB::isTiltRight()
{
	int angle = angleXY();
	if(((angle>=95 && angle<=180) || (angle>=-179 && angle<=-163)) && isScreenUp()==false && isScreenDown()==false && isBoardUp()==false)
		return true;
	else
		return false;
}
bool KB::isFreeFall()
{
	if(aStrength()<acc_free_fall_level)
		return true;
	else
		return false;
}
int KB::angleXY()
{
	int32_t acc[3];
	Acc->GetAxes(acc);
	LSM303AGR_sensor a = {acc[0], acc[1], acc[2]};
	double angle = 180 * atan2(a.raw.y,a.raw.x) / PI;
	return (int)angle;
}
int KB::roll()
{
	int32_t acc[3];
	Acc->GetAxes(acc);
	LSM303AGR_sensor a = {acc[0], acc[1], acc[2]};
	//double angle = (180 * atan2(-a.raw.z,a.raw.x) / PI) - 90;	// work for roll only
	
	double angle = 180 + (180 * atan2(-a.raw.z,-a.raw.x) / PI);
	angle = (angle >= 270)?(270-angle):(angle>=90)?(fmod(90- angle,-180)+180):-90-angle;
	
	return (int)angle;
}
int KB::pitch()
{
	int32_t acc[3];
	Acc->GetAxes(acc);
	LSM303AGR_sensor a = {acc[0], acc[1], acc[2]};
	double angle = 180 * atan2(a.raw.y,-a.raw.z) / PI;
	return (int)angle;
}
//=============== Compass sensor ==================================================================//
void KB::set_mag_min_max(LSM303AGR_raw mag_min, LSM303AGR_raw mag_max) {
  my_mag_min = mag_min;
  my_mag_max = mag_max;
}
void KB::cCalibrate() {
	
  LSM303AGR_raw acc_min = {9999, 9999, 9999};
  LSM303AGR_raw acc_max = { -9999, -9999, -9999};
  
  LSM303AGR_raw mag_min = {9999, 9999, 9999};
  LSM303AGR_raw mag_max = { -9999, -9999, -9999};
  
  LSM303AGR_sensor a ;
  LSM303AGR_sensor m ;
  long now = millis();
  print(" ");
  int pixel[16][8];
  int st_ok=1;
  int count_timer=0;
  int last_x=0;
  int last_y=0;

   for(int sx=0;sx<16;sx++){
		 delay(1);
		 for(int sy=0;sy<8;sy++){
			 delay(1);
			 pixel[sx][sy]=0;
		  }
	 }
  // TODO stop automatically if no more changes
  scroll("TILT TO FILL SCREEN");
  while (st_ok) {
     int32_t accelerometer[3];
	 int32_t magnetometer[3];
     Acc->GetAxes(accelerometer);
     a = {accelerometer[0], accelerometer[1], accelerometer[2]}; 
     Mag->GetAxes(magnetometer);
     m = {magnetometer[0], magnetometer[1], magnetometer[2]};
	 
	 int x = map(a.raw.x,800,-800,0,15);
	 int y = map(a.raw.y,-800,800,0,7);
	 if((x<=15)&&(x>=0)&&(y<=7)&&(y>=0)){
		matrix.drawPixel(x, y, 1);
		pixel[x][y]=1;
		if (m.raw.x < mag_min.x) mag_min.x = m.raw.x;
		if (m.raw.x > mag_max.x) mag_max.x = m.raw.x;

		if (m.raw.y < mag_min.y) mag_min.y = m.raw.y;
		if (m.raw.y > mag_max.y) mag_max.y = m.raw.y;

		if (m.raw.z < mag_min.z) mag_min.z = m.raw.z;
		if (m.raw.z > mag_max.z) mag_max.z = m.raw.z;
	 }
	 st_ok=0;
	 int8_t sy=0;
	 int8_t sx=0;
	 int8_t count_pixel=0;
	 for(sx=0;sx<15;sx++){
		 delay(1);
		 for(sy=0;sy<8;sy++){
			 delay(1);
			if(pixel[sx][sy]==0){
				st_ok=1;
				// break;
			}else{
				count_pixel++;
			}
		  }
	 }
	 //Serial.println(count_pixel);
	 if(count_timer>5){
		 if(count_pixel>102){
			 st_ok=0;
		 }else{
			 print(" ");
	         scroll("Time Out!");
			 return;
		 }
	 }
	 if (millis() - now > 500) {
		drawPixel(x, y, 0);
	  delay(100);
	  if((x<=15)&&(x>=0)&&(y<=7)&&(y>=0)){
		drawPixel(x, y, 1);
	  }
	  if((last_x==x)&&(last_y==y)){
		  count_timer++;
	  }else{
		  count_timer=0;
	  }
	  last_x=x;
	  last_y=y;
      now = millis();
    }
	
  }
  	print(" ");
	print("FINISH");
	delay(500);
	print(" ");
	my_mag_min = mag_min;
    my_mag_max = mag_max;
	setSramWord(10,mag_min.x);
	setSramWord(11,mag_min.y);
	setSramWord(12,mag_min.z);
    setSramWord(13,mag_max.x);
	setSramWord(14,mag_max.y);
	setSramWord(15,mag_max.z);
}
void KB::calibrate_mag(void) {
  Serial.println("Move sensor in all directions until max/min value do not change anymore");
  // Read magnetometer LSM303AGR.

  LSM303AGR_raw mag_min = {9999, 9999, 9999};
  LSM303AGR_raw mag_max = { -9999, -9999, -9999};
  LSM303AGR_sensor m ;
  long now = millis();

  // TODO stop automatically if no more changes
  while (1) {
    int32_t magnetometer[3];
    Mag->GetAxes(magnetometer);
    m = {magnetometer[0], magnetometer[1], magnetometer[2]};
    if (m.raw.x < mag_min.x) mag_min.x = m.raw.x;
    if (m.raw.x > mag_max.x) mag_max.x = m.raw.x;

    if (m.raw.y < mag_min.y) mag_min.y = m.raw.y;
    if (m.raw.y > mag_max.y) mag_max.y = m.raw.y;

    if (m.raw.z < mag_min.z) mag_min.z = m.raw.z;
    if (m.raw.z > mag_max.z) mag_max.z = m.raw.z;

    if (millis() - now > 1000) {
      Serial.print("Mag mins (X, Y, Z):{");
	  Serial.print(mag_min.x);
	  Serial.print(",");
	  Serial.print(mag_min.y);
	  Serial.print(",");
	  Serial.print(mag_min.z);
	  Serial.println("}");
      Serial.print("Mag maxs (X, Y, Z):{");
	  Serial.print(mag_max.x);
	  Serial.print(",");
	  Serial.print(mag_max.y);
	  Serial.print(",");
	  Serial.print(mag_max.z);
	  Serial.println("}");
      now = millis();
    }
  }
}
int KB::heading(void){
	// Read accelerometer LSM303AGR.
  int32_t accelerometer[3];
  int compass_heading;
  Acc->GetAxes(accelerometer);
  // Read magnetometer LSM303AGR.
  int32_t magnetometer[3];
  Mag->GetAxes(magnetometer);

  LSM303AGR_sensor m = {magnetometer[0], magnetometer[1], magnetometer[2]};
  LSM303AGR_sensor a = {accelerometer[0], accelerometer[1], accelerometer[2]};

  double x_mag = (0.0 + m.raw.x - my_mag_min.x) / (my_mag_max.x - my_mag_min.x) * 2 - 1;
  double y_mag = (0.0 + m.raw.y - my_mag_min.y) / (my_mag_max.y - my_mag_min.y) * 2 - 1;
  double z_mag = (0.0 + m.raw.z - my_mag_min.z) / (my_mag_max.z - my_mag_min.z) * 2 - 1;
  //Serial.printf("Mag norm (x, y, z): (%f, %f, %f)\n", x_mag, y_mag, z_mag);

  // Normalize acceleration measurements so they range from 0 to 1
  double s = sqrt(pow(a.raw.x, 2) + pow(a.raw.y, 2) + pow(a.raw.z, 2));
  double xAccelNorm = a.raw.x / s;
  double yAccelNorm = a.raw.y / s;
  //DF("Acc norm (x, y): (%f, %f)\n", xAccelNorm, yAccelNorm);

  
  double pitch = asin(-xAccelNorm);
  double roll = asin(yAccelNorm / cos(pitch));

  // tilt compensated magnetic sensor measurements
  double x_mag_comp = x_mag * cos(pitch) + z_mag * sin(pitch);
  double y_mag_comp = x_mag * sin(roll) * sin(pitch) + y_mag * cos(roll) - z_mag * sin(roll) * cos(pitch);

  // arctangent of y/x converted to degrees
  double heading = 180 * atan2(x_mag_comp,y_mag_comp) / PI;

	if (heading <= 0) 
	{
		heading = - heading;
	} else {
		heading = 360  - heading;
	}
	if(heading>360){
		return compass_heading;
	}else
	{
		compass_heading=heading;
		return heading;
	}

}
int32_t KB::cx()
{
    int32_t magnetometer[3];
	Mag->GetAxes(magnetometer);
	LSM303AGR_sensor m = {magnetometer[0], magnetometer[1], magnetometer[2]};
	return -m.raw.x*0.1;	// uTestla unit
}
int32_t KB::cy()
{
    int32_t magnetometer[3];
	Mag->GetAxes(magnetometer);
	LSM303AGR_sensor m = {magnetometer[0], magnetometer[1], magnetometer[2]};
	return -m.raw.y*0.1;	// uTestla unit
}
int32_t KB::cz()
{
    int32_t magnetometer[3];
	Mag->GetAxes(magnetometer);
	LSM303AGR_sensor m = {magnetometer[0], magnetometer[1], magnetometer[2]};
	return -m.raw.z*0.1;	// uTestla unit
}
long KB::cStrength()	
{
	int32_t magnetometer[3];
	Mag->GetAxes(magnetometer);

	LSM303AGR_sensor c = {magnetometer[0], magnetometer[1], magnetometer[2]};
	double s = sqrt(pow(c.raw.x, 2) + pow(c.raw.y, 2) + pow(c.raw.z, 2));
	s = 0.1*s;	// Convert ot uTestla unit
	return (long)s;	// u Testla unit
}
*/
