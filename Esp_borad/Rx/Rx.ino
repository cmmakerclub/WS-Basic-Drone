// stable 2.0.0 rc1 //

#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <WiFiUDP.h>
#include <EEPROM.h>
#include <Wire.h>

#define DEFAULT_SSID_LENGTH 16
#define ARM_Address 55
#define LED 16

typedef struct
{
  int8_t startByte;
  int8_t roll;
  int8_t pitch;
  int8_t throttle;
  int8_t yaw;
  int8_t checksum;
  char ssid[DEFAULT_SSID_LENGTH];
} ControlData;

typedef struct
{
  int8_t startByte;
  int8_t startByte2;
  int8_t yawPitchRoll;
  int16_t kp;
  int16_t ki;
  int16_t kd;
  int16_t checksum;
  char ssid[DEFAULT_SSID_LENGTH];
} TuningData;

WiFiUDP udp;
byte data[512] = {0};
unsigned int localPort = 12345;
TuningData tuningData[3] = {0};

int8_t Trim_value[3] = {0};  // trim //  yaw : pitch : roll
char accessPointName[DEFAULT_SSID_LENGTH] = {'\0'};
Ticker ticker_DEGUG;


String readEEPROM(int index, int length);
int writeEEPROM(int index, String text);
void loadTuningData(void);
void saveTuningData(int i);
void loadTrimData(void);
void saveTrimData(int8_t *tmp);
String ipToString(IPAddress ip);
String floatToString(float value, int length, int decimalPalces);
String intToString(int value, int length);
String hexToString(byte value);
void blink(void);
uint8_t setPIDgain(void);
void sentControlcommand(int8_t roll_tmp, int8_t pitch_tmp, int8_t throttle_tmp, int8_t yaw_tmp);
void Read_udp(void);
void senttrimcommand(void);
int8_t limmit(int8_t value, int8_t low_b, int8_t high_b);

IPAddress local_ip(192, 168, 5, 1);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 255);

void setup()
{
  delay(1000);

  WiFi.mode(WIFI_STA);
  delay(50);

  WiFi.disconnect();
  delay(200);

  pinMode(LED, OUTPUT);
  delay(200);

  Serial.begin(115200);

  sprintf(accessPointName, "BDrone-%lu", ESP.getChipId());
  WiFi.softAP(accessPointName, "12345678");
  delay(50);
  accessPointName[DEFAULT_SSID_LENGTH - 1] = {'\0'};


  WiFi.mode(WIFI_AP_STA);
  delay(50);

  udp.begin(localPort);
  delay(50);

  Wire.begin();
  Wire.setClock(400000);


  EEPROM.begin(512);
  delay(50);

  loadTuningData();
  loadTrimData();
  delay(1000);

  setPIDgain();

  ticker_DEGUG.attach_ms(1000, blink);


}

void loop()
{
  delay(5);
  Read_udp();
}

void Read_udp(void)
{
  int numberOfBytes = udp.parsePacket();

  if (numberOfBytes > 0)
  {

    udp.read(data, numberOfBytes);



    if (data[0] == 0XF0 && data[1] == 0XF0) // tuning data
    {
      TuningData tuningDataBuffer = {0};
      memcpy(&tuningDataBuffer, data, sizeof(TuningData));
      int16_t checksum = tuningDataBuffer.yawPitchRoll + tuningDataBuffer.kp + tuningDataBuffer.ki + tuningDataBuffer.kd;
      tuningDataBuffer.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';


      if (tuningDataBuffer.yawPitchRoll >= 0X01 && tuningDataBuffer.yawPitchRoll <= 0X03 && tuningDataBuffer.checksum == checksum && strcmp(tuningDataBuffer.ssid, accessPointName) == 0)
      {
        int i = tuningDataBuffer.yawPitchRoll - 1;
        tuningData[i] = tuningDataBuffer;
        saveTuningData(i);

        if (tuningDataBuffer.yawPitchRoll == 0X03) setPIDgain();

        float kp = tuningDataBuffer.kp / 10.0f;
        float ki = tuningDataBuffer.ki / 10.0f;
        float kd = tuningDataBuffer.kd / 10.0f;

      }
    }
    else if (data[0] == 0XFC && data[1] == 0XFC) // get tuning data
    {

      TuningData tuningDataBuffer = {0};
      memcpy(&tuningDataBuffer, data, sizeof(TuningData));
      int16_t checksum = tuningDataBuffer.yawPitchRoll + tuningDataBuffer.kp + tuningDataBuffer.ki + tuningDataBuffer.kd;
      tuningDataBuffer.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';

      if (tuningDataBuffer.checksum == checksum && strcmp(tuningDataBuffer.ssid, accessPointName) == 0)
      {
        // send tuning data to remote control
        for (tuningDataBuffer.yawPitchRoll = 1; tuningDataBuffer.yawPitchRoll <= 3; tuningDataBuffer.yawPitchRoll++)
        {
          int i = tuningDataBuffer.yawPitchRoll - 1;
          tuningDataBuffer.kp = tuningData[i].kp;
          tuningDataBuffer.ki = tuningData[i].ki;
          tuningDataBuffer.kd = tuningData[i].kd;
          tuningDataBuffer.checksum = tuningDataBuffer.yawPitchRoll + tuningDataBuffer.kp + tuningDataBuffer.ki + tuningDataBuffer.kd;
          memcpy(&data, &tuningDataBuffer, sizeof(TuningData));

          udp.beginPacket(udp.remoteIP(), udp.remotePort());
          udp.write(data, sizeof(TuningData));
          udp.endPacket();
          delay(200); // don't forget to delay
        }
      }
    }
    else if (data[0] == 0XFE) // trim and control
    {
      static ControlData controlData_prev;
      ControlData controlData = {0};
      memcpy(&controlData, data, sizeof(ControlData));

      int8_t checksum = controlData.roll + controlData.pitch + controlData.throttle + controlData.yaw;
      controlData.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';

      if (controlData.checksum == checksum && strcmp(controlData.ssid, accessPointName) == 0)
      {
        int8_t trimFlag = 0XFF;

        if (controlData.roll == trimFlag && controlData.pitch == trimFlag && controlData.throttle == trimFlag && controlData.yaw == trimFlag)
        {
          // Trim //
          int8_t trim_tmp_0[3] = {0};

          trim_tmp_0[0] = limmit(controlData_prev.roll, -15, 15);
          trim_tmp_0[1] = limmit(controlData_prev.pitch, -15, 15);
          trim_tmp_0[2] = limmit(controlData_prev.yaw, -15, 15);

          saveTrimData(Trim_value);
          memcpy(&Trim_value, &trim_tmp_0, 3);

          senttrimcommand();

        }
        else
        {
          // control //

          sentControlcommand(controlData.roll + Trim_value[0], controlData.pitch + Trim_value[1] , controlData.throttle, controlData.yaw + Trim_value[2]);

          memcpy(&controlData_prev, data, sizeof(ControlData));
          blink();

        }
      }

    }
  }
}

String readEEPROM(int index, int length)
{
  String text = "";
  char ch = 1;

  for (int i = index; (i < (index + length)) && ch; ++i)
  {
    if (ch = EEPROM.read(i))
    {
      text.concat(ch);
    }
  }

  return text;
}

int writeEEPROM(int index, String text)
{
  for (int i = index; i < text.length() + index; ++i)
  {
    EEPROM.write(i, text[i - index]);
  }

  EEPROM.write(index + text.length(), 0);
  EEPROM.commit();

  return text.length() + 1;
}

void loadTuningData(void)
{
  for (int i = 0; i < 3; i++)
  {
    int address = (i + 1) * 12;

    String str = readEEPROM(address, 4);
    tuningData[i].kp = str.toInt();
    address += 4;

    str = readEEPROM(address, 4);
    tuningData[i].ki = str.toInt();
    address += 4;

    str = readEEPROM(address, 4);
    tuningData[i].kd = str.toInt();
  }
}

void saveTuningData(int i)
{
  int address = (i + 1) * 12;

  String str = String(tuningData[i].kp);
  writeEEPROM(address, str);
  address += 4;

  str = String(tuningData[i].ki);
  writeEEPROM(address, str);
  address += 4;

  str = String(tuningData[i].kd);
  writeEEPROM(address, str);
}

void loadTrimData(void)
{
  for (int i = 0; i < 3; i++)
  {
    int address = (i) * 2;

    Trim_value[i] = EEPROM.read(address);

  }
}

void saveTrimData(int8_t *tmp)
{
  for (int i = 0; i < 3; i++)
  {
    int address = (i) * 2;
    EEPROM.write(address, *tmp + i );
  }
  EEPROM.commit();
}

String ipToString(IPAddress ip)
{
  return String(ip[0]) + "." + ip[1] + "." + ip[2] + "." + ip[3];
}

String floatToString(float value, int length, int decimalPalces)
{
  String stringValue = String(value, decimalPalces);
  String prefix = "";

  for (int i = 0; i < length - stringValue.length(); i++)
  {
    prefix += " ";
  }

  return prefix + stringValue;
}

String intToString(int value, int length)
{
  String stringValue = String(value);
  String prefix = "";

  for (int i = 0; i < length - stringValue.length(); i++)
  {
    prefix += " ";
  }

  return prefix + stringValue;
}

String hexToString(byte value)
{
  int length = 2;
  String stringValue = String(value, HEX);
  String prefix = "";

  for (int i = 0; i < length - stringValue.length(); i++)
  {
    prefix += "0";
  }

  return "0x" + prefix + stringValue;
}

void blink(void)
{
  static int8_t led = 0;
  led = 1 - led;
  digitalWrite(LED, led);
}

uint8_t setPIDgain(void)
{
  int16_t temp ;

  //1st
  uint8_t command = 0XFE;

  int8_t data_buffer[20] = {0};

  data_buffer[0] = (uint8_t)(tuningData[0].kp >> 8);
  data_buffer[1] = (uint8_t)(tuningData[0].kp);
  data_buffer[2] = (uint8_t)(tuningData[0].ki >> 8);
  data_buffer[3] = (uint8_t)(tuningData[0].ki);
  data_buffer[4] = (uint8_t)(tuningData[0].kd >> 8);
  data_buffer[5] = (uint8_t)(tuningData[0].kd);
  data_buffer[6] = (uint8_t)(tuningData[1].kp >> 8);
  data_buffer[7] = (uint8_t)(tuningData[1].kp);
  data_buffer[8] = (uint8_t)(tuningData[1].ki >> 8);
  data_buffer[9] = (uint8_t)(tuningData[1].ki);
  data_buffer[10] = (uint8_t)(tuningData[1].kd >> 8);
  data_buffer[11] = (uint8_t)(tuningData[1].kd);
  data_buffer[12] = (uint8_t)(tuningData[2].kp >> 8);
  data_buffer[13] = (uint8_t)(tuningData[2].kp);
  data_buffer[14] = (uint8_t)(tuningData[2].ki >> 8);
  data_buffer[15] = (uint8_t)(tuningData[2].ki);
  data_buffer[16] = (uint8_t)(tuningData[2].kd >> 8);
  data_buffer[17] = (uint8_t)(tuningData[2].kd);
  int16_t checksum_buffer = tuningData[0].kp + tuningData[0].ki + tuningData[0].kd + tuningData[1].kp + tuningData[1].ki + tuningData[1].kd + tuningData[2].kp + tuningData[2].ki + tuningData[2].kd ;
  data_buffer[18] = (uint8_t)(checksum_buffer >> 8);
  data_buffer[19] = (uint8_t)(checksum_buffer);

  for (int index = 0; index < 20; index++)
  {
    twi_writeTo(ARM_Address, (uint8_t*)data_buffer + index, 1, 1);
  }

  twi_writeTo(ARM_Address, &command, 1, 1);
  twi_writeTo(ARM_Address, &command, 1, 1);
  // 2nd

  for (int index = 0; index < 20; index++)
  {
    twi_writeTo(ARM_Address, (uint8_t*)data_buffer + index, 1, 1);
  }

  twi_writeTo(ARM_Address, &command, 1, 1);
  twi_writeTo(ARM_Address, &command, 1, 1);
  // Wire.requestFrom(ARM_Address, 1);
  // temp = Wire.read() << 8 | Wire.read();

  return 0;
}

void sentControlcommand(int8_t roll_tmp, int8_t pitch_tmp, int8_t throttle_tmp, int8_t yaw_tmp)
{
  //1st

  static int16_t throttle_tmp_buffer;


  if (analogRead(A0) < 770)
  {
    if (throttle_tmp_buffer > 0) throttle_tmp_buffer-- ;
    if (throttle_tmp > throttle_tmp_buffer / 10) throttle_tmp = throttle_tmp_buffer / 10;

  } else {
    throttle_tmp_buffer = throttle_tmp * 10;
  }

  int8_t command = roll_tmp + pitch_tmp + throttle_tmp + yaw_tmp;

  twi_writeTo(ARM_Address, (uint8_t*)&roll_tmp, 1, 1);
  twi_writeTo(ARM_Address, (uint8_t*)&pitch_tmp, 1, 1);
  twi_writeTo(ARM_Address, (uint8_t*)&throttle_tmp, 1, 1);
  twi_writeTo(ARM_Address, (uint8_t*)&yaw_tmp, 1, 1);
  twi_writeTo(ARM_Address, (uint8_t*)&command, 1, 1);

  command = 0xFD;
  twi_writeTo(ARM_Address, (uint8_t*)&command, 1, 1);
  twi_writeTo(ARM_Address, (uint8_t*)&command, 1, 1);
  // 2nd
  command = roll_tmp + pitch_tmp + throttle_tmp + yaw_tmp;

  twi_writeTo(ARM_Address, (uint8_t*)&roll_tmp, 1, 1);
  twi_writeTo(ARM_Address, (uint8_t*)&pitch_tmp, 1, 1);
  twi_writeTo(ARM_Address, (uint8_t*)&throttle_tmp, 1, 1);
  twi_writeTo(ARM_Address, (uint8_t*)&yaw_tmp, 1, 1);
  twi_writeTo(ARM_Address, (uint8_t*)&command, 1, 1);

  command = 0xFD;
  twi_writeTo(ARM_Address, (uint8_t*)&command, 1, 1);
  twi_writeTo(ARM_Address, (uint8_t*)&command, 1, 1);
}

void senttrimcommand(void)
{
  //1st

  uint8_t command = 0xFC;
  twi_writeTo(ARM_Address, (uint8_t*)&command, 1, 1);
  twi_writeTo(ARM_Address, (uint8_t*)&command, 1, 1);
  // 2nd

  command = 0xFC;
  twi_writeTo(ARM_Address, (uint8_t*)&command, 1, 1);
  twi_writeTo(ARM_Address, (uint8_t*)&command, 1, 1);


}


int8_t limmit(int8_t value, int8_t low_b, int8_t high_b)
{
  if (value < low_b) value = low_b;
  if (value > high_b) value = high_b;
  return value;
}


