#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>
#include <Arduino.h>
#include "wiring_private.h"
#define LED 13
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define RF95_FREQ 925.0
TinyGPSPlus gps;
#define BEACON_TIMEOUT 200
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);  // D10 TX ,D11 RX UWB software serial
//Uart Serial3 (&sercom4, 5, 2, SERCOM_RX_PAD_3, UART_TX_PAD_2);
Uart Serial3 (&sercom3, 21, 20, SERCOM_RX_PAD_1, UART_TX_PAD_0); //  GPS , SDA SCL

int triger_data[10] = { 0 };
int ACK_data[10] = { 0 };
int ACKACK_data[10] = { 0 };
int ID = 0;
int ID_ACK = 00;
uint8_t ID_ACKACK = 000;
int ACK_ls[10] = { 11, 22, 33, 44 };
int ACKACK_ls[10] = { 111, 222, 333, 444 };

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
int Trigger_Flag = 0;

String uwb_str = "";
String comdata = "";
String comdata2 = "";
char buffer[128];
String str0, str1, str2, str3, str4, str5;
volatile char chr5 ;
int m = 0;
long uwb_data[10] = { 0 };
int16_t data[2];

void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_Handler()
{
  Serial3.IrqHandler();
}

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200); // UWB Read
  digitalWrite(RFM95_RST, LOW); delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed"); while (1);
  }
  rf95.setTxPower(23, false);
  pinPeripheral(10, PIO_SERCOM); // Open Serial port
  pinPeripheral(11, PIO_SERCOM); // OPen Serial port
  pinPeripheral(20, PIO_SERCOM);
  pinPeripheral(21, PIO_SERCOM);
  digitalWrite(LED, LOW);

}

void serial1Event() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    if (inChar == 'o') {
      stringComplete = true;
    }
    else {
      inputString += inChar;
    }
  }
}

void loop()
{
  serial1Event();
  handle_uwb_str();
  if (stringComplete) {
    Trigger_Flag = inputString.toInt();
    inputString = "";
    stringComplete = false;
    Serial.print("From Rpi Trigger_Flag: "); Serial.println(Trigger_Flag);
    if (Trigger_Flag == 1) {
//      digitalWrite(LED, HIGH);
      triger_data[0] = Trigger_Flag;
      rf95.send((uint8_t*)triger_data, sizeof(triger_data)); rf95.waitPacketSent();
      Serial.print("Send Trigger_Flag: "); Serial.println(Trigger_Flag);
//      digitalWrite(LED, LOW);
      if (rf95.waitAvailableTimeout(BEACON_TIMEOUT))
      {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        int* int_buf = (int*)buf;
        if (rf95.recv((uint8_t*)buf, &len))
        {
//          Serial.print("Get ACK: "); Serial.println(int_buf[0]);
          ACKACK_data[0] = ACKACK_ls[0];
//          for (int i = 0; i < 3; i++) {
//            rf95.send((uint8_t*)ACKACK_data, sizeof(ACKACK_data)); rf95.waitPacketSent();
//            Serial.print("Send ACKACK1: "); Serial.println(ACKACK_data[0]);
//            Serial1.print(ACKACK_data[0]); Serial1.print(","); delay(100);
//          }
        }
        else
        {
          Serial.println("RECV FAIL");
        }
      }
    }
    else if (Trigger_Flag == 2) {
//      digitalWrite(LED, HIGH);
      triger_data[0] = Trigger_Flag;
      rf95.send((uint8_t*)triger_data, sizeof(triger_data)); rf95.waitPacketSent();
      Serial.print("Send Trigger_Flag: "); Serial.println(Trigger_Flag);
//      digitalWrite(LED, LOW);
      if (rf95.waitAvailableTimeout(BEACON_TIMEOUT))
      {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        int* int_buf = (int*)buf;
        if (rf95.recv((uint8_t*)buf, &len))
        {
//          Serial.print("Get ACK: "); Serial.println(int_buf[0]);
          ACKACK_data[0] = ACKACK_ls[1];
//          for (int i = 0; i < 3; i++) {
//            rf95.send((uint8_t*)ACKACK_data, sizeof(ACKACK_data)); rf95.waitPacketSent();
//            Serial.print("Send ACKACK2: "); Serial.println(ACKACK_data[0]);
//            Serial1.print(ACKACK_data[0]); Serial1.print(","); delay(100);
//          }
        }
        else
        {
          Serial.println("RECV FAIL");
        }
      }
    }
    else if (Trigger_Flag == 3) {
//      digitalWrite(LED, HIGH);
      triger_data[0] = Trigger_Flag;
      rf95.send((uint8_t*)triger_data, sizeof(triger_data)); rf95.waitPacketSent();
      Serial.print("Send Trigger_Flag: "); Serial.println(Trigger_Flag);
//      digitalWrite(LED, LOW);
      if (rf95.waitAvailableTimeout(BEACON_TIMEOUT))
      {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        int* int_buf = (int*)buf;
        if (rf95.recv((uint8_t*)buf, &len))
        {
//          Serial.print("Get ACK: "); Serial.println(int_buf[0]);
          ACKACK_data[0] = ACKACK_ls[2];
//          for (int i = 0; i < 3; i++) {
//            rf95.send((uint8_t*)ACKACK_data, sizeof(ACKACK_data)); rf95.waitPacketSent();
//            Serial.print("Send ACKACK3: "); Serial.println(ACKACK_data[0]);
//            Serial1.print(ACKACK_data[0]); Serial1.print(","); delay(100);
//          }
        }
        else
        {
          Serial.println("RECV FAIL");
        }
      }
    }

    Trigger_Flag = 0;
  }

}

void handle_uwb_str()
{
  if (Serial2.available())
  {
    comdata = Serial2.readStringUntil('\n');
    if (comdata[0] == 'm')
    {
      digitalWrite(LED, HIGH);
      //      m = comdata.indexOf(' ') - 2; //找 m 是第几个字节
      //      str0 = comdata.substring(m + 6, m + 14);
      //      long A0 = strtol(str0.c_str(), NULL, 16); //HexString conver to 10's
      //      str1 = comdata.substring(m + 15, m + 23);
      //      long A1 = strtol(str1.c_str(), NULL, 16); //HexString convert to 10's
      //      str2 = comdata.substring(m + 24, m + 32);
      //      long A2 = strtol(str2.c_str(), NULL, 16);
      //      str3 = comdata.substring(m + 33, m + 41);
      //      long A3 = strtol(str3.c_str(), NULL, 16);
      //      str4 = comdata.substring(m + 60, m + 61); // tag_num
      //      long tag_num = strtol(str4.c_str(), NULL, 16);
      //
      //      str5 = comdata.substring(m + 59, m + 60); // anchor or tag
      //      chr5 = str5[0];
      //      uwb_data[0] = (A0); uwb_data[1] = (A1); uwb_data[2] = (A2); uwb_data[3] = (A3);

      Serial1.print(comdata);// Serial1.print("; ");Serial.println("Send All UWB data ");
      //      Serial1.print(uwb_data[0]); Serial1.print(';'); Serial1.print(uwb_data[1]); Serial1.print(';');
      //      Serial1.print(uwb_data[2]); Serial1.print(';'); Serial1.print(uwb_data[3]);
      //      long uwb_data[10] = { 0 };
      digitalWrite(LED, LOW);
    }
    else {
      comdata = "";
    }
  }
  //  while (Serial2.read() >= 0)
  //  {} //清空串口缓存
}
