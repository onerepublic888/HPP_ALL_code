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
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_Handler()
{
  Serial3.IrqHandler();
}

int16_t data[2];   // [0] = ID, [1] = RSSI
int triger_data[10] = { 0 };
int ACK_data[10] = { 0 };
uint8_t ID = 1;
uint8_t ID_ACK = 11;
uint8_t ID_ACKACK = 111;



// Trigger variable
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
uint8_t Trigger_Flag = 0;
bool polling_ACK = true;
bool set_tag = true;
int ack_data = 11;

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
  pinPeripheral(10, PIO_SERCOM); // Open Serial port
  pinPeripheral(11, PIO_SERCOM); // OPen Serial port
  pinPeripheral(20, PIO_SERCOM);
  pinPeripheral(21, PIO_SERCOM);
  rf95.setTxPower(23, false);  // 5~23 dBm
  digitalWrite(LED, LOW);

}



void loop()
{
  if (set_tag) {
    Serial2.write("AT+SW=11000010\r\n"); delay(1000);
    Serial2.write("AT+SW=11000010\r\n"); delay(1000);
    set_tag = false;
  }

  if (polling_ACK) {
    if (rf95.available())
    {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      uint8_t* data = (uint8_t*)buf;
      if (rf95.recv(buf, &len))
      {
        Serial.print("Receive buf: "); Serial.println(data[0]);
        if (ID == data[0]) {
          Serial2.write("AT+SW=11010010\r\n"); delay(1000);  //should be serial2 not serial1,cause UWB use serial2
          //How to check UWB have change?
          Serial2.write("AT+SW=11010010\r\n"); delay(1000);
          digitalWrite(LED, HIGH);
          ACK_data[0] = ack_data;
          Serial.print("Send ID_ACK: "); Serial.println(ACK_data[0]);
          rf95.send((uint8_t*)ACK_data, sizeof(ACK_data)); rf95.waitPacketSent();
          digitalWrite(LED, LOW);
          polling_ACK = false;
        }
        //        else if (ID_ACKACK == data[0]) {
        //          Serial.print("ID_ACKACK: "); Serial.println(ID_ACKACK);
        //          polling_ACK = false;
        //          Serial.println("Stop polling!");
        //        }
        else
        {
          Serial.println("Receive failed");
        }
      }
    }

  }
  else {
    digitalWrite(LED, HIGH);
    Serial.println("polling_ACK false, ID_ACKACK done ");
    digitalWrite(LED, LOW);
    delay(1000);
  }
  delay(1000);
}
