
#define E32_TTL_1W
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

SoftwareSerial mySerial(3, 4);
LoRa_E32 Kozmos(&mySerial);
TinyGPSPlus gps; 

#define M0 7
#define M1 6

#define Adres 4   
#define Kanal 23  // The channel received from the receiver is 23 allowing manual configuration :) 

// define the packet layout with all your sensor and GPS value
struct Signal 
{
  char sifre[7]; // holds my team ID string
  bool btn1;
  float btn2;  
  float btn3;  .
  float btn4;  
  float ivmex; 
  float ivmey;
  float ivmez;
  float gyrox;
  float gyroy;
  float gyroz;
  float gpsen;
  float gpsboy;
};

void setup() 
{
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  Serial.begin(9600);
  Kozmos.begin();

  LoraE32Ayarlar();

  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);

  delay(500);
  Serial.println("KOZMOS ROKET AVİYONİK ");
}

void loop() 
{
 // check if there is at least 1 byte waiting
  while (Kozmos.available() > 1) {
    ResponseStructContainer rsc = Kozmos.receiveMessage(sizeof(Signal));
    if (rsc.status.code == SUCCESS) {
      Signal* veri = (Signal*)rsc.data;

      if (strcmp(veri->sifre, "Kozmos") == 0) {
        Serial.print("BASINÇ: ");
        Serial.print(veri->btn2); 
        Serial.print("  YÜKSEKLİK: ");
        Serial.print(veri->btn3); 
        Serial.print("  SICAKLIK: ");
        Serial.print(veri->btn4); 

        Serial.print("  İVME_X: ");
        Serial.print(veri->ivmex);
        Serial.print("  İVME_Y: ");
        Serial.print(veri->ivmey);
        Serial.print("  İVME_Z: ");
        Serial.print(veri->ivmez);

        Serial.print("  GYRO_X: ");
        Serial.print(veri->gyrox);
        Serial.print("  GYRO_Y: ");
        Serial.print(veri->gyroy);
        Serial.print("  GYRO_Z: ");
        Serial.print(veri->gyroz);

        // GPS verileri
        Serial.print("  GPS_ENLEM: ");
        Serial.print(veri->gpsen);
        Serial.print("  GPS_BOYLAM: ");
        Serial.print(veri->gpsboy);
        Serial.println(); 
      }
    }
    rsc.close();
  }
}

// Configuration settings
void LoraE32Ayarlar() {
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);

  ResponseStructContainer c = Kozmos.getConfiguration();
  Configuration configuration = (Configuration)c.data;

  configuration.ADDL = lowByte(Adres);
  configuration.ADDH = highByte(Adres);
  configuration.CHAN = Kanal;

  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24; 
  configuration.OPTION.transmissionPower = POWER_30; 

  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.uartParity = MODE_00_8N1;
  configuration.OPTION.fec = FEC_0_OFF;
  configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
  configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;
  configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;

  ResponseStatus rs = Kozmos.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  c.close();
}