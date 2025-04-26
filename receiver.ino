#define E32_TTL_1W  
#include <TinyGPS++.h>
#include "LoRa_E32.h"
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#define SEALEVELPRESSURE_HPA (1013.25)
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>

SoftwareSerial mySerial(3, 4); 
SoftwareSerial gpsSerial(8, 9); 
Adafruit_MPU6050 mpu;
LoRa_E32 Kozmos(&mySerial); 
TinyGPSPlus gps;
Adafruit_BMP3XX bmp;

#define M0 7
#define M1 6

// Parameters
#define Adres 23
#define Kanal 23
#define GonderilecekAdres 4

// hold all telemetry data for transmission
struct Signal 
{
  char sifre[7] = "Kozmos Havacılık"; // kozmos Havacılık is our team name
  bool btn1;
  float btn2;
  float btn3;
  float btn4;
  float ivmex;
  float ivmey;
  float ivmez;
  float gyrox;
  float gyroy;
  float gyroz;
  double gpsen; 
  double gpsboy; 
} veri;

void setup() 
{
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  Serial.begin(9600);
  gpsSerial.begin(9600); 

  Kozmos.begin(); // Initialize the LoRa E32 module
  if (!bmp.begin_I2C()) // Try to start the BMP388 barometric sensor over I2C
   {
    Serial.println("BMP388 is not found!!");
    while (1);
  }
  if (!mpu.begin()) // Try to start the MPU6050 IMU sensor
 {
    Serial.println("MPU6050 is not found!!");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Configure accelerometer full-scale range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);     // Configure gyroscope full-scale range

  LoraE32Ayarlar();

  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);

  delay(1000);
}

void loop() 
{
  sensors_event_t a, g, temp; // Create event objects for accelerometer - gyroa - temp
  mpu.getEvent(&a, &g, &temp); // Read the latest data from the MPU6050 sensor

  if (!bmp.performReading()) // Trigger a reading on the BMP388 sensor
  {
    Serial.println("BMP388 read error");
    return;
  }

  while (gpsSerial.available() > 0)  // Read all available bytes from the GPS module
  {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isValid()) { // Check if GPS has a valid fix
    veri.gpsen = gps.location.lat();
    veri.gpsboy = gps.location.lng();
  } else {
    veri.gpsen = 0;
    veri.gpsboy = 0;
  }

//Reset-store data-convert data..
  veri.btn1 =false;
  veri.btn2 = bmp.pressure / 100.0;
  veri.btn3 = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  veri.btn4 = bmp.temperature;
  veri.ivmex = a.acceleration.x;
  veri.ivmey = a.acceleration.y;
  veri.ivmez = a.acceleration.z;
  veri.gyrox = g.gyro.x;
  veri.gyroy = g.gyro.y;
  veri.gyroz =g.gyro.z;

  ResponseStatus rs = Kozmos.sendFixedMessage(highByte(GonderilecekAdres), lowByte(GonderilecekAdres), Kanal, &veri, sizeof(Signal));
  Serial.println(rs.getResponseDescription());

  delay(500); 
}

// LoRa SX1278 E32 433T30D Configuration settings :) 
void LoraE32Ayarlar() {
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);

  ResponseStructContainer c = Kozmos.getConfiguration();
  Configuration configuration = (Configuration)c.veri;

  configuration.ADDL = lowByte(Adres);
  configuration.ADDH = highByte(Adres);
  configuration.CHAN = Kanal;

  configuration.SPED.airveriRate = AIR_veri_RATE_010_24;
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




