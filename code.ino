#include <WiFi.h>
#include <ThingSpeak.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <math.h>

// ——— Wi-Fi & ThingSpeak ———
const char* ssid = "realme C12";
const char* password = "123456789";
unsigned long myChannelNumber = 2924131;
const char* myWriteAPIKey     = "FBCBG6RPAICJIEVE";
WiFiClient client;

// ——— TDS Settings ———
#define TdsSensorPin 34
#define VREF         3.3
#define SCOUNT       30
float calibrationFactor = 1.0;
const float standardTDS = 342.0;  // ppm solution for calibration
int analogBuffer[SCOUNT];
int analogBufferIndex = 0;

// ——— DS18B20 Settings ———
#define ONE_WIRE_BUS 25
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ——— Timing ———
unsigned long lastPublish = 0;
const unsigned long publishInterval = 15000;

// ——— Helpers ———
// median filter for ADC
int getMedian(int *arr, int len) {
  static int tmp[SCOUNT];
  memcpy(tmp, arr, sizeof(tmp));
  for(int i=0;i<len-1;i++)
    for(int j=0;j<len-1-i;j++)
      if(tmp[j]>tmp[j+1]) { int t=tmp[j]; tmp[j]=tmp[j+1]; tmp[j+1]=t; }
  return (len&1) ? tmp[len/2] : (tmp[len/2]+tmp[len/2-1])/2;
}

// read and average ADC ring buffer
float getAverageADC() {
  long sum = 0;
  for(int i=0;i<SCOUNT;i++) sum += analogBuffer[i];
  return sum / (float)SCOUNT;
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);

  // prefill ADC buffer
  for(int i=0;i<SCOUNT;i++) analogBuffer[i] = analogRead(TdsSensorPin);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // connect Wi-Fi
  WiFi.begin(ssid, password);
  while(WiFi.status()!=WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
  ThingSpeak.begin(client);

  // init DS18B20
  sensors.begin();

  // load calibration factor from EEPROM (float at address 0)
  EEPROM.get(0, calibrationFactor);
  if (!isfinite(calibrationFactor) || calibrationFactor <= 0) {
    calibrationFactor = 1.0;
  }

  // perform one-point calibration on startup
  Serial.println("Calibrating TDS. Immerse probe in " + String(standardTDS) + " ppm solution.");
  delay(10000); // give time to immerse
   
  // sample and compute raw TDS
  int medianADC = getMedian(analogBuffer, SCOUNT);
  float voltage = medianADC * (VREF / 4096.0);
  float waterTemp = sensors.getTempCByIndex(0);
  float compV = voltage / (1.0 + 0.02*(waterTemp-25.0));
  float rawTDS = (133.42*pow(compV,3) - 255.86*pow(compV,2) + 857.39*compV)*0.5;
  
  calibrationFactor = standardTDS / rawTDS;
  Serial.printf("Calibration factor: %.4f\n", calibrationFactor);
  EEPROM.put(0, calibrationFactor);
  EEPROM.commit();
  Serial.println("Calibration saved.\n");
}

void loop() {
  unsigned long now = millis();

  // sample ADC every 40 ms
  static unsigned long lastSample=0;
  if (now - lastSample >= 40) {
    lastSample = now;
    analogBuffer[analogBufferIndex++] = analogRead(TdsSensorPin);
    if (analogBufferIndex >= SCOUNT) analogBufferIndex = 0;
  }

  // publish every interval
  if (now - lastPublish >= publishInterval) {
    lastPublish = now;

    // compute raw TDS    
    float avgADC = getAverageADC();
    float voltage = avgADC * (VREF / 4096.0);
    float waterTemp = sensors.getTempCByIndex(0);
    // compensation: alpha=0.02 per °C
    float compV = voltage / (1.0 + 0.02*(waterTemp-25.0));
    float rawTDS = (133.42*pow(compV,3) - 255.86*pow(compV,2) + 857.39*compV)*0.5;
    // apply calibration
    float tdsValue = rawTDS * calibrationFactor;

    Serial.printf("Temp: %.2f°C  RawTDS: %.1f  CalTDS: %.1f ppm\n",
                  waterTemp, rawTDS, tdsValue);

    // send to ThingSpeak
    if (WiFi.status()==WL_CONNECTED) {
      ThingSpeak.setField(1, tdsValue);
      ThingSpeak.setField(2, waterTemp);
      int resp = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      Serial.print("TS resp: "); Serial.println(resp);
    }
  }
}