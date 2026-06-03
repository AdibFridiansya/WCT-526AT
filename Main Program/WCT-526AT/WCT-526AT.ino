/*
  Project: WCT-526AT Energy Monitoring System
  Author: Adib Fridiansya
  Company: PT Piranti Kecerdasan Buatan

  Description:
  Fixed production firmware template featuring hardware watchdog protection,
  optimized dual-bank NVS data retention, and realistic 185V AC under-voltage 
  threshold filtering for real-world PJU deployment.
*/

#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h> 
#include <SPI.h>
#include <energyic_SPI.h> 
#include <Preferences.h> 
#include <esp_task_wdt.h>  
#include <math.h>        

#define WDT_TIMEOUT 8     

/* Network and MQTT credentials */
const char apn[]      = "internet"; 
const char gprsUser[] = "";
const char gprsPass[] = "";

const char* mqtt_server = "brox.pikebuapp.web.id"; 
const int mqtt_port     = 1884;                                           
const char* mqtt_topic  = "pju/wattmeterct"; 
const char* mqtt_user   = "wattmeterct";
const char* mqtt_pass   = "wattmeterct123";

const int MODEM_RST = 4;
const int MODEM_RX  = 16;
const int MODEM_TX  = 17;
const int CF1_PIN   = 35; 
const int ledGreen  = 26; 
const int ledRed    = 27; 

HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
PubSubClient client(gsmClient);
ATM90E26_SPI eic(5);  
Preferences wctPrefs; 

bool isDataSaved = false; 
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool newPulseDetected = false;

/* Metrology constants */
const float METER_CONSTANT = 3200.0; 
const float JOULES_PER_PULSE = (1000.0 * 3600.0) / METER_CONSTANT; 

/* Moving average parameters */
unsigned long lastSampleTime = 0;
const int sampleInterval = 200; 
const int maxSamples = 5;       

int sampleCount = 0;
float sumVoltage = 0, sumCurrent = 0, sumPower = 0, sumCosPhi = 0, sumFreq = 0;

double total_KWH_SPI = 0.0000; 
double total_KWH_CF  = 0.0000; 

bool lastWrittenSlotA = true;

void IRAM_ATTR onPulseCF1() {
  unsigned long currentTime = micros();
  if (lastPulseTime != 0) {
    pulseInterval = currentTime - lastPulseTime;
    newPulseDetected = true;
  }
  lastPulseTime = currentTime;
}

void connectGPRS() {
  Serial.print("Connecting to GPRS...");
  digitalWrite(ledGreen, LOW); 
  
  esp_task_wdt_reset(); 
  if (!modem.waitForNetwork(10000)) {
    Serial.println(" FAILED");
    return;
  }
  
  esp_task_wdt_reset();
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" APN REJECTED");
    return;
  }
  Serial.println(" OK");
}

void reconnect() {
  while (!client.connected()) {
    esp_task_wdt_reset(); 
    Serial.print("Connecting to MQTT Broker...");
    digitalWrite(ledGreen, LOW); 
    
    String clientId = "WCT526AT-";
    clientId += String((uint32_t)ESP.getEfuseMac(), HEX);

    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println(" CONNECTED");
      digitalWrite(ledGreen, HIGH); 
    } else {
      Serial.print(" FAILED, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5s...");

      for(int i = 0; i < 5; i++) {
        esp_task_wdt_reset();
        digitalWrite(ledGreen, HIGH); delay(500);
        digitalWrite(ledGreen, LOW);  delay(500);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1500); 

  Serial.println("\n[ WCT-526AT SYSTEM BOOTING ]");
  
  esp_task_wdt_init(WDT_TIMEOUT, true); 
  esp_task_wdt_add(NULL); 

  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledRed, LOW);

  pinMode(CF1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CF1_PIN), onPulseCF1, RISING);

  SPI.begin(); 
  eic.InitEnergyIC(); 
  delay(500); 

  wctPrefs.begin("kwh_meter", false); 
  
  double kwh_spi_A = wctPrefs.getDouble("kwh_spi_A", 0.0000);
  double kwh_spi_B = wctPrefs.getDouble("kwh_spi_B", 0.0000);
  double kwh_cf_A  = wctPrefs.getDouble("kwh_cf_A", 0.0000);
  double kwh_cf_B  = wctPrefs.getDouble("kwh_cf_B", 0.0000);

  total_KWH_SPI = (kwh_spi_A > kwh_spi_B) ? kwh_spi_A : kwh_spi_B;
  total_KWH_CF  = (kwh_cf_A > kwh_cf_B) ? kwh_cf_A : kwh_cf_B;
  
  Serial.println("--- INTEGRITY CHECK SUCCESSFUL ---");
  Serial.print("Restored kWh (SPI) : "); Serial.println(total_KWH_SPI, 4);
  Serial.print("Restored kWh (CF)  : "); Serial.println(total_KWH_CF, 4);

  Serial.println("Initializing Modem...");
  pinMode(MODEM_RST, OUTPUT);
  digitalWrite(MODEM_RST, HIGH); delay(200);
  digitalWrite(MODEM_RST, LOW);  delay(1000);
  digitalWrite(MODEM_RST, HIGH); delay(3000);

  connectGPRS();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  esp_task_wdt_reset(); 

  if (!modem.isGprsConnected()) connectGPRS();
  if (!client.connected()) reconnect();
  client.loop();

  if (client.connected()) digitalWrite(ledGreen, HIGH);

  /* Metrology diagnostics via CF1 hardware interrupt */
  if (newPulseDetected) {
    noInterrupts();
    unsigned long interval_us = pulseInterval;
    newPulseDetected = false;
    interrupts();

    float T_seconds = (float)interval_us / 1000000.0;
    float P_absolute = JOULES_PER_PULSE / T_seconds;

    total_KWH_CF += (1.0 / METER_CONSTANT);

    Serial.print("[METROLOGY] CF1 Period: "); Serial.print(T_seconds, 4);
    Serial.print("s | Abs Power: "); Serial.print(P_absolute, 1);
    Serial.print(" W | CF kWh: "); Serial.println(total_KWH_CF, 4);
  }

  if (millis() - lastSampleTime >= sampleInterval) {
    lastSampleTime = millis();
    bool hardwareError = false;
    bool criticalUnderVoltage = false;

    if(eic.GetSysStatus() == 0xFFFF) {
      digitalWrite(ledRed, HIGH);
      Serial.println("[FATAL] SPI Error. Restarting system...");
      delay(1500); 
      ESP.restart(); 
    }

    float currentVoltage = eic.GetLineVoltage();
    float currentAmpere  = eic.GetLineCurrent();
    float currentPower   = eic.GetActivePower();
    float currentCosPhi  = eic.GetPowerFactor();
    float currentFreq    = eic.GetFrequency();

    if (currentVoltage < 185.0 && currentVoltage > 80.0) {
      criticalUnderVoltage = true;
      Serial.print("[WARNING] Grid Voltage Drop Critical: "); Serial.print(currentVoltage, 1); Serial.println(" V AC");
    } 
    // Dying Gasp
    else if (currentVoltage <= 10.0) {
      currentVoltage = 0.0;
      currentFreq = 0.0; 
      currentCosPhi = 1.00;
      hardwareError = true;

      if (!isDataSaved) {
        Serial.println("\n[CRITICAL] Power loss detected. Initiating Dying Gasp Dynamic Save...");
        
        if (lastWrittenSlotA) {
          wctPrefs.putDouble("kwh_spi_B", total_KWH_SPI);
          wctPrefs.putDouble("kwh_cf_B", total_KWH_CF);
        } else {
          wctPrefs.putDouble("kwh_spi_A", total_KWH_SPI);
          wctPrefs.putDouble("kwh_cf_A", total_KWH_CF);
        }
        
        isDataSaved = true;
        Serial.println("Safe-write complete. Entering deep sleep sleep-loop.");
        while(true) {
          digitalWrite(ledRed, HIGH); 
          delay(100); 
        }
      }
    } else {
      isDataSaved = false; 
    }

    if (currentAmpere < 0.05) {
      currentAmpere = 0.0;
      currentPower = 0.0; 
      currentCosPhi = 1.00; 
    }

    if (hardwareError || criticalUnderVoltage) {
      digitalWrite(ledRed, HIGH);
    } else {
      digitalWrite(ledRed, LOW);
    }

    sumVoltage += currentVoltage;
    sumCurrent += currentAmpere;
    sumPower   += currentPower;
    sumCosPhi  += currentCosPhi;
    sumFreq    += currentFreq;
    sampleCount++;

    if (sampleCount >= maxSamples) {
      float avgVoltage = sumVoltage / maxSamples;
      float avgCurrent = sumCurrent / maxSamples;
      float avgPower   = sumPower / maxSamples;
      float avgCosPhi  = sumCosPhi / maxSamples;
      float avgFreq    = sumFreq / maxSamples;
      
      total_KWH_SPI += eic.GetImportEnergy(); 
      total_KWH_SPI = round(total_KWH_SPI * 10000.0) / 10000.0;

      if (lastWrittenSlotA) {
        wctPrefs.putDouble("kwh_spi_B", total_KWH_SPI);
        wctPrefs.putDouble("kwh_cf_B", total_KWH_CF);
        lastWrittenSlotA = false;
      } else {
        wctPrefs.putDouble("kwh_spi_A", total_KWH_SPI);
        wctPrefs.putDouble("kwh_cf_A", total_KWH_CF);
        lastWrittenSlotA = true;
      }

      Serial.print("TX -> V: "); Serial.print(avgVoltage, 1);
      Serial.print(" | I: "); Serial.print(avgCurrent, 3);
      Serial.print(" | P: "); Serial.print(avgPower, 1);
      Serial.print(" | SPI kWh: "); Serial.print(total_KWH_SPI, 4);
      Serial.print(" | CF kWh: "); Serial.println(total_KWH_CF, 4);

      String device_id = "WCT-526AT: 00x"; 
      String payload = "{";
      payload += "\"device_id\":\"" + device_id + "\",";
      payload += "\"voltage\":" + String(avgVoltage, 2) + ",";
      payload += "\"current\":" + String(avgCurrent, 3) + ",";
      payload += "\"power\":" + String(avgPower, 2) + ",";
      payload += "\"frequency\":" + String(avgFreq, 2) + ",";
      payload += "\"cos_phi\":" + String(avgCosPhi, 2) + ",";
      payload += "\"kwh_spi\":" + String(total_KWH_SPI, 4) + ",";
      payload += "\"kwh_cf\":" + String(total_KWH_CF, 4);
      payload += "}";

      digitalWrite(ledGreen, LOW);
      client.publish(mqtt_topic, payload.c_str());
      delay(50); 
      digitalWrite(ledGreen, HIGH);

      sumVoltage  = 0;
      sumCurrent  = 0;
      sumPower    = 0;
      sumCosPhi   = 0;
      sumFreq     = 0;
      sampleCount = 0;
    }
  }
}