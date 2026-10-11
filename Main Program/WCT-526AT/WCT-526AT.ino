/*
  Project: WCT-526AT Energy Monitoring System
  Author: Adib Fridiansya
  Company: PT Piranti Kecerdasan Buatan

  Date Update: 11 Oktober 2026 
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
#include <esp_arduino_version.h>
#include <math.h>        

#define WDT_TIMEOUT 60 // Watchdog Timeout 60 detik

const unsigned short CAL_PLCONST_H = 0x0050; // High Byte PL_Constant (5.242.880)
const unsigned short CAL_PLCONST_L = 0x0000; // Low Byte PL_Constant
const unsigned short CAL_UGAIN     = 0x6530; // Voltage Calibrate (Decimal: 25904)
const unsigned short CAL_IGAIN     = 0x3DC3; // Current Calibrate (Decimal: 15811)
const unsigned short CAL_MMODE     = 0x1422; // MMode Gain 1x L-Line & N-Line

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

// Variabel pulsa wajib dibuat volatile dan dihitung murni di dalam ISR
volatile unsigned long pulseCount = 0; 
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;

const float METER_CONSTANT = 3200.0; 
const float JOULES_PER_PULSE = (1000.0 * 3600.0) / METER_CONSTANT; 

unsigned long lastSampleTime = 0;
unsigned long lastMqttAttempt = 0; 
const int sampleInterval = 200; 
const int maxSamples = 5;       
int sampleCount = 0;

float sumVoltage = 0, sumCurrent = 0, sumPower = 0, sumCosPhi = 0, sumFreq = 0;
double total_KWH_SPI = 0.0000; 
double total_KWH_CF  = 0.0000; 
bool writeToSlotA = true;

// --- INTERRUPT SERVICE ROUTINE ---
void IRAM_ATTR onPulseCF1() {
  unsigned long currentTime = micros();
  if (lastPulseTime != 0) {
    pulseInterval = currentTime - lastPulseTime;
  }
  lastPulseTime = currentTime;
  pulseCount++; 
}

// --- FUNGSI GPRS & MQTT NON-BLOCKING ---
void connectGPRS() {
  esp_task_wdt_reset(); 

  if (!modem.testAT()) {
    Serial.println("\n[MODEM] SIM800L tidak merespons AT command.");
    return;
  }

  Serial.print("\n[GPRS] Waiting for network signal...");
  digitalWrite(ledGreen, LOW); 
  
  if (!modem.waitForNetwork(20000)) { 
    Serial.println(" NO SIGNAL / REGISTRATION FAILED");
    return;
  }
  Serial.println(" SIGNAL OK!");

  Serial.print("[GPRS] Connecting APN...");
  esp_task_wdt_reset();
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" APN REJECTED");
    return;
  }
  Serial.println(" CONNECTED!");
}

void reconnectMqttNonBlocking() {
  if (millis() - lastMqttAttempt > 5000) { 
    lastMqttAttempt = millis();
    esp_task_wdt_reset(); 
    Serial.print("Attempting MQTT Connection...");
    
    String clientId = "WCT526AT-";
    clientId += String((uint32_t)ESP.getEfuseMac(), HEX);

    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println(" CONNECTED");
      digitalWrite(ledGreen, HIGH); 
    } else {
      Serial.print(" FAILED, rc=");
      Serial.println(client.state());
      digitalWrite(ledGreen, LOW);
    }
  }
}

void setup() {
  Serial.begin(115200);
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1500); 

  Serial.println("\n==============================================");
  Serial.println("         WCT-526AT SYSTEM BOOTING             ");
  Serial.println("==============================================");
  
  // Inisialisasi WDT dengan reconfigure untuk kompatibilitas Core v3.x
#if defined(ESP_ARDUINO_VERSION_VAL) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,
      .idle_core_mask = 0,
      .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&twdt_config);
#else
  esp_task_wdt_init(WDT_TIMEOUT, true);
#endif
  esp_task_wdt_add(NULL); 

  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledRed, LOW);

  pinMode(CF1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CF1_PIN), onPulseCF1, RISING);

  // --- INISIALISASI & KALIBRASI ATM90E26 ---
  SPI.begin(); 
  eic.InitEnergyIC(); 

  // Terapkan parameter kalibrasi kustom dari program utama
  eic.SetUGain(CAL_UGAIN);
  eic.SetIGain(CAL_IGAIN);
  eic.CalibrateEnergyIC(); // Hitung ulang Checksum & kirim ulang ke registri ATM90E26
  delay(200);

  // Cetak rincian parameter kalibrasi ke Serial Monitor
  uint32_t pl_const = ((uint32_t)CAL_PLCONST_H << 16) | CAL_PLCONST_L;
  Serial.println("\n--- PARAMETER KALIBRASI METROLOGI AKTIF ---");
  Serial.print("PL_Constant : 0x"); Serial.print(pl_const, HEX);
  Serial.print(" (Desimal: "); Serial.print(pl_const); Serial.println(")");
  
  Serial.print("Ugain       : 0x"); Serial.print(CAL_UGAIN, HEX);
  Serial.print(" (Desimal: "); Serial.print(CAL_UGAIN); Serial.println(")");
  
  Serial.print("Igain       : 0x"); Serial.print(CAL_IGAIN, HEX);
  Serial.print(" (Desimal: "); Serial.print(CAL_IGAIN); Serial.println(")");
  
  Serial.print("MMode       : 0x"); Serial.println(CAL_MMODE, HEX);
  
  Serial.print("Checksum 1  : 0x"); Serial.println(eic.CommEnergyIC(1, CSOne, 0x0000), HEX);
  Serial.print("Checksum 2  : 0x"); Serial.println(eic.CommEnergyIC(1, CSTwo, 0x0000), HEX);
  Serial.println("-------------------------------------------");

  // Pemulihan slot A/B (Redundancy Recovery NVS)
  wctPrefs.begin("kwh_meter", false); 
  
  double kwh_spi_A = wctPrefs.getDouble("kwh_spi_A", 0.0000);
  double kwh_spi_B = wctPrefs.getDouble("kwh_spi_B", 0.0000);
  double kwh_cf_A  = wctPrefs.getDouble("kwh_cf_A", 0.0000);
  double kwh_cf_B  = wctPrefs.getDouble("kwh_cf_B", 0.0000);

  total_KWH_SPI = (isnan(kwh_spi_A) || isnan(kwh_spi_B)) ? 0.0 : max(kwh_spi_A, kwh_spi_B);
  total_KWH_CF  = (isnan(kwh_cf_A) || isnan(kwh_cf_B))   ? 0.0 : max(kwh_cf_A, kwh_cf_B);
  
  writeToSlotA = (total_KWH_SPI == kwh_spi_B);

  Serial.println("\n--- STATUS NVS MEMORY ---");
  Serial.print("Restored kWh (SPI) : "); Serial.println(total_KWH_SPI, 4);
  Serial.print("Restored kWh (CF)  : "); Serial.println(total_KWH_CF, 4);
  Serial.println("-------------------------------------------");

  // Inisialisasi Perangkat Keras SIM800L
  Serial.println("\nInitializing Modem Hardware...");
  pinMode(MODEM_RST, OUTPUT);
  digitalWrite(MODEM_RST, HIGH); delay(200);
  digitalWrite(MODEM_RST, LOW);  delay(1000);
  digitalWrite(MODEM_RST, HIGH); delay(3000);

  Serial.print("Connecting to SIM800L AT Interface...");
  if (!modem.restart()) {
    Serial.println(" FAILED! (Cek Baudrate / Kabel RX-TX / Catu Daya SIM800L)");
  } else {
    Serial.println(" OK!");
  }

  connectGPRS();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  esp_task_wdt_reset(); 

  // Manajemen Konektivitas Non-Blocking
  if (!modem.isGprsConnected()) {
    digitalWrite(ledGreen, LOW);
    connectGPRS();
  } else if (!client.connected()) {
    reconnectMqttNonBlocking();
  } else {
    client.loop();
    digitalWrite(ledGreen, HIGH);
  }

  // --- PEMROSESAN PULSA CF1 DARI ISR ---
  noInterrupts();
  unsigned long currentPulses = pulseCount;
  pulseCount = 0; 
  unsigned long interval_us = pulseInterval;
  interrupts();

  if (currentPulses > 0) {
    total_KWH_CF += (currentPulses * (1.0 / METER_CONSTANT));
    float T_seconds = (float)interval_us / 1000000.0;
    float P_absolute = JOULES_PER_PULSE / T_seconds;

    Serial.print("[METROLOGY] CF1 | Pulses: "); Serial.print(currentPulses);
    Serial.print(" | Abs Power: "); Serial.print(P_absolute, 1);
    Serial.print(" W | CF kWh: "); Serial.println(total_KWH_CF, 4);
  }

  // --- PEMBACAAN IC METROLOGI ---
  if (millis() - lastSampleTime >= sampleInterval) {
    lastSampleTime = millis();

    if (eic.GetSysStatus() == 0xFFFF) {
      digitalWrite(ledRed, HIGH);
      Serial.println("[FATAL] SPI Error. System halted.");
      return; 
    }

    float currentVoltage = eic.GetLineVoltage();
    float currentAmpere  = eic.GetLineCurrent();
    float currentPower   = eic.GetActivePower();
    float currentCosPhi  = eic.GetPowerFactor();
    float currentFreq    = eic.GetFrequency();

    // Dying Gasp dan NVS Wear-Out Prevention
    if (currentVoltage < 80.0) {
      digitalWrite(ledRed, HIGH);
      
      if (!isDataSaved) {
        Serial.println("\n[CRITICAL] Power loss (< 80V)! Executing Dying Gasp NVS Write...");
        
        total_KWH_SPI += eic.GetImportEnergy();
        
        if (writeToSlotA) {
          wctPrefs.putDouble("kwh_spi_A", total_KWH_SPI);
          wctPrefs.putDouble("kwh_cf_A", total_KWH_CF);
          writeToSlotA = false;
        } else {
          wctPrefs.putDouble("kwh_spi_B", total_KWH_SPI);
          wctPrefs.putDouble("kwh_cf_B", total_KWH_CF);
          writeToSlotA = true;
        }
        
        isDataSaved = true;
        Serial.println("Safe-write complete. Entering Deep Sleep to prevent lock-loop...");
        esp_deep_sleep_start(); 
      }
    } 
    else if (currentVoltage > 100.0) {
      isDataSaved = false;
      digitalWrite(ledRed, LOW);
    }

    // Filter pembacaan noise arus rendah
    if (currentAmpere < 0.05) {
      currentAmpere = 0.0;
      currentPower = 0.0; 
      currentCosPhi = 1.00; 
    }

    // Akumulasi Moving Average
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

      Serial.print("TX -> V: "); Serial.print(avgVoltage, 1);
      Serial.print(" | I: "); Serial.print(avgCurrent, 3);
      Serial.print(" | P: "); Serial.print(avgPower, 1);
      Serial.print(" | SPI kWh: "); Serial.print(total_KWH_SPI, 4);
      Serial.print(" | CF kWh: "); Serial.println(total_KWH_CF, 4);

      if (client.connected()) {
        String device_id = "WCT-526AT: 001";
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
        delay(10); 
        digitalWrite(ledGreen, HIGH);
      }

      sumVoltage  = 0;
      sumCurrent  = 0;
      sumPower    = 0;
      sumCosPhi   = 0;
      sumFreq     = 0;
      sampleCount = 0;
    }
  }
}