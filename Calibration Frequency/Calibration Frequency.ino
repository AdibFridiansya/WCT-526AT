#include <SPI.h>
#include "energyic_SPI.h" 

ATM90E26_SPI eic(5);  // Pin CS/SS

// Pin Metrologi
const int CF1_PIN = 35; 

// Variabel untuk Kalibrasi Presisi (Time-based)
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool newPulseDetected = false;

// Timer untuk Serial Print
unsigned long lastPrintTime = 0;

// Konstanta Meter Bawaan (Industri)
const float METER_CONSTANT = 3200.0; // imp/kWh
const float JOULES_PER_PULSE = (1000.0 * 3600.0) / METER_CONSTANT; // = 1125 W.s/pulsa

// ==========================================
// RUTIN INTERRUPT (Menangkap jarak waktu)
// ==========================================
void IRAM_ATTR onPulseCF1() {
  unsigned long currentTime = micros(); 
  
  if (lastPulseTime != 0) {
    pulseInterval = currentTime - lastPulseTime;
    newPulseDetected = true;
  }
  lastPulseTime = currentTime;
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000); 

  Serial.println("\n=======================================================");
  Serial.println("  WCT-001AT CALIBRATION: TIME-BASED ABSOLUTE POWER   ");
  Serial.println("=======================================================");

  pinMode(CF1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CF1_PIN), onPulseCF1, RISING);

  SPI.begin(); 
  delay(100);
  
  Serial.println("Inisialisasi IC...");
  eic.InitEnergyIC(); 
  delay(1000); 

  if(eic.GetSysStatus() == 0xFFFF) {
    Serial.println("ERROR: IC Mati / SPI Gagal!");
    while(true);
  } else {
    Serial.println("IC OK! Nyalakan Halogen 500W Anda sekarang...\n");
  }
}

// ==========================================
// LOOP
// ==========================================
void loop() {
  if (newPulseDetected) {
    noInterrupts();
    unsigned long interval_us = pulseInterval; 
    newPulseDetected = false;
    interrupts();

    float T_seconds = (float)interval_us / 1000000.0;
    float P_absolute = JOULES_PER_PULSE / T_seconds;
    float P_spi = eic.GetActivePower();

    // Pastikan nilai ini SAMA PERSIS dengan measurement[_igain] di energyic_SPI.cpp Anda saat ini
    unsigned short current_Igain = 0x4A38; 
    
    Serial.println("--- DETAK PULSA TERDETEKSI ---");
    Serial.print("Jarak Waktu (T): "); Serial.print(T_seconds, 4); Serial.println(" detik");
    Serial.print("[KEBENARAN] Daya Mutlak (Pulse) : "); Serial.print(P_absolute, 1); Serial.println(" Watt");
    Serial.print("[BACAAN IC] Daya Sensor (SPI)   : "); Serial.print(P_spi, 1); Serial.println(" Watt");

    // Proteksi Divide by Zero (Pastikan sensor mendeteksi daya minimal sebelum menghitung Gain)
    if (abs(P_spi) > 5.0) { 
        float ratio = P_absolute / P_spi;
        unsigned short recommended_Igain = (unsigned short)(current_Igain * ratio);
        
        Serial.print(">>> REKOMENDASI IGAIN BARU      : 0x"); 
        Serial.println(recommended_Igain, HEX);
    } else {
        Serial.println(">>> (Abaikan) Daya SPI terlalu kecil, membaca offset...");
    }
    Serial.println("------------------------------\n");
  }
  
  // Bacaan biasa yang aman dari spam (muncul tiap 2 detik jika tidak ada pulsa)
  if (!newPulseDetected && millis() - lastPrintTime >= 2000) {
     lastPrintTime = millis();
     Serial.print("V: "); Serial.print(eic.GetLineVoltage(), 1);
     Serial.print(" | I: "); Serial.println(eic.GetLineCurrent(), 3);
  }
}