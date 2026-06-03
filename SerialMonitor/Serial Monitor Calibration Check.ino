#include <SPI.h>
#include <energyic_SPI.h> 
#include <Preferences.h> // Library untuk Flash Memory
#include <math.h>        // Library untuk fungsi round()

// ==========================================
// HARDWARE & METROLOGI
// ==========================================
ATM90E26_SPI eic(5);  // Pin CS/SS

// Pin LED Diagnostik
const int ledGreen = 26; // Status Operasi Normal (Heartbeat)
const int ledRed   = 27; // Status Error Hardware / Kelistrikan

// Objek Memori & Bendera Dying Gasp
Preferences wctPrefs; 
bool isDataSaved = false; 

// Variabel untuk metode rata-rata (Moving Average)
unsigned long lastSampleTime = 0;
const int sampleInterval = 200; // 0,2 detik per sampel
const int maxSamples = 5;       // 1 detik total (5 sampel x 0,2 detik)

int sampleCount = 0;
float sumVoltage = 0, sumCurrent = 0, sumPower = 0, sumCosPhi = 0, sumFreq = 0;

// Variabel Celengan KWh (Double 64-bit)
double total_KWH = 0.0000; 

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1500); 

  Serial.println("\n=========================================");
  Serial.println("  WCT WATTMETER STAND-ALONE MODE ACTIVE  "); 
  Serial.println("=========================================");

  // Inisialisasi LED
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  digitalWrite(ledGreen, HIGH); // Nyala solid tanda sistem siap
  digitalWrite(ledRed, LOW);

  // Inisialisasi SPI & Metrologi
  SPI.begin(); 
  eic.InitEnergyIC(); 
  delay(500); 

  // Membuka Memori Flash dan memuat KWh terakhir 
  wctPrefs.begin("kwh_meter", false); 
  total_KWH = wctPrefs.getDouble("saved_kwh", 0.0000); 
  
  Serial.print("Data KWh sebelumnya berhasil dimuat: ");
  Serial.print(total_KWH, 4);
  Serial.println(" kWh");
  Serial.println("Sistem siap, mulai membaca...\n");
}

// ==========================================
// LOOP UTAMA
// ==========================================
void loop() {
  // Proses Sampling Data setiap 0,2 detik
  if (millis() - lastSampleTime >= sampleInterval) {
    lastSampleTime = millis();

    bool hardwareError = false;

    // Cek Status SPI
    if(eic.GetSysStatus() == 0xFFFF) {
      Serial.println("Jalur SPI mati, me-restart ESP32...");
      hardwareError = true;
      digitalWrite(ledRed, HIGH);
      delay(1000);
      ESP.restart(); 
    }

    // 1. Ambil data mentah
    float currentVoltage = eic.GetLineVoltage();
    float currentAmpere  = eic.GetLineCurrent();
    float currentPower   = eic.GetActivePower();
    float currentCosPhi  = eic.GetPowerFactor();
    float currentFreq    = eic.GetFrequency();

    // 2. DEADBAND FILTER TEGANGAN & DYING GASP
    if (currentVoltage < 10.0) {
      currentVoltage = 0.0;
      currentFreq = 0.0; 
      currentCosPhi = 1.00;
      hardwareError = true;

      // ---> FITUR DYING GASP TERPICU SAAT MATI LAMPU <---
      if (!isDataSaved) {
        digitalWrite(ledRed, HIGH); // Nyalakan LED Merah
        digitalWrite(ledGreen, LOW); // Matikan LED Hijau
        
        Serial.println("\n!!! PERINGATAN: LISTRIK PLN MATI !!!");
        Serial.println("Menyimpan saldo KWh ke memori Flash...");
        
        // Menyimpan menggunakan putDouble untuk presisi 64-bit
        wctPrefs.putDouble("saved_kwh", total_KWH); 
        
        Serial.println("Data KWh berhasil diamankan!");
        isDataSaved = true;

        // JEBAKAN KEMATIAN: Matikan semua proses agar kapasitor tidak terkuras
        while(true) {
          delay(100); 
        }
      }
    } else {
      isDataSaved = false; // Reset bendera jika listrik menyala normal
    }

    // 3. DEADBAND FILTER ARUS
    if (currentAmpere < 0.05) {
      currentAmpere = 0.0;
      currentPower = 0.0; 
      currentCosPhi = 1.00; 
    }

    // Eksekusi LED Merah berdasarkan evaluasi Hardware Error
    if (hardwareError) {
      digitalWrite(ledRed, HIGH); 
    } else {
      digitalWrite(ledRed, LOW);
    }

    // 4. Akumulasi data bersih
    sumVoltage += currentVoltage;
    sumCurrent += currentAmpere;
    sumPower   += currentPower;
    sumCosPhi  += currentCosPhi;
    sumFreq    += currentFreq;
    sampleCount++;

    // 5. Jika sudah mencapai 5 sampel (1 detik), hitung rata-rata dan cetak
    if (sampleCount >= maxSamples) {
      float avgVoltage = sumVoltage / maxSamples;
      float avgCurrent = sumCurrent / maxSamples;
      float avgPower   = sumPower / maxSamples;
      float avgCosPhi  = sumCosPhi / maxSamples;
      float avgFreq    = sumFreq / maxSamples;
      
      // Tambahkan pecahan energi terbaru ke celengan KWh
      total_KWH += eic.GetImportEnergy(); 
      
      // KUNCI PRESISI: Hard Rounding persis 4 digit di belakang koma
      total_KWH = round(total_KWH * 10000.0) / 10000.0;

      // Tampilkan di Serial Monitor
      Serial.print("Data (1 Detik) -> V: "); Serial.print(avgVoltage, 1);
      Serial.print("V | I: "); Serial.print(avgCurrent, 3);
      Serial.print("A | P: "); Serial.print(avgPower, 1);
      Serial.print("W | Freq: "); Serial.print(avgFreq, 2);
      Serial.print("Hz | CosPhi: "); Serial.print(avgCosPhi, 2);
      Serial.print(" | kWh: "); Serial.print(total_KWH, 4); Serial.println(" kWh");

      // Efek visual Heartbeat: Matikan LED Hijau sejenak saat mencetak data
      digitalWrite(ledGreen, LOW);
      delay(50); // Dipercepat menjadi 50ms agar kedipan lebih gesit
      digitalWrite(ledGreen, HIGH);

      // Reset variabel rata-rata (total_KWH TIDAK DIRESET)
      sumVoltage  = 0;
      sumCurrent  = 0;
      sumPower    = 0;
      sumCosPhi   = 0;
      sumFreq     = 0;
      sampleCount = 0;
    }
  }
}