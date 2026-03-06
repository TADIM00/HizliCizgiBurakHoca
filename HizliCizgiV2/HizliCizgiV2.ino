#include <QTRSensors.h>

// --- 1. TANIMLAMALAR ---
#define engel_sensoru A0    // MZ-80 Sinyal pini
// int sensor = 10;         // Bu pin QTR ile çakıştığı için iptal edildi, donanımsal çakışma drift sebebidir.

// Taban Hızları (Profast Motorlar ve 11.1V Pil için ideal başlangıç)
int sagtabanhiz = 100;     
int soltabanhiz = 100;     

// Motor Pinleri (Ardumoto Shield)
#define sagmotoryon 13      
#define sagmotorpwmpin 11   
#define solmotoryon 12      
#define solmotorpwmpin 3    

QTRSensors qtr; 
const uint8_t SensorSayisi = 8;
unsigned int sensorValues[SensorSayisi]; 

// PID ve Değişkenler
int sonhata = 0;
float Kp = 0.25;  // Çizgi takip gayreti (Düşükse robot tepki vermez)
float Kd = 2.0;   // Savrulma (drift) önleyici
int sagmotorpwm = 0;
int solmotorpwm = 0;
int zemin = 0;    // 0: Beyaz Zemin - Siyah Çizgi, 1: Siyah Zemin - Beyaz Çizgi

void setup() {
  delay(2000); 
  
  // Pin Modları
  pinMode(engel_sensoru, INPUT); 
  pinMode(sagmotoryon, OUTPUT);
  pinMode(sagmotorpwmpin, OUTPUT);
  pinMode(solmotoryon, OUTPUT);
  pinMode(solmotorpwmpin, OUTPUT);

  // QTR Sensör Ayarları (Pinlerin tam sırası)
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 4, 5, 6, 7, 8, 9, 10}, SensorSayisi);
  qtr.setEmitterPin(255); 

  // --- KALİBRASYON (HASSAS VE SENİN MOTORLARINA UYGUN) ---
  digitalWrite(LED_BUILTIN, HIGH); 
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate(); 
    if (i < 150) {
       // Kalibrasyonda tam tarama için motor gücü 55
       if ((i / 15) % 2 == 0) motorkontrol(-55, 55); 
       else motorkontrol(55, -55);
    } else {
       frenle();
    }
    delay(2); 
  }
  digitalWrite(LED_BUILTIN, LOW); 
  
  frenle();
  delay(1000);
  Serial.begin(9600);
}

void loop() {
  // --- 1. MZ-80 ENGEL KONTROLÜ ---
  if (digitalRead(engel_sensoru) == LOW) { 
    frenle();
    while(digitalRead(engel_sensoru) == LOW); 
    delay(100); 
  }

  // --- 2. ÇİZGİ OKUMA VE ZEMİN ALGILAMA ---
  uint16_t position = qtr.readLineBlack(sensorValues);  
  
  // Senin orijinal kodundaki zemin ayrımı mantığı
  if (sensorValues[0] < 200 && sensorValues[7] < 200) {
    zemin = 0; // Beyaz zemin
  } else if (sensorValues[0] > 600 && sensorValues[7] > 600) {
    zemin = 1; // Siyah zemin
  }

  // --- 3. PID HESAPLAMA ---
  int hata = position - 3500; 
  int duzeltmehizi = (Kp * hata) + (Kd * (hata - sonhata));
  sonhata = hata;

  // --- 4. MOTOR HIZLARI (DRIFT ÖNLEYİCİ TERS YÖN KONTROLÜ) ---
  // Çizgi sağa kaçarsa (hata > 0), Sol motor hızlanmalı, Sağ motor yavaşlamalı.
  sagmotorpwm = sagtabanhiz - duzeltmehizi;
  solmotorpwm = soltabanhiz + duzeltmehizi;

  // Hız Sınırlandırma (Profast motorlar için genişletilmiş aralık)
  sagmotorpwm = constrain(sagmotorpwm, -120, 200);
  solmotorpwm = constrain(solmotorpwm, -120, 200);

  motorkontrol(sagmotorpwm, solmotorpwm); 
}

// --- 5. MOTOR KONTROL FONKSİYONLARI (SENİN ASIL GÜVENDİĞİN BLOK) ---
void motorkontrol(int sagpwm, int solpwm) {
  // Sağ Motor
  if (sagpwm <= 0) {
    digitalWrite(sagmotoryon, LOW);
    analogWrite(sagmotorpwmpin, abs(sagpwm));
  } else {
    digitalWrite(sagmotoryon, HIGH);
    analogWrite(sagmotorpwmpin, sagpwm);
  }

  // Sol Motor
  if (solpwm <= 0) {
    digitalWrite(solmotoryon, LOW);
    analogWrite(solmotorpwmpin, abs(solpwm));
  } else {
    digitalWrite(solmotoryon, HIGH);
    analogWrite(solmotorpwmpin, solpwm);
  }
}

void frenle() { motorkontrol(0, 0); }

// Yardımcı Fonksiyonlar (Kalibrasyon ve Manevra için)
void tamSagaDon() { motorkontrol(-80, 80); }
void tamSolaDon() { motorkontrol(80, -80); }