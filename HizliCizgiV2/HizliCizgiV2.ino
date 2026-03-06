#include <QTRSensors.h>

// --- 1. PINLER ---
#define engel_sensoru A0    
#define sag_yon 13          
#define sag_pwm 11          
#define sol_yon 12          
#define sol_pwm 3           

// --- 2. AGRESİF HIZ AYARLARI ---
int sag_taban_hiz = 220;    // Düzlükte uçuş modu (185'ten 220'ye çıktı)
int sol_taban_hiz = 220;     
float yavaslama_katsayisi = 0.02; // Daha az fren, daha çok hız

QTRSensors qtr; 
const uint8_t SensorSayisi = 8;
unsigned int sensorValues[SensorSayisi]; 

int sonhata = 0;
float Kp = 0.55;  // Hız arttığı için çizgiyi daha sert tutmalı
float Kd = 6.0;   // Yüksek hızda kafa atmasını engellemek için Kd artırıldı

void setup() {
  delay(2000); 
  pinMode(engel_sensoru, INPUT); 
  pinMode(sag_yon, OUTPUT);
  pinMode(sag_pwm, OUTPUT);
  pinMode(sol_yon, OUTPUT);
  pinMode(sol_pwm, OUTPUT);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 4, 5, 6, 7, 8, 9, 10}, SensorSayisi);
  
  // --- GÜÇLENDİRİLMİŞ KALİBRASYON ---
  digitalWrite(LED_BUILTIN, HIGH); 
  for (uint16_t i = 0; i < 250; i++) {
    qtr.calibrate(); 
    if (i < 200) {
       // 55 PWM: Motorun dişliyi rahat döndürdüğü ama fırlamadığı değer
       if ((i / 15) % 2 == 0) motorkontrol(-55, 55); 
       else motorkontrol(55, -55);
    } else { frenle(); }
    delay(2); 
    if (i % 8 == 0) motorkontrol(0, 0); // Daha az kesinti, daha akıcı tarama
  }
  digitalWrite(LED_BUILTIN, LOW); 
  frenle();
  delay(1000);
}

void loop() {
  // ENGEL KONTROLÜ
  if (digitalRead(engel_sensoru) == LOW) { 
    frenle();
    while(digitalRead(engel_sensoru) == LOW); 
  }

  // ÇİZGİ OKUMA
  uint16_t position = qtr.readLineBlack(sensorValues);  
  int hata = position - 3500; 

  // PID HESAPLAMA
  int duzeltme = (Kp * hata) + (Kd * (hata - sonhata));
  sonhata = hata;

  // --- TURBO HIZ MANTIĞI ---
  int aktif_hiz = sag_taban_hiz; 

  // Virajda hızı sadece hata çok büyükse (1500+) ve daha az miktarda kırıyoruz
  if (abs(hata) > 1500) {
    aktif_hiz = sag_taban_hiz - (abs(hata) * yavaslama_katsayisi);
  }

  int sag_motor_pwm = aktif_hiz - duzeltme;
  int sol_motor_pwm = aktif_hiz + duzeltme;

  // MAX GÜÇ (255 tam kapasite)
  sag_motor_pwm = constrain(sag_motor_pwm, -255, 255);
  sol_motor_pwm = constrain(sol_motor_pwm, -255, 255);

  motorkontrol(sag_motor_pwm, sol_motor_pwm); 
}

void motorkontrol(int sagpwm, int solpwm) {
  if (sagpwm <= 0) {
    digitalWrite(sag_yon, LOW);
    analogWrite(sag_pwm, min(abs(sagpwm), 255));
  } else {
    digitalWrite(sag_yon, HIGH);
    analogWrite(sag_pwm, min(sagpwm, 255));
  }

  if (solpwm <= 0) {
    digitalWrite(sol_yon, LOW);
    analogWrite(sol_pwm, min(abs(solpwm), 255));
  } else {
    digitalWrite(sol_yon, HIGH);
    analogWrite(sol_pwm, min(solpwm, 255));
  }
}

void frenle() { motorkontrol(0, 0); }