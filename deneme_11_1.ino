#include <Wire.h>
#include <VL53L1X.h>
#include <MPU6050.h>

unsigned long prevmillis = 0;
const unsigned long measurementInterval = 1000; // 1 saniye ölçüm aralığı

VL53L1X sensor;
MPU6050 accelgyro;
int16_t ax, ay, az; // ivme tanımlama
int16_t gx, gy, gz; // gyro tanımlama

// CWD556 sürücü pin bağlantıları
const int pullPin = 17;    // PULL pini
const int dirPin = 18;     // Yön pini
const int enablePin = 5;   // Enable pini
const int stepsPerRevolution = 1600; // Step motor adım sayısı (pulse/rev)

// Giriş pinleri
const int inputPins[] = {2, 15, 4, 16 };

const int pirPin = 35;   // PIR Sensörün çıkış pini GPIO 35'e bağlı
const int audioPin = 19;  // audio bağlı olduğu GPIO pin 19
int audioValue = 0;


// 32. pin, motorun aktif olduğu durumda aynı durumda olacak olan role pin
const int motorRolePin = 32;
const int motorRolePin1 = 33;
const int motorRolePin3 = 26;
const int motorRolePin7 = 13;
const int motorRolePin2 = 27;


void setup() {
  Wire.begin(21, 22);
  Wire.setClock(400000); // 400 kHz I2C hızı
  sensor.setTimeout(50);

  pinMode(pirPin, INPUT);

  pinMode(audioPin, INPUT_PULLUP);

  pinMode(pullPin, OUTPUT);   // PULL pini çıkış olarak ayarla
  pinMode(dirPin, OUTPUT);    // Yön pini çıkış olarak ayarla
  pinMode(enablePin, OUTPUT); // Enable pini çıkış olarak ayarla

  pinMode(motorRolePin,  OUTPUT); // Motor role pinini çıkış olarak ayarla
  pinMode(motorRolePin1, OUTPUT); 
  pinMode(motorRolePin2, OUTPUT);
  pinMode(motorRolePin3, OUTPUT);
  pinMode(motorRolePin7, OUTPUT);

  digitalWrite(enablePin, HIGH); // Step motoru etkinleştir

  // Giriş pinlerini giriş olarak ayarla
  for (int i = 0; i < sizeof(inputPins) / sizeof(inputPins[0]); i++) {
    pinMode(inputPins[i], INPUT);
  }

  accelgyro.initialize();

  if (!sensor.init(false)) {
    Serial.println("Sensör tespit edilemedi veya başlatılamadı!");
  } else {
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(50);
  }

  Serial.begin(115200);
  while (!Serial) {}
}

void loop() {
  if ((millis() - prevmillis) >= measurementInterval) {
    prevmillis = millis();

    int pirValue = digitalRead(pirPin);

    int audioValue = digitalRead(audioPin);

    long vl53l1x = sensor.read();
    Serial.print("Entfernung VL53L1X: ");
    Serial.print(vl53l1x);
    Serial.println(" mm");

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // ivme ve gyro değerlerini okuma
    delay(700); // 0.7 saniye dur

    if (digitalRead(inputPins[3]) == HIGH || audioValue == LOW || pirValue == HIGH ) {  //vl53l1x < 1000 || 

    digitalWrite(motorRolePin2, HIGH); // Motor role pini aktif

    ileriGit(4 * stepsPerRevolution); // İleri gitme fonksiyonu
    delay(21000); // 7 saniye dur
    geriGit(4.2 * stepsPerRevolution); // Geri gitme fonksiyonunu kontrolle
    delay(3000); // 3 saniye dur

    digitalWrite(motorRolePin2, LOW); // Motor role pini pasif

    digitalWrite(enablePin, HIGH); // Step motoru devre dışı bırak
      
    } else if (digitalRead(inputPins[2]) == HIGH) {
         digitalWrite(motorRolePin2, HIGH); // Motor role pini pasif

        // İkinci durumda işlemler
        ileriGit(4 * stepsPerRevolution); // İleri gitme fonksiyonu
        delay(7000); // 14 saniye dur
        geriGit(4.2 * stepsPerRevolution); // Geri gitme fonksiyonunu kontrolle
        delay(3000); // 3 saniye dur
        digitalWrite(motorRolePin2, LOW); // Motor role pini pasif

        digitalWrite(enablePin, HIGH); // Step motoru devre dışı bırak
        // Diğer giriş pinleri için benzer şekilde işlemleri devam ettirebilirsiniz
    } else if (digitalRead(inputPins[1]) == HIGH) {
        // Üçüncü durumda işlemler
        digitalWrite(motorRolePin2, HIGH); // Motor role pini pasif

        ileriGit(4 * stepsPerRevolution); // İleri gitme fonksiyonu
        delay(14000); // 21 saniye dur
        geriGit(4.2 * stepsPerRevolution); // Geri gitme fonksiyonunu kontrolle
        delay(3000); // 3 saniye dur
        digitalWrite(motorRolePin2, LOW); // Motor role pini pasif

        digitalWrite(enablePin, HIGH); // Step motoru devre dışı bırak
    } else if (digitalRead(inputPins[0]) == HIGH) {
        // Dördüncü durumda işlemler
        digitalWrite(motorRolePin2, HIGH); // Motor role pini pasif

        ileriGit(4 * stepsPerRevolution); // İleri gitme fonksiyonu
        delay(1000);
         while (true) {
            if (digitalRead(inputPins[0]) == HIGH) {
                geriGit(4.2 * stepsPerRevolution);
                break; // İkinci butona basıldı, iç içe döngüden çık
            }
          }
        digitalWrite(motorRolePin2, LOW); // Motor role pini pasif
  
    }
    //------------gyro---------------
    if ( gy > 300) { 


      digitalWrite(motorRolePin, HIGH);
      digitalWrite(motorRolePin3, HIGH);
      
      ileriGit(4 * stepsPerRevolution); // İleri gitme fonksiyonu

      delay(1000); // .. saniye dur

      digitalWrite(enablePin, HIGH); // Step motoru devre dışı bırak
      digitalWrite(motorRolePin, LOW);
      digitalWrite(motorRolePin3, LOW); // Motor role3 pini pasif

    }

  }
}

void geriGit(int stepSayisi) {
  digitalWrite(dirPin, HIGH);   // Geri yön
  digitalWrite(enablePin, LOW); // Step motoru etkinleştir
  digitalWrite(motorRolePin, HIGH); // Motor role pini aktif
  delay(500);
  digitalWrite(motorRolePin, LOW);

  for (int i = 0; i < stepSayisi; i++) {
    digitalWrite(pullPin, HIGH);
    delayMicroseconds(500); // Uygun adım aralığı
    digitalWrite(pullPin, LOW);
    delayMicroseconds(500);
  }
  delay(100);
  
}

void ileriGit(int stepSayisi) {
  digitalWrite(dirPin, LOW);   // İleri yön
  digitalWrite(enablePin, LOW); // Step motoru etkinleştir
  digitalWrite(motorRolePin, HIGH); // Motor role pini aktif
  delay(500);
  digitalWrite(motorRolePin, LOW);
  for (int i = 0; i < stepSayisi; i++) {
    digitalWrite(pullPin, HIGH);
    delayMicroseconds(500); // Uygun adım aralığı
    digitalWrite(pullPin, LOW);
    delayMicroseconds(500);
  }
    
}

/*
void geriGitKontrollu(int stepSayisi, int geriDonusAdimi) {
  digitalWrite(dirPin, HIGH);   // Geri yön
  digitalWrite(enablePin, LOW); // Step motoru etkinleştir
  digitalWrite(motorRolePin1, HIGH); // Motor role pini aktif

  while (stepSayisi > 0) {
    digitalWrite(pullPin, HIGH);
    delayMicroseconds(500); // Uygun adım aralığı
    digitalWrite(pullPin, LOW);
    delayMicroseconds(500);

    // Mesafe sensöründen okuma yap
    delay(1000); 
    long vl53l1x = sensor.read();
    delay(500); 

    // Eğer sensör değeri belirli bir eşik değerinin altına düşerse ileri git
    if (vl53l1x < 1500) { // Örnek bir eşik değeri (300)

      digitalWrite(motorRolePin1, HIGH); // Motor role pini aktif
      digitalWrite(motorRolePin2, HIGH); // Motor role pini aktif
      digitalWrite(motorRolePin3, HIGH); // Motor role pini aktif
      digitalWrite(motorRolePin7, HIGH); // Motor role pini aktif

      ileriGit(stepSayisi - geriDonusAdimi ); // Geri gidilen adım sayısı kadar ileri git
      digitalWrite(motorRolePin1, LOW); // Motor role pini aktif
      digitalWrite(motorRolePin2, LOW); // Motor role pini aktif
      digitalWrite(motorRolePin3, LOW); // Motor role pini aktif
      digitalWrite(motorRolePin7, LOW); // Motor role pini aktif

      //break; // Döngüden çık
    }

    // Geri adım sayısını azalt
    stepSayisi -= 1;
  }

  digitalWrite(motorRolePin1, LOW); // Motor role pini devre dışı bırak
}  */

void geriGitKontrollu(int stepSayisi) {
  digitalWrite(dirPin, HIGH);   // Geri yön
  digitalWrite(enablePin, LOW); // Step motoru etkinleştir
  digitalWrite(motorRolePin1, HIGH); // Motor role pini aktif
  delay(500);

  //int adimMiktari = 1600; // Her adımda gidilecek miktar
  int totalAdim = stepSayisi ; // Toplam adım miktarı

  bool devamEt = true; // Döngünün devam etmesini kontrol etmek için bir bayrak

  while (devamEt) { // Sonsuz döngüyü başlat
    for (int i = 0; i < totalAdim; i++) {

      digitalWrite(dirPin, HIGH);   // Geri yön
      digitalWrite(enablePin, LOW); // Step motoru etkinleştir

      digitalWrite(pullPin, HIGH);
      delayMicroseconds(500); // Uygun adım aralığı
      digitalWrite(pullPin, LOW);
      delayMicroseconds(500);

      // Her 1.1*1600 adımda bir kontrol yap
      if (i % (int)(1.1 * stepsPerRevolution) == 0) {
        // Buraya, sensörden okunan mesafeyi almanız gerekiyor
        delay(100); 
        long vl53l1x = sensor.read();
        delay(100); 

        if (vl53l1x < 1500) {
          // Eğer mesafe 1500'den küçükse bir sonraki 1.1*1600 adımı git
          digitalWrite(motorRolePin1, HIGH); // Motor role pini aktif
          digitalWrite(motorRolePin2, HIGH); // Motor role pini aktif
          digitalWrite(motorRolePin3, HIGH); // Motor role pini aktif
          digitalWrite(motorRolePin7, HIGH); // Motor role pini aktif

          ileriGit(1.1 * stepsPerRevolution ); // Geri gidilen adım sayısı kadar ileri git
          digitalWrite(motorRolePin1, LOW); // Motor role pini aktif
          digitalWrite(motorRolePin2, LOW); // Motor role pini aktif
          digitalWrite(motorRolePin3, LOW); // Motor role pini aktif
          digitalWrite(motorRolePin7, LOW); // Motor role pini aktif
          i = -1; // For döngüsünün bir sonraki adımda 0'dan başlamasını sağlar
          break; // İçteki for döngüsünden çıkarak while döngüsünün başına geri döner
        }
      }

      if (i == totalAdim - 1) {
        devamEt = false; // For döngüsü tamamlandığında sonsuz döngüyü sonlandırır
      }
    }
  }

  delay(100);
  digitalWrite(motorRolePin, LOW);
} 








bool isAnyInputHigh() {
  for (int i = 0; i < sizeof(inputPins) / sizeof(inputPins[0]); i++) {
    if (digitalRead(inputPins[i]) == HIGH) {
      return true;
    }
  }
  return false;
}