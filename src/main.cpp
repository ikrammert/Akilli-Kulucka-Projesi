// Hazırlayan: İkram MERT ikram_mert@hotmail.com
// Event Tabanlı - Minimum Blynk Kota Kullanımı
// Sadece sorun olduğunda bildirim gönderir!

#include <ArduinoOTA.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include "secrets.h"

// Blynk kütüphanesi
#define BLYNK_TEMPLATE_ID SECRET_BLYNK_TEMPLATE_ID
#define BLYNK_TEMPLATE_NAME SECRET_BLYNK_TEMPLATE_NAME
#define BLYNK_AUTH_TOKEN SECRET_BLYNK_AUTH_TOKEN

#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp8266.h>

// Wifi Ağı
#define SSID          SECRET_SSID
#define WIFI_PASSWORD SECRET_PASS

// Donanım 
#define DHTPIN D5
#define DHTTYPE DHT11
#define ONE_WIRE_BUS D6
#define RELAY1 D8   // Lamba
#define RELAY2 D7   // Nem modülü

// Blynk Virtual Pin tanımlamaları
#define V_CURRENT_TEMP    V0   // Mevcut sıcaklık
#define V_CURRENT_HUM     V1   // Mevcut nem
#define V_SET_TEMP        V2   // Hedef sıcaklık ayarı
#define V_SET_HUM         V3   // Hedef nem ayarı
#define V_HEAT_STATE      V4   // Isıtıcı durumu LED
#define V_HUM_STATE       V5   // Nem modülü durumu LED
#define V_QUICK1          V7   // Quick Set 1 butonu
#define V_QUICK2          V8   // Quick Set 2 butonu
#define V_REFRESH         V9   // Manuel güncelleme butonu (YENİ!)

LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

// EEPROM 
#define EEPROM_SIZE 64
#define TEMP_ADDR 0
#define HUM_ADDR 10

float EEPROMReadFloat(int addr){
  float x=0; byte *p=(byte*)&x;
  for(int i=0;i<4;i++) *p++=EEPROM.read(addr+i);
  return x;
}

void EEPROMWriteFloat(int addr,float x){
  byte *p=(byte*)&x;
  for(int i=0;i<4;i++) EEPROM.write(addr+i,*p++);
}

// Set değerleri 
float setTemperature = 37.7;
float setHumidity   = 60.0;
float lastSavedTemp = 0;
float lastSavedHum  = 0;

// Quick set
const float quick1Temp = 37.8, quick1Hum = 55.0;
const float quick2Temp = 37.5, quick2Hum = 70.0;

// Sistem durumları
bool wifiConnected = false;
bool blynkConnected = false;
bool lampState = false;
bool humidState = false;

// Son değerler (veri tasarrufu için)
float lastSentTemp = -999;
float lastSentHum = -999;
float lastSentSetTemp = -999;
float lastSentSetHum = -999;
bool lastSentLampState = false;
bool lastSentHumidState = false;

// Alarm durumları (Event için)
bool tempAlarmSent = false;
bool humAlarmSent = false;
unsigned long lastTempAlarmMs = 0;
unsigned long lastHumAlarmMs = 0;
unsigned long lastPeriodicUpdateMs = 0;

// Alarm ayarları
const unsigned long ALARM_COOLDOWN = 600000; // 10 dakika (ms)

// Zamanlama değişkenleri
unsigned long lastSensorMs = 0;
unsigned long wifiReconnectTime = 0;

// Tolerans değerleri
const float TEMP_TOLERANCE = 0.3;
const float HUM_TOLERANCE = 2.0;

// Alarm eşikleri
const float TEMP_ALARM_LOW = 37.0;   // Çok düşük sıcaklık alarmı
const float TEMP_ALARM_HIGH = 39.0;  // Çok yüksek sıcaklık alarmı
const float HUM_ALARM_LOW = 40.0;    // Çok düşük nem alarmı
const float HUM_ALARM_HIGH = 80.0;   // Çok yüksek nem alarmı

struct SensorData {
  float temperature;
  float humidity;
};

// Sensör okuma fonksiyonu
SensorData readSensors() {
  SensorData data;
  data.humidity = dht.readHumidity();
  tempSensor.requestTemperatures();
  data.temperature = tempSensor.getTempCByIndex(0);
  return data;
}

// LCD güncelleme fonksiyonu
void updateLCD(const SensorData &data) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("S:");
  lcd.print(data.temperature, 1);
  lcd.print("/");
  lcd.print(setTemperature, 1);
  
  lcd.setCursor(0, 1);
  lcd.print("N:");
  lcd.print(data.humidity, 0);
  lcd.print("%/");
  lcd.print(setHumidity, 0);
  lcd.print("%");
  
  // WiFi ve Blynk durumu göstergesi
  lcd.setCursor(13, 0);
  if (blynkConnected) {
    lcd.print("ON ");
  } else if (wifiConnected) {
    lcd.print("W? ");
  } else {
    lcd.print("OFF");
  }
}

void controlHeater(float temperature) {
  // Hata kontrolü: -127 veya NAN ise röleyi kapat
  if (isnan(temperature) || temperature == -127.00 || temperature < -100) {
    if (lampState) {
      digitalWrite(RELAY1, LOW);
      lampState = false;
      Serial.println("⚠️ Sensör hatası - Isıtıcı GÜVENLİK KAPAMASI!");
    }
    return;
  }
  
  bool shouldHeat = temperature < (setTemperature - 0.5);
  bool shouldStop = temperature > setTemperature;
  
  if (shouldHeat && !lampState) {
    digitalWrite(RELAY1, HIGH);
    lampState = true;
    Serial.println("🔥 Isıtıcı açıldı");
  } else if (shouldStop && lampState) {
    digitalWrite(RELAY1, LOW);
    lampState = false;
    Serial.println("✅ Isıtıcı kapatıldı");
  }
}

void controlHumidifier(float humidity) {
  // Hata kontrolü: NAN ise röleyi kapat
  if (isnan(humidity) || humidity < 0 || humidity > 100) {
    if (humidState) {
      digitalWrite(RELAY2, LOW);
      humidState = false;
      Serial.println("⚠️ Nem sensörü hatası - Nem modülü GÜVENLİK KAPAMASI!");
    }
    return;
  }
  
  bool shouldHumidify = humidity < (setHumidity - 2.0);
  bool shouldStop = humidity > (setHumidity + 3.0);
  
  if (shouldHumidify && !humidState) {
    digitalWrite(RELAY2, HIGH);
    humidState = true;
    Serial.println("💧 Nem modülü açıldı");
  } else if (shouldStop && humidState) {
    digitalWrite(RELAY2, LOW);
    humidState = false;
    Serial.println("✅ Nem modülü kapatıldı");
  }
}

void saveToEEPROM(){
  if (abs(setTemperature - lastSavedTemp) >= 0.1 || abs(setHumidity - lastSavedHum) >= 1.0){
    EEPROMWriteFloat(TEMP_ADDR, setTemperature);
    EEPROMWriteFloat(HUM_ADDR, setHumidity);
    EEPROM.commit();
    lastSavedTemp=setTemperature;
    lastSavedHum=setHumidity;
  }
}

/* Event kontrolü - FREE PLAN İÇİN Notification kullanıyor!
void checkAndSendEvents(const SensorData &data) {
  if (!blynkConnected) return;
  
  unsigned long currentTime = millis();
  
  // Sıcaklık alarmı kontrolü
  if (!isnan(data.temperature)) {
    bool canSendTempAlarm = (currentTime - lastTempAlarmMs >= ALARM_COOLDOWN);
    
    if (data.temperature < TEMP_ALARM_LOW && canSendTempAlarm) {
      // FREE PLAN: Event yerine Notification kullanıyoruz
      Blynk.notify(String("⚠️ Sıcaklık çok düşük: ") + String(data.temperature, 1) + "°C");
      lastTempAlarmMs = currentTime;
      tempAlarmSent = true;
      Serial.println("⚠️ Düşük sıcaklık alarmı gönderildi! (10dk cooldown)");
    } 
    else if (data.temperature > TEMP_ALARM_HIGH && canSendTempAlarm) {
      Blynk.notify(String("⚠️ Sıcaklık çok yüksek: ") + String(data.temperature, 1) + "°C");
      lastTempAlarmMs = currentTime;
      tempAlarmSent = true;
      Serial.println("⚠️ Yüksek sıcaklık alarmı gönderildi! (10dk cooldown)");
    }
    else if (data.temperature >= TEMP_ALARM_LOW && data.temperature <= TEMP_ALARM_HIGH) {
      if (tempAlarmSent) {
        Serial.println("✅ Sıcaklık normal seviyeye döndü");
        tempAlarmSent = false;
      }
    }
  }
  
  // Nem alarmı kontrolü
  if (!isnan(data.humidity)) {
    bool canSendHumAlarm = (currentTime - lastHumAlarmMs >= ALARM_COOLDOWN);
    
    if (data.humidity < HUM_ALARM_LOW && canSendHumAlarm) {
      Blynk.notify(String("⚠️ Nem çok düşük: ") + String(data.humidity, 0) + "%");
      lastHumAlarmMs = currentTime;
      humAlarmSent = true;
      Serial.println("⚠️ Düşük nem alarmı gönderildi! (10dk cooldown)");
    }
    else if (data.humidity > HUM_ALARM_HIGH && canSendHumAlarm) {
      Blynk.notify(String("⚠️ Nem çok yüksek: ") + String(data.humidity, 0) + "%");
      lastHumAlarmMs = currentTime;
      humAlarmSent = true;
      Serial.println("⚠️ Yüksek nem alarmı gönderildi! (10dk cooldown)");
    }
    else if (data.humidity >= HUM_ALARM_LOW && data.humidity <= HUM_ALARM_HIGH) {
      if (humAlarmSent) {
        Serial.println("✅ Nem normal seviyeye döndü");
        humAlarmSent = false;
      }
    }
  }
}*/

// Periyodik güncelleme (günde 2 kez - sabah/akşam)
void sendPeriodicUpdate(const SensorData &data) {
  if (!blynkConnected) return;
  
  unsigned long currentTime = millis();
  // 12 saatte bir güncelleme (43200000 ms)
  if (currentTime - lastPeriodicUpdateMs < 43200000) return;
  lastPeriodicUpdateMs = currentTime;
  
  Serial.println("Periyodik güncelleme gönderiliyor...");
  
  if (!isnan(data.temperature)) {
    Blynk.virtualWrite(V_CURRENT_TEMP, data.temperature);
    lastSentTemp = data.temperature;
  }
  
  if (!isnan(data.humidity)) {
    Blynk.virtualWrite(V_CURRENT_HUM, data.humidity);
    lastSentHum = data.humidity;
  }
  
  Blynk.virtualWrite(V_SET_TEMP, setTemperature);
  Blynk.virtualWrite(V_SET_HUM, setHumidity);
  Blynk.virtualWrite(V_HEAT_STATE, lampState ? 1 : 0);
  Blynk.virtualWrite(V_HUM_STATE, humidState ? 1 : 0);
  
  lastSentSetTemp = setTemperature;
  lastSentSetHum = setHumidity;
  lastSentLampState = lampState;
  lastSentHumidState = humidState;
}

// Manuel güncelleme (Kullanıcı butona bastığında)
void sendManualUpdate(const SensorData &data) {
  if (!blynkConnected) return;
  
  Serial.println("Manuel güncelleme yapılıyor...");
  
  if (!isnan(data.temperature)) {
    Blynk.virtualWrite(V_CURRENT_TEMP, data.temperature);
  }
  
  if (!isnan(data.humidity)) {
    Blynk.virtualWrite(V_CURRENT_HUM, data.humidity);
  }
  
  Blynk.virtualWrite(V_SET_TEMP, setTemperature);
  Blynk.virtualWrite(V_SET_HUM, setHumidity);
  Blynk.virtualWrite(V_HEAT_STATE, lampState ? 1 : 0);
  Blynk.virtualWrite(V_HUM_STATE, humidState ? 1 : 0);
}

void checkBlynkConnection() {
  blynkConnected = Blynk.connected();
  
  if (!blynkConnected && wifiConnected) {
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > 30000) {
      Serial.println("Blynk'e yeniden bağlanmaya çalışılıyor...");
      Blynk.connect();
      lastReconnectAttempt = millis();
    }
  }
}

void checkWiFiConnection() {
  bool currentWifiState = (WiFi.status() == WL_CONNECTED);
  
  if (currentWifiState != wifiConnected) {
    wifiConnected = currentWifiState;
    if (wifiConnected) {
      Serial.println("WiFi bağlandı: " + WiFi.localIP().toString());
    } else {
      Serial.println("WiFi bağlantısı kesildi");
      blynkConnected = false;
    }
  }
  
  if (!wifiConnected && millis() - wifiReconnectTime > 300000) {
    Serial.println("WiFi'ye yeniden bağlanmaya çalışılıyor...");
    WiFi.begin(SSID, WIFI_PASSWORD);
    wifiReconnectTime = millis();
  }
}

// Blynk Virtual Pin fonksiyonları
BLYNK_WRITE(V_SET_TEMP) {
  float newTemp = param.asFloat();
  setTemperature = newTemp;
  saveToEEPROM();
  Serial.println("Yeni hedef sıcaklık: " + String(setTemperature));
}

BLYNK_WRITE(V_SET_HUM) {
  float newHum = param.asFloat();
  setHumidity = newHum;
  saveToEEPROM();
  Serial.println("Yeni hedef nem: " + String(setHumidity));
}

BLYNK_WRITE(V_QUICK1) {
  if (param.asInt() == 1) {
    setTemperature = quick1Temp;
    setHumidity = quick1Hum;
    saveToEEPROM();
    Serial.println("Quick Set 1 uygulandı");
    Blynk.virtualWrite(V_QUICK1, 0);
  }
}

BLYNK_WRITE(V_QUICK2) {
  if (param.asInt() == 1) {
    setTemperature = quick2Temp;
    setHumidity = quick2Hum;
    saveToEEPROM();
    Serial.println("Quick Set 2 uygulandı");
    Blynk.virtualWrite(V_QUICK2, 0);
  }
}

// YENİ: Manuel güncelleme butonu
BLYNK_WRITE(V_REFRESH) {
  if (param.asInt() == 1) {
    SensorData currentData = readSensors();
    sendManualUpdate(currentData);
    Serial.println("Manuel güncelleme tamamlandı");
    Blynk.virtualWrite(V_REFRESH, 0); // Butonu sıfırla
  }
}

BLYNK_CONNECTED() {
  Serial.println("Blynk'e bağlanıldı!");
  blynkConnected = true;
  
  // İlk bağlantıda hemen güncelle
  lastSentTemp = -999;
  lastSentHum = -999;
  lastSentSetTemp = -999;
  lastSentSetHum = -999;
  lastSentLampState = !lampState;
  lastSentHumidState = !humidState;
}

BLYNK_DISCONNECTED() {
  Serial.println("Blynk bağlantısı kesildi!");
  blynkConnected = false;
}

void setup(){
  Serial.begin(9600);
  pinMode(RELAY1,OUTPUT); pinMode(RELAY2,OUTPUT);
  digitalWrite(RELAY1,LOW); digitalWrite(RELAY2,LOW);

  lcd.init(); lcd.backlight();
  lcd.setCursor(0,0); lcd.print("Kulucka Basliyor");
  lcd.setCursor(0, 1); lcd.print("Lutfen bekleyin");

  dht.begin(); tempSensor.begin();

  EEPROM.begin(EEPROM_SIZE);
  float savedTemp = EEPROMReadFloat(TEMP_ADDR);
  float savedHum = EEPROMReadFloat(HUM_ADDR);
  if(!isnan(savedTemp)&&savedTemp>0) setTemperature=savedTemp;
  if(!isnan(savedHum)&&savedHum>0) setHumidity=savedHum;
  lastSavedTemp=setTemperature; lastSavedHum=setHumidity;

  WiFi.begin(SSID,WIFI_PASSWORD);
  unsigned long wifiStartTime = millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-wifiStartTime<10000){
    delay(500); Serial.print(".");
  }
  wifiConnected=(WiFi.status()==WL_CONNECTED);
  
  if (wifiConnected) {
    Serial.println("\nWiFi bağlandı: " + WiFi.localIP().toString());
    
    // OTA Ayarları
    ArduinoOTA.setHostname("Kulucka-Makinesi-ESP8266");
    
    ArduinoOTA.onStart([]() {
      String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
      Serial.println("OTA Başladı: " + type);
      lcd.clear();
      lcd.print("OTA Yukleniyor");
    });
    
    ArduinoOTA.onEnd([]() {
      Serial.println("\nOTA Tamamlandı");
      lcd.clear();
      lcd.print("OTA Basarili!");
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      int percent = (progress / (total / 100));
      Serial.printf("İlerleme: %u%%\r", percent);
      lcd.setCursor(0, 1);
      lcd.print("%");
      lcd.print(percent);
      lcd.print("  ");
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("OTA Hata[%u]: ", error);
      lcd.clear();
      lcd.print("OTA Hatasi!");
    });
    
    ArduinoOTA.begin();
    Serial.println("OTA hazır!");
    
    // Blynk başlatma
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect();
  } else {
    Serial.println("\nWiFi bağlantısı başarısız - Offline modda çalışılıyor");
  }
  
  lcd.clear();
  Serial.println("Sistem hazır!");
}

void loop(){
  unsigned long currentTime=millis();

  checkWiFiConnection();
  
  if (wifiConnected) {
    ArduinoOTA.handle();
    Blynk.run();
    checkBlynkConnection();
  }

  // Sensör okuma - 2 saniyede bir
  if (currentTime - lastSensorMs >= 2000) {
    lastSensorMs = currentTime;
    
    SensorData sensorData = readSensors();
    
    // Kontrol sistemleri (her zaman çalışır)
    controlHeater(sensorData.temperature);
    Serial.println("Sıcaklık: " + String(sensorData.temperature,1) + "°C, Nem: " + String(sensorData.humidity,0) + "%");
    controlHumidifier(sensorData.humidity);
    
    // LCD her sensör okumasında güncellenir
    updateLCD(sensorData);
    
    // Event kontrolü (sorun varsa bildirim gönder)
    //checkAndSendEvents(sensorData);
    
    // Periyodik güncelleme (12 saatte bir)
    sendPeriodicUpdate(sensorData);
  }

  delay(10);
}