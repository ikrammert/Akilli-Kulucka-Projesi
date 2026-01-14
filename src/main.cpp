// Hazırlayan: İkram MERT ikram_mert@hotmail.com
// Fan Sürekli çalıacağından Fan kontrolü yorum satırına alınmıştır.
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
#define BLYNK_AUTH_TOKEN SECRET_BLYNK_AUTH_TOKEN  // Blynk auth token

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
// #define V_WIFI_STATE      V6   // WiFi durum LED
#define V_QUICK1          V7   // Quick Set 1 butonu
#define V_QUICK2          V8   // Quick Set 2 butonu

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
bool lastSentWifiState = false;

// Zamanlama değişkenleri
unsigned long lastSensorMs = 0;
unsigned long lastBlynkSendMs = 0;
unsigned long lastLCDUpdateMs = 0;
unsigned long lastEEPROMSaveMs = 0;
unsigned long wifiReconnectTime = 0;

// Tolerans değerleri (ne kadar fark olursa veri gönderilsin)
const float TEMP_TOLERANCE = 0.3; // Sürekli değişim gürültüsünü azaltmak için
const float HUM_TOLERANCE = 1.0;

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
    lcd.print("W? ");  // WiFi var ama Blynk yok
  } else {
    lcd.print("OFF");
  }
}

// Isıtıcı kontrolü
void controlHeater(float temperature) {
  if (isnan(temperature)) return;
  
  bool shouldHeat = temperature < (setTemperature - 0.5);
  bool shouldStop = temperature > setTemperature;
  
  if (shouldHeat && !lampState) {
    digitalWrite(RELAY1, HIGH);
    lampState = true;
    Serial.println("Isıtıcı açıldı");
  } else if (shouldStop && lampState) {
    digitalWrite(RELAY1, LOW);
    lampState = false;
    Serial.println("Isıtıcı kapatıldı");
  }
}

// Nem kontrolü
void controlHumidifier(float humidity) {
  if (isnan(humidity)) return;
  
  bool shouldHumidify = humidity < (setHumidity - 2.0);
  bool shouldStop = humidity > (setHumidity + 3.0);
  
  if (shouldHumidify && !humidState) {
    digitalWrite(RELAY2, HIGH);
    humidState = true;
    Serial.println("Nem modülü açıldı");
  } else if (shouldStop && humidState) {
    digitalWrite(RELAY2, LOW);
    humidState = false;
    Serial.println("Nem modülü kapatıldı");
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

// Blynk'e veri gönderme (sadece değişiklik varsa)
void sendToBlynk(const SensorData &data) {
  if (!blynkConnected) return;
  
  // Sıcaklık gönder (değişiklik varsa)
  if (!isnan(data.temperature) && abs(data.temperature - lastSentTemp) >= TEMP_TOLERANCE) {
    Blynk.virtualWrite(V_CURRENT_TEMP, data.temperature);
    //Blynk.virtualWrite(V_TEMP_GAUGE, data.temperature);
    lastSentTemp = data.temperature;
  }
  
  // Nem gönder (değişiklik varsa)
  if (!isnan(data.humidity) && abs(data.humidity - lastSentHum) >= HUM_TOLERANCE) {
    Blynk.virtualWrite(V_CURRENT_HUM, data.humidity);
    //Blynk.virtualWrite(V_HUM_GAUGE, data.humidity);
    lastSentHum = data.humidity;
  }
  
  // Hedef değerler gönder (değişiklik varsa)
  if (abs(setTemperature - lastSentSetTemp) >= TEMP_TOLERANCE) {
    Blynk.virtualWrite(V_SET_TEMP, setTemperature);
    lastSentSetTemp = setTemperature;
  }
  
  if (abs(setHumidity - lastSentSetHum) >= HUM_TOLERANCE) {
    Blynk.virtualWrite(V_SET_HUM, setHumidity);
    lastSentSetHum = setHumidity;
  }
  
  // Röle durumları gönder (değişiklik varsa)
  if (lampState != lastSentLampState) {
    Blynk.virtualWrite(V_HEAT_STATE, lampState ? 1 : 0);
    lastSentLampState = lampState;
  }
  
  if (humidState != lastSentHumidState) {
    Blynk.virtualWrite(V_HUM_STATE, humidState ? 1 : 0);
    lastSentHumidState = humidState;
  }
  
  // WiFi durumu gönder (değişiklik varsa)
  if (wifiConnected != lastSentWifiState) {
    //Blynk.virtualWrite(V_WIFI_STATE, wifiConnected ? 1 : 0);
    lastSentWifiState = wifiConnected;
  }
}

// Blynk bağlantı kontrolü
void checkBlynkConnection() {
  blynkConnected = Blynk.connected();
  
  if (!blynkConnected && wifiConnected) {
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > 30000) { // 30 saniyede bir dene
      Serial.println("Blynk'e yeniden bağlanmaya çalışılıyor...");
      Blynk.connect();
      lastReconnectAttempt = millis();
    }
  }
}

// WiFi bağlantı kontrolü
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
  
  // WiFi yoksa yeniden bağlanmaya çalış
  if (!wifiConnected && millis() - wifiReconnectTime > 300000) { // 5 dakikada bir
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
    
    // Butonu sıfırla
    Blynk.virtualWrite(V_QUICK1, 0);
  }
}

BLYNK_WRITE(V_QUICK2) {
  if (param.asInt() == 1) {
    setTemperature = quick2Temp;
    setHumidity = quick2Hum;
    saveToEEPROM();
    Serial.println("Quick Set 2 uygulandı");
    
    // Butonu sıfırla
    Blynk.virtualWrite(V_QUICK2, 0);
  }
}

// Blynk bağlantı olayları
BLYNK_CONNECTED() {
  Serial.println("Blynk'e bağlanıldı!");
  blynkConnected = true;
  
  // İlk bağlantıda tüm verileri gönder
  lastSentTemp = -999;
  lastSentHum = -999;
  lastSentSetTemp = -999;
  lastSentSetHum = -999;
  lastSentLampState = !lampState;
  lastSentHumidState = !humidState;
  lastSentWifiState = !wifiConnected;
}

BLYNK_DISCONNECTED() {
  Serial.println("Blynk bağlantısı kesildi!");
  blynkConnected = false;
}

// Setup 
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
    
    // Blynk başlatma
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect();
  } else {
    Serial.println("\nWiFi bağlantısı başarısız - Offline modda çalışılıyor");
  }
  
  lcd.clear();
  Serial.println("Sistem hazır!");
  // OTA Ayarları
  ArduinoOTA.setHostname("Kulucka-Makinesi-ESP8266");
  //ArduinoOTA.setPassword("pass"); // İstersen şifre koy
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
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
    lcd.print("%" + String(percent));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Hata[%u]: ", error);
    lcd.clear();
    lcd.print("OTA Hatasi!");
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
  Serial.println("OTA hazır!");
}

void loop(){
  unsigned long currentTime=millis();

  // WiFi durumunu kontrol et
  checkWiFiConnection();
  
  // Blynk işlemlerini çalıştır (sadece WiFi bağlıysa)
  if (wifiConnected) {
    ArduinoOTA.handle();
    Blynk.run();
    checkBlynkConnection();
  }

  // Sensör okuma ve kontrol (her 2 saniyede bir)
  if (currentTime - lastSensorMs >= 2000) {
    lastSensorMs = currentTime;
    
    SensorData sensorData = readSensors();
    
    // Kontrol sistemleri (offline da çalışır)
    controlHeater(sensorData.temperature);
    controlHumidifier(sensorData.humidity);
    
    // LCD güncelleme (her seferinde değil, 3 saniyede bir)
    //if (currentTime - lastLCDUpdateMs >= 3000) {
    updateLCD(sensorData);
    //  lastLCDUpdateMs = currentTime;
    //}
    
    // Blynk'e veri gönderme (sadece bağlıysa ve değişiklik varsa)
    if (currentTime - lastBlynkSendMs >= 20000) { // Her 20 saniyede bir (Blynk Veri sınırını aşmamak için)
      sendToBlynk(sensorData);
      lastBlynkSendMs = currentTime;
    }
  }

  delay(10);
}