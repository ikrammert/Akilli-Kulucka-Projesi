// Hazırlayan: İkram MERT ikram_mert@hotmail.com
// Fan Sürekli çalıacağından Fan kontrolü yorum satırına alınmıştır.


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

#include <ESP8266mDNS.h>  // mDNS için
const char* hostname = "kuluckamakinesi";  // mDNS için (.local erişimi)

// WiFi Ayarları
const char* ssid = "ssid"; // Kendi SSID'nizi girin
const char* password = "password"; // Kendi şifrenizi girin

ESP8266WebServer server(80);

bool wifiConnected = false;
IPAddress local_IP(192, 168, 1, 185);    // Sabit IP adresi
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   // İsteğe bağlı
IPAddress secondaryDNS(8, 8, 4, 4); // İsteğe bağlı


// EEPROM ayarları
#define EEPROM_SIZE 64
#define TEMP_ADDR 0
#define HUM_ADDR 10

// Quick Set değerleri
const float quickSet1Temp = 37.8;
const float quickSet1Humidity = 55.0;
const float quickSet2Temp = 37.5;
const float quickSet2Humidity = 70.0;

// LCD Ayarları
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Sensörler
#define DHTPIN D5
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define ONE_WIRE_BUS D6
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

struct SensorData {
  float humidity;
  float tempDHT;
  float tempDS;
};

// Röle Pinleri
#define RELAY1 D7   // Ampul
#define RELAY2 D4   // Nem Modülü
#define TPIN D8     // Nem Modülü Trigger
//#define FANPIN D3   // Fan

// Set Değerleri
float setHumidity;
float setTemperature;

float lastWrittenTemp = 0;
float lastWrittenHum = 0;

// Grafik verileri için
const int maxDataPoints = 20;
float tempHistory[maxDataPoints] = {0};
float humHistory[maxDataPoints] = {0};
int dataIndex = 0;

float EEPROMReadFloat(int addr) {
  float x = 0.0;
  byte* ptr = (byte*)(void*)&x;
  for (int i = 0; i < 4; i++) {
    *ptr++ = EEPROM.read(addr + i);
  }
  return x;
}

void EEPROMWriteFloat(int addr, float x) {
  byte* ptr = (byte*)(void*)&x;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(addr + i, *ptr++);
  }
  
}

void saveToEEPROM() {
  // Sadece değerler değişmisse yaz.
  if (abs(setTemperature - lastWrittenTemp) >= 0.1 || abs(setHumidity - lastWrittenHum) >= 1.0) {
    ESP.wdtDisable();
    EEPROMWriteFloat(TEMP_ADDR, setTemperature);
    EEPROMWriteFloat(HUM_ADDR, setHumidity);
    if (EEPROM.commit()) {
      lastWrittenTemp = setTemperature;
      lastWrittenHum = setHumidity;
      Serial.println("EEPROM'a yazıldı");
    } else {
      Serial.println("EEPROM yazma hatası!");
    }
    ESP.wdtEnable(WDTO_8S);
  }
}

SensorData readSensors() {
  SensorData data;
  data.humidity = dht.readHumidity();
  data.tempDHT = dht.readTemperature();
  tempSensor.requestTemperatures();
  data.tempDS = tempSensor.getTempCByIndex(0);

  // Verileri grafik için kaydet
  tempHistory[dataIndex] = data.tempDS;
  humHistory[dataIndex] = data.humidity;
  dataIndex = (dataIndex + 1) % maxDataPoints;

  Serial.println("----- SENSOR VERILERI -----");
  Serial.print("DHT11 Sicaklik: "); Serial.print(data.tempDHT); Serial.print(" *C | Nem: "); Serial.print(data.humidity); Serial.println(" %");
  Serial.print("DS18B20 Sicaklik: "); Serial.println(data.tempDS);

  return data;
}

void handleRoot() {
  SensorData sensors = readSensors();
  
  String html = R"=====(
  <!DOCTYPE html>
  <html lang="tr">
  <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Kuluçka Makinesi Kontrol Paneli</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css" rel="stylesheet">
    <style>
      body {
        padding: 20px;
        background-color: #f8f9fa;
      }
      .card {
        margin-bottom: 20px;
        box-shadow: 0 4px 8px rgba(0,0,0,0.1);
      }
      .card-header {
        font-weight: bold;
      }
      .real-time-value {
        font-size: 1.5rem;
        font-weight: bold;
      }
      .chart-container {
        position: relative;
        height: 300px;
        margin-bottom: 20px;
      }
      .btn-quick {
        width: 100%;
        margin-bottom: 10px;
      }
    </style>
  </head>
  <body>
    <div class="container">
      <h1 class="text-center mb-4">Kuluçka Makinesi Kontrol Paneli</h1>
      
      <div class="row">
        <div class="col-md-6">
          <div class="card">
            <div class="card-header bg-primary text-white">
              Anlık Değerler
            </div>
            <div class="card-body">
              <div class="row">
                <div class="col-6">
                  <p>Sıcaklık</p>
                  <div class="real-time-value" id="currentTemp">)=====";
  html += String(sensors.tempDS, 1);
  html += R"=====( °C</div>
                </div>
                <div class="col-6">
                  <p>Nem</p>
                  <div class="real-time-value" id="currentHum">)=====";
  html += String(sensors.humidity, 0);
  html += R"=====( %</div>
                </div>
              </div>
            </div>
          </div>
        </div>
        
        <div class="col-md-6">
          <div class="card">
            <div class="card-header bg-success text-white">
              Hedef Değerler
            </div>
            <div class="card-body">
              <div class="row">
                <div class="col-6">
                  <p>Hedef Sıcaklık</p>
                  <div class="real-time-value">)=====";
  html += String(setTemperature, 1);
  html += R"=====( °C</div>
                </div>
                <div class="col-6">
                  <p>Hedef Nem</p>
                  <div class="real-time-value">)=====";
  html += String(setHumidity, 0);
  html += R"=====( %</div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
      
      <div class="row mt-4">
        <div class="col-md-8">
          <div class="card">
            <div class="card-header bg-info text-white">
              Sensör Geçmişi
            </div>
            <div class="card-body">
              <div class="chart-container">
                <canvas id="sensorChart"></canvas>
              </div>
            </div>
          </div>
        </div>
        
        <div class="col-md-4">
          <div class="card">
            <div class="card-header bg-warning text-dark">
              Hızlı Ayarlar
            </div>
            <div class="card-body">
              <button onclick="location.href='/quickset1'" class="btn btn-primary btn-quick">Başlangıç Modu<br>37.8°C / 55%</button>
              <button onclick="location.href='/quickset2'" class="btn btn-success btn-quick">Çıkım Modu<br>37.5°C / 70%</button>
              
              <hr>
              
              <h5>Manuel Ayarlar</h5>
              <form action="/set" method="POST" onsubmit="return validateForm()">
                <div class="mb-3">
                  <label for="temp" class="form-label">Yeni Sıcaklık (°C)</label>
                  <input type="number" step="0.1" class="form-control" id="temp" name="temp" required>
                </div>
                <div class="mb-3">
                  <label for="hum" class="form-label">Yeni Nem (%)</label>
                  <input type="number" step="1" class="form-control" id="hum" name="hum" required>
                </div>
                <button type="submit" class="btn btn-primary w-100">Güncelle</button>
              </form>
            </div>
          </div>
        </div>
      </div>
    </div>
    
    <script>
      // Grafik oluşturma
      const ctx = document.getElementById('sensorChart').getContext('2d');
      const chart = new Chart(ctx, {
        type: 'line',
        data: {
          labels: Array.from({length: )=====";
  html += String(maxDataPoints);
  html += R"=====(}, (_, i) => i),
          datasets: [
            {
              label: 'Sıcaklık (°C)',
              data: [)=====";
  for (int i = 0; i < maxDataPoints; i++) {
    html += String(tempHistory[i]);
    if (i < maxDataPoints - 1) html += ",";
  }
  html += R"=====(],
              borderColor: 'rgba(255, 99, 132, 1)',
              backgroundColor: 'rgba(255, 99, 132, 0.2)',
              tension: 0.1,
              yAxisID: 'y'
            },
            {
              label: 'Nem (%)',
              data: [)=====";
  for (int i = 0; i < maxDataPoints; i++) {
    html += String(humHistory[i]);
    if (i < maxDataPoints - 1) html += ",";
  }
  html += R"=====(],
              borderColor: 'rgba(54, 162, 235, 1)',
              backgroundColor: 'rgba(54, 162, 235, 0.2)',
              tension: 0.1,
              yAxisID: 'y1'
            }
          ]
        },
        options: {
          responsive: true,
          interaction: {
            mode: 'index',
            intersect: false,
          },
          scales: {
            y: {
              type: 'linear',
              display: true,
              position: 'left',
              title: {
                display: true,
                text: 'Sıcaklık (°C)'
              }
            },
            y1: {
              type: 'linear',
              display: true,
              position: 'right',
              grid: {
                drawOnChartArea: false,
              },
              title: {
                display: true,
                text: 'Nem (%)'
              }
            }
          }
        }
      });
      
      // Form doğrulama
      function validateForm() {
        const temp = parseFloat(document.getElementById('temp').value);
        const hum = parseFloat(document.getElementById('hum').value);
        
        if (temp < 30 || temp > 40) {
          alert('Sıcaklık 30-40°C arasında olmalıdır!');
          return false;
        }
        
        if (hum < 30 || hum > 90) {
          alert('Nem 30-90% arasında olmalıdır!');
          return false;
        }
        
        return true;
      }
      
      // Gerçek zamanlı veri güncelleme
      function updateSensorData() {
        fetch('/sensor-data')
          .then(response => response.json())
          .then(data => {
            document.getElementById('currentTemp').textContent = data.temp.toFixed(1) + ' °C';
            document.getElementById('currentHum').textContent = data.hum.toFixed(0) + ' %';
            
            // Grafik verilerini güncelle
            chart.data.datasets[0].data.push(data.temp);
            chart.data.datasets[1].data.push(data.hum);
            
            // Eski verileri sil
            if (chart.data.datasets[0].data.length > )=====";
  html += String(maxDataPoints);
  html += R"=====() {
              chart.data.datasets[0].data.shift();
              chart.data.datasets[1].data.shift();
            }
            
            chart.update();
          })
          .catch(error => console.error('Veri alınamadı:', error));
      }
      
      // Her 5 saniyede bir verileri güncelle
      setInterval(updateSensorData, 5000);
    </script>
  </body>
  </html>
  )=====";

  server.send(200, "text/html", html);
}

void handleSensorData() {
  SensorData sensors = readSensors();
  
  String json = "{";
  json += "\"temp\":" + String(sensors.tempDS) + ",";
  json += "\"hum\":" + String(sensors.humidity);
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleSetValues() {
  if (server.hasArg("temp") && server.hasArg("hum")) {
    setTemperature = server.arg("temp").toFloat();
    setHumidity = server.arg("hum").toFloat();
    saveToEEPROM();
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleQuickSet1() {
  setTemperature = quickSet1Temp;
  setHumidity = quickSet1Humidity;
  saveToEEPROM();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleQuickSet2() {
  setTemperature = quickSet2Temp;
  setHumidity = quickSet2Humidity;
  saveToEEPROM();
  server.sendHeader("Location", "/");
  server.send(303);
}

// mDNS ve WiFi bağlantı iyileştirmesi
void setupWiFi() {
  WiFi.disconnect(true);  // Önceki bağlantıları temizle
  delay(1000);
  
  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostname);  // Cihaz hostname ayarla
  
  // Opsiyonel: Eğer statik IP kullanmak istiyorsanız (DHCP yerine)
  /*
  IPAddress local_IP(192, 168, 1, 185);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Statik IP atanamadı! DHCP kullanılacak");
  }
  */

  Serial.println("WiFi bağlantısı yapılıyor...");
  lcd.setCursor(0, 0);
  lcd.print("WiFi Baglaniyor");
  lcd.setCursor(0, 1);
  lcd.print("Lutfen Bekleyin");

  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(250);
    Serial.print(".");
    lcd.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nWiFi'ya bağlandı!");
    Serial.print("IP Adresi: "); Serial.println(WiFi.localIP());
    Serial.print("Hostname: "); Serial.println(hostname);

    // mDNS başlatma
    if (MDNS.begin(hostname)) {
      Serial.println("mDNS başlatıldı: http://" + String(hostname) + ".local");
    } else {
      Serial.println("mDNS başlatılamadı!");
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Baglandi");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP().toString());

    // Web Server Ayarları
    server.on("/", handleRoot);
    server.on("/sensor-data", handleSensorData);
    server.on("/set", HTTP_POST, handleSetValues);
    server.on("/quickset1", handleQuickSet1);
    server.on("/quickset2", handleQuickSet2);
    server.begin();
    Serial.println("Web server başlatıldı");
    
    // mDNS'ye servis ekleme
    MDNS.addService("http", "tcp", 80);
    
  } else {
    wifiConnected = false;
    WiFi.disconnect();
    Serial.println("\nWiFi bağlantısı başarısız! Offline moda geçiliyor");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Baglanamadi");
    lcd.setCursor(0, 1);
    lcd.print("Offline Mod...");
    delay(2000);
  }
}

void setup() {
  Serial.begin(115200);

  // Röle ve Fan Pinleri
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(TPIN, OUTPUT);
  //pinMode(FANPIN, OUTPUT);

  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);
  digitalWrite(TPIN, LOW);
  //digitalWrite(FANPIN, LOW);

  // LCD Başlat
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Sensör Başlat
  dht.begin();
  tempSensor.begin();

  // EEPROM Başlat ve Değerleri Oku
  EEPROM.begin(EEPROM_SIZE);
  setTemperature = EEPROMReadFloat(TEMP_ADDR);
  setHumidity = EEPROMReadFloat(HUM_ADDR);
  lastWrittenTemp = setTemperature;
  lastWrittenHum = setHumidity;

  if (isnan(setTemperature) || setTemperature == 0.0) {
    setTemperature = 37.5;
    EEPROMWriteFloat(TEMP_ADDR, setTemperature);
  }
  
  if (isnan(setHumidity) || setHumidity == 0.0) {
    setHumidity = 60.0;
    EEPROMWriteFloat(HUM_ADDR, setHumidity);
  }  

  setupWiFi();

  // Watchdog Timer WDT
  ESP.wdtDisable(); // Önce WDT'yi kapat
  ESP.wdtEnable(WDTO_8S); // 8 saniyelik WDT ayarla
}

void writeLCD(const SensorData& sensor) {
  lcd.setCursor(0, 0);
  lcd.print("Sicaklik:");
  lcd.print(sensor.tempDS, 2);
  lcd.print("C ");

  lcd.setCursor(0, 1);
  lcd.print("Nem:     ");
  lcd.print(sensor.humidity, 0);
  lcd.print("%");
}

void controlRelay1(float temp) {
  static bool relayState = false;

  if (temp < (setTemperature - 1.3) && !relayState) {
    digitalWrite(RELAY1, HIGH);
    relayState = true;
    Serial.println(">> Röle AÇILDI (Sıcaklık düşük)");
  } 
  else if (temp > (setTemperature) && relayState) {
    digitalWrite(RELAY1, LOW);
    relayState = false;
    Serial.println(">> Röle KAPANDI (Sıcaklık yüksek)");
  }
}

void triggerButton() {
  digitalWrite(TPIN, HIGH);
  delay(100);
  digitalWrite(TPIN, LOW);
}

void controlHumidifier(float humidity) {
  static bool relayState2 = false;

  if (humidity < (setHumidity) && !relayState2) {
    digitalWrite(RELAY2, HIGH);
    relayState2 = true;
    Serial.println(">> Nem Modülü AÇILDI (Nem düşük)");
  } 
  else if (humidity > (setHumidity + 5) && relayState2) {
    digitalWrite(RELAY2, LOW);
    relayState2 = false;
    Serial.println(">> Nem Modülü KAPANDI (Nem yüksek)");
  }
}

void loop() {
  if (wifiConnected && WiFi.status() != WL_CONNECTED) {
    // Bağlantı kopmuş!
    Serial.println("WiFi baglantisi koptu! Offline moda geciliyor...");

    wifiConnected = false;
    server.close();    // WebServer durduruluyor
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Yok");
    lcd.setCursor(0, 1);
    lcd.print("Offline Mod...");
    ESP.wdtFeed(); // Her saniye WDT'yi besle
    delay(2000);
    ESP.wdtFeed(); // Her saniye WDT'yi besle
  }

  if (wifiConnected) {
    server.handleClient();
    MDNS.update();  // mDNS güncellemesi
  }
  ESP.wdtFeed(); // Her saniye WDT'yi besle
  delay(2000);
  ESP.wdtFeed(); // Her saniye WDT'yi besle

  //digitalWrite(FANPIN, HIGH);

  // Sensör okuma ve röle kontrol fonksiyonları burada devam edecek
  SensorData sensor = readSensors();
  writeLCD(sensor);
  controlRelay1(sensor.tempDS);
  controlHumidifier(sensor.humidity);
}