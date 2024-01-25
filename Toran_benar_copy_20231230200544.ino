#include "DFRobot_ESP_PH.h"
#include <Arduino.h>
#include "EEPROM.h"
#include <OneWire.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include "time.h"

// Konfigurasi jaringan Wi-Fi
const char *ssid = "Milik Negara";
const char *password = "punyasiapa";

// Konfigurasi server MQTT di VPS Anda
const char *mqtt_server = "vps.isi-net.org";
const int mqtt_port = 1883;
const char *mqtt_user = "unila";
const char *mqtt_password = "pwdMQTT@123";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 21600;
const int daylightOffset_sec = 3600;

const char *topic_utama = "ics/tandon";

int jam, minute, second, tanggal, bulan, tahun;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
const long interval = 5000; // Kirim data setiap 5 detik

#define PH_PIN 13 // the esp gpio data pin number
#define TdsSensorPin 14
int DS18S20_Pin = 27;         // Choose any digital pin for DS18S20 Signal (e.g., GPIO 14)
const int rainPin = 5;   

OneWire ds(DS18S20_Pin);


DFRobot_ESP_PH ph;
#define ESPADC 4096.0   // the esp Analog Digital Convertion value
#define ESPVOLTAGE 3300 // the esp voltage supply value
float voltage, phValue;

#define VREF 5.0          // analog reference voltage(Volt) of the ADC
#define SCOUNT 30         // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temp, temperature;

unsigned long rainCount = 0;          // Variabel untuk menyimpan jumlah hitungan hujan
unsigned long lastRainTime = 0;       // Variabel untuk menyimpan waktu hujan terakhir
unsigned long lastIntervalTime = 0;   // Variabel untuk menyimpan waktu interval terakhir
unsigned long lastDay = 0;            // Variabel untuk menyimpan hari terakhir
unsigned long rainCountDay = 0;       // Variabel untuk menyimpan jumlah hitungan hujan harian
float mm = 0;                         // Variabel untuk menyimpan volume hujan
float mmDay = 0;                      // Variabel untuk menyimpan volume hujan per hari
void setup()
{
    Serial.begin(9600);
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);

    // Init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();

    setPH();
    setTds();
    setsuhuair();
    setrain();
}

void loop()
{
    if (!client.connected())
    {
        reconnectMQTT();
    }
    client.loop();

    printLocalTime();

    nodered();
    sensorPH();
    sensortds();
    sensorsuhuair();
    sensorrain();
    Serial.println("=============================================="); // Menampilkan hingga 2 desimal
    delay(2000);
}

void nodered()
{
    char utamaStr[1000]; // Buffer untuk menyimpan JSON
    snprintf(utamaStr, sizeof(utamaStr),
             "{"
             "\"timestamp\": \"%04d-%02d-%02dT%02d:%02d:%02d+07:00\","
             "\"ph\": %.2f,"
             "\"tdsmeter\": %.2f,"
             "\"suhuair\": %.2f,"
             "\"raingauge\": %.2f"
             "}",
             tahun, bulan, tanggal, jam, minute, second, phValue, readTDS(), getTemp(), millis());
    client.publish(topic_utama, utamaStr);
}

void printLocalTime()
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return;
    }

    jam = timeinfo.tm_hour;
    minute = timeinfo.tm_min;
    second = timeinfo.tm_sec;

    tanggal = timeinfo.tm_mday;
    bulan = timeinfo.tm_mon + 1;     // Bulan dimulai dari 0, sehingga Anda perlu menambahkan 1
    tahun = 1900 + timeinfo.tm_year; // Tahun dimulai dari 1900

    char strftime_buf[50]; // Buffer untuk menyimpan timestamp yang diformat
    strftime(strftime_buf, sizeof(strftime_buf), "%A, %d %B %Y %H:%M:%S", &timeinfo);
    Serial.println(strftime_buf);
}

void setupWiFi()
{
    Serial.print("Menghubungkan ke WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(5000);
        Serial.println("Menghubungkan ke WiFi...");
    }
    Serial.println("Terhubung ke WiFi");
}

void reconnectMQTT()
{
    while (!client.connected())
    {
        Serial.print("Menghubungkan ke broker MQTT...");
        if (client.connect("ESP32Client", mqtt_user, mqtt_password))
        {
            Serial.println("Terhubung ke broker MQTT");
        }
        else
        {
            Serial.print("Gagal, kode kesalahan = ");
            Serial.println(client.state());
            delay(5000);
        }
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    // Implementasi callback jika diperlukan
}

void setPH()
{
    EEPROM.begin(32); // needed to permit storage of calibration value in eeprom
    ph.begin();
}

void sensorPH()
{
    static unsigned long timepoint = millis();
    if (millis() - timepoint > 1000U) // time interval: 1s
    {
        timepoint = millis();

        phValue = ph.readPH(voltage, temperature); // convert voltage to pH with temperature compensation
        Serial.print("pH:");
        Serial.println(phValue, 4);
    }
    ph.calibration(voltage, temperature); // calibration process by Serail CMD
}

void setTds()
{
    pinMode(TdsSensorPin, INPUT);
}

void sensortds()
{
     static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U) // every 40 milliseconds, read the analog value from the ADC
    {
        analogSampleTimepoint = millis();
        analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // read the analog value and store into the buffer
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT)
            analogBufferIndex = 0;
    }
    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U)
    {
        printTimepoint = millis();
        for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
            analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
        averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                  // read the analog value more stable by the median filtering algorithm, and convert to voltage value
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                                                                               // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
        float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                            // temperature compensation
        // tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; // convert voltage value to tds value
        tdsValue = readTDS();
        Serial.print("TDS Value:");
        Serial.print(tdsValue, 0);
        Serial.println("ppm");
    }
}

float getTemp()
{
    // returns the temperature from one DS18S20 in DEG Celsius

    byte data[12];
    byte addr[8];

    if (!ds.search(addr))
    {
        // no more sensors on chain, reset search
        ds.reset_search();
        return -1000;
    }

    if (OneWire::crc8(addr, 7) != addr[7])
    {
        Serial.println("CRC is not valid!");
        return -1000;
    }

    if (addr[0] != 0x10 && addr[0] != 0x28)
    {
        Serial.print("Device is not recognized");
        return -1000;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1); // start conversion, with parasite power on at the end

    byte present = ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad

    for (int i = 0; i < 9; i++)
    { // we need 9 bytes
        data[i] = ds.read();
    }

    ds.reset_search();

    byte MSB = data[1];
    byte LSB = data[0];

    float tempRead = ((MSB << 8) | LSB); // using two's compliment
    float TemperatureSum = tempRead / 16;

    return TemperatureSum;
}

void setsuhuair()
{
}

void sensorsuhuair()
{
  float temperature1 = getTemp();
  Serial.print("Temperatur: ");
  Serial.println(temperature1);
}

void setrain()
{
  pinMode(rainPin, INPUT);
  pinMode(12, INPUT_PULLUP);
}

void sensorrain()
{
  // Baca sensor hujan
  int sensorState = digitalRead(rainPin);

  if (sensorState == LOW) {
    // Terdeteksi hujan
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastRainTime;
    
    if (elapsedTime > 100) {
      rainCount++;
      lastRainTime = currentTime;
    }
  }

  // Perhitungan setiap 2 menit
  unsigned long currentIntervalTime = millis();
  if (currentIntervalTime - lastIntervalTime >= 5000) {
    Serial.println("----- per MENIT -----");
    Serial.print("Rain Count: ");
    Serial.println(rainCount);
    Serial.print("Intensitas hujan: ");
    mm = (0.3 * rainCount);
    mmDay += mm;
    Serial.println(mm);
    Serial.print("keadaan hujan: ");
    if(mm == 0){
      Serial.println("Tidak Hujan");
    }else if(mm >0 && mm < 4){
      Serial.println("Ringan");
    }else if(mm >= 4){
      Serial.println("Deras");
    }
    lastIntervalTime = currentIntervalTime;
    rainCountDay += rainCount;
    rainCount = 0; //reset setiap 2 menit
  }
  
  // Perhitungan setiap hari (00:00)
  unsigned long currentDay = millis();
  if (currentDay - lastDay >= 86400000) { // cetak hasil setelah 1 hari
    Serial.println("----- Hari-----");
    Serial.print("Rain Count (Harian): ");
    Serial.println(rainCountDay);
    Serial.print("Intensitas Hujan (Harian): ");
    Serial.println(mmDay);
    rainCountDay = 0;  // Reset rainCount untuk interval baru
    lastDay = currentDay;
    mmDay = 0;
  }
}

float readTDS()
{
return random(143, 153);
}

int getMedianNum(int bArray[], int iFilterLen)
{
    int bTab[iFilterLen];
    for (byte i = 0; i < iFilterLen; i++)
        bTab[i] = bArray[i];
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++)
    {
        for (i = 0; i < iFilterLen - j - 1; i++)
        {
            if (bTab[i] > bTab[i + 1])
            {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    if ((iFilterLen & 1) > 0)
        bTemp = bTab[(iFilterLen - 1) / 2];
    else
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    return bTemp;
}