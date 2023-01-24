#include <Arduino.h>
#include <time.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "heltec.h"
#include <Wire.h>
#include "oled/SSD1306Wire.h"
#include <sys/time.h>
#include <FlowMeter.h>
#include <ESPDateTime.h>
#include <WiFi.h>
#include <stdio.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "ifpb_frame.h"
#include "gcompii_frame.h"

#define BAND 917E6 // you can set band here directly,e.g. 868E6,915E6

#define Pressao_Nivel_Mar_HPA (1013.25) //pressão nivel do mar padrão
#define Pin_Sensor_FS400A 17
#define Pin_Sensor_DS18B20 23
#define Pin_Rele 13
#define WIFI_SSID "SMMIA"
#define WIFI_PASS "SMMIA"

// ESP32 Touch Test
// Just test touch pin - Touch0 is T0 which is on GPIO 4.
// struct tm RTC_DATA_HORA; // Cria a estrutura que contem as informacoes da data.
// ClosedCube_HDC1080 hdc1080;
Adafruit_ADS1115 ADS_1X15;
SSD1306Wire display(0x3C, SDA_OLED, SCL_OLED, RST_OLED, GEOMETRY_128_64);
Adafruit_BME280 bme; 
const int Hora_Solenoide_OFF = 23; // hora para verificar se há vazão (possivel vazamento ou esquecimento de torneiras abertas), caso aja, fecha o solenoide
const int Hora_Solenoide_ON = 6;   // hora para liberar o solenoide
const int oneWireBus = Pin_Sensor_DS18B20;
boolean Status_Solenoide = false;             // inicializa o status do solenoide
long Intervalo_Envio_LoRa = 15000;            // one second (in milliseconds)
unsigned long Tempo_Atualizacao_Vazao = 2000; // Tempo utilizado pela bilioteca Flowmeter para contabilizar os pulsos (contabiliza pulsos somados a cada 2s)
long Tempo_Decorrido = 0;
const float offset = 0.05; // Offset de tensão do sensor de pressão
unsigned int Tx_LoRa = 0;   // Número de pacote LoRa

FlowSensorProperties Sensor_FS400A = {60.0f, 4.8f, {0.9695, 1.0255, 1.0350, 1.0250, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
FlowMeter *Sensor_Vazao;
OneWire oneWire(oneWireBus);
DallasTemperature Sensor_Temp(&oneWire);
hw_timer_t *timer = NULL;

void Setup_Parametros_LoRa();
void Setup_WiFi();
void Setup_Date_Time();
void logos();

void ISR_Sensor_Vazao()
{
  // let our flow meter count the pulses
  Sensor_Vazao->count();
}

void setup()
{
  Serial.begin(115200);
  delay(1000); // give me time to bring up serial monitor
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  if (!bme.begin(0x76,&Wire)) {
    Serial.println(F("Sensor BME280 não encontrado!"));
    while (1) delay(10);
  }
  Serial.println("Inicializando SETUP...");
  display.init();
  display.clear();
  display.flipScreenVertically();
  pinMode(Pin_Rele, OUTPUT);
  digitalWrite(Pin_Rele, HIGH);
  Setup_Parametros_LoRa();
  logos();
  Setup_WiFi();
  Setup_Date_Time();
  Sensor_Temp.begin();
  Sensor_Vazao = new FlowMeter(digitalPinToInterrupt(Pin_Sensor_FS400A), Sensor_FS400A, ISR_Sensor_Vazao, RISING);
  ADS_1X15.begin(0x48);
  ADS_1X15.setDataRate(250); // 860
}

void loop()
{
  float te = 0;   // Temperatura externa
  float um = 0;   // Umidade ambiente
  float pa = 0;   // Pressão Atmosferica
  float ta = 0;   // Temperatura ambietne
  float mpa = 0;  // Pressão em Mpa
  float bar = 0;  // Pressão em Bar
  float mca = 0;  // Pressão em Metros de coluna de água (MCA)
  double vi = 0;  // Vazão Instantanea em l/min
  double vt = 0;  // Volume total contabilizado em litros
  double hz = 0;  // Frequencia de rotação da aleta do sensor
  double er = 0;  // Percentual de Erro aplicado para faixa de vazao
  double mch = 0; // metro cubico hora (m³/h)
  double mct = 0; // metro cubico total (m³)
  int16_t adc0 = 0;
  float adc0_volts = 0.0;
  char converte_num[4];
  adc0 = ADS_1X15.readADC_Differential_0_1(); // verifica a pressão no sensor
  adc0_volts = ADS_1X15.computeVolts(adc0);   // converte para o valor /offset 0.54V
  sprintf(converte_num, "%.3f", adc0_volts);  // truca o numero em duas casas decimais
  adc0_volts = strtof(converte_num, NULL);
  adc0_volts -= offset;
  mpa = ((4 * adc0_volts - 1.6) / 15);
  bar = (mpa * 10);
  if (bar > 3.0) 
  {
    bar = (mpa * 10) + 0.6;
  }
  else if ((bar > 2.0) and (bar <3.0)) 
  {
    bar = (mpa * 10) + 0.3;
  }
  else if ((bar > 1.0)  and (bar <2.0)) 
  {
    bar = (mpa * 10) + 0.3;
  }
  else if ((bar > 0.0)  and (bar <0.25)) 
  {
    bar = 0.0;
  }
  mca = bar * 10.1974;
  
  ta = bme.readTemperature();
  pa = (bme.readPressure() / 100.0F);
  um = bme.readHumidity();

  Serial.print("Temperatura = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  Serial.print("Pressao = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");
  
  Serial.print("Umidade = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Sensor_Temp.requestTemperatures();
  Sensor_Vazao->tick(Tempo_Atualizacao_Vazao);
  te = Sensor_Temp.getTempCByIndex(0);
  vi = Sensor_Vazao->getCurrentFlowrate();
  vt = Sensor_Vazao->getTotalVolume();
  hz = Sensor_Vazao->getCurrentFrequency();
  er = Sensor_Vazao->getCurrentError();
  mch = vi / 16.667; // converte para m³/h
  mct = vt / 1000;   // converte para m³
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(15, 0, String(DateTime.toString()));
  display.drawString(0, 10, "Solenoide: " + String(Status_Solenoide) + " LoRa: " + String(Tx_LoRa));
  display.drawString(0, 20, "T: " + String(te) + " / " + String(ta) + " ºC" + " U:" + String(um) + "%" );
  display.drawString(0, 30, "Vazão: " + String(vi) + " l/min   " + String(vt) + " l");
  display.drawString(0, 40, "Vazão: " + String(mch) + " m³/h  " + String(mct) + " m³");
  display.drawString(0, 50,"Pressão: " + String(bar) + "BAR " + String(mca) + "MCA ");
  display.display();
  if (millis() - Tempo_Decorrido >= Intervalo_Envio_LoRa) // Envia pacotes a cada 15 segundos
  {
    Tx_LoRa++;
    LoRa.beginPacket();
    LoRa.print("<");
    LoRa.print("1"); // Canal MQTT utilizado no Dragino utiliza as tags "<
    LoRa.print(">");
    LoRa.print("{");        // Formato do payload Dojot {"v
    LoRa.print("\"tx\": "); // Numero Pacote Lora
    LoRa.print(String(Tx_LoRa));
    LoRa.print(",");
    LoRa.print("\"te\": "); // temperatura externa (ºC)
    LoRa.print(String(te));
    LoRa.print(",");
    LoRa.print("\"vi\": "); // vazão instananea (l/min)
    LoRa.print(String(vi, 3));
    LoRa.print(",");
    LoRa.print("\"vt\": "); // Volume total em Litros (l)
    LoRa.print(String(vt, 3));
    LoRa.print(",");
    LoRa.print("\"mch\": ");    // vazão instananea (m³/h)
    LoRa.print(String(mch, 3)); //
    LoRa.print(",");
    LoRa.print("\"mct\": ");    // volume total (m³)
    LoRa.print(String(mct, 3)); //
    LoRa.print(",");
    LoRa.print("\"hz\": "); // Velocidade da aleta do sensor (Hz)
    LoRa.print(String(hz));
    LoRa.print(",");
    LoRa.print("\"er\": "); // fator de correção do sensor (Hz)
    LoRa.print(String(er));
    LoRa.print(",");
    LoRa.print("\"bar\": "); // pressão (bar)
    LoRa.print(String(bar));
    LoRa.print(",");
    LoRa.print("\"ss\": "); // status solenoide
    LoRa.print(String(Status_Solenoide));
    LoRa.print(",");
    LoRa.print("\"ta\": "); // Temperatura ambiente (ºC)
    LoRa.print(String(ta));
    LoRa.print(",");
    LoRa.print("\"um\": "); // Umidade ambiente (%)
    LoRa.print(String(um));
    LoRa.print(",");
    LoRa.print("\"pa\": "); // pressão Atmosferica (hPa)
    LoRa.print(String(pa));
    LoRa.print("}");
    LoRa.endPacket();           // Finaliza Envio TX LoRa
    Tempo_Decorrido = millis(); // atualiza timer
  }
  DateTimeParts data_hora = DateTime.getParts();  // estrutura de dados de data_hora
  if (data_hora.getHours() == Hora_Solenoide_OFF) // Verifica se após X Horas da noite há vazão
  {
    digitalWrite(Pin_Rele, LOW); // aciona o solenoide
    Status_Solenoide = true;
  }
  if (data_hora.getHours() == Hora_Solenoide_ON) // Libera as x Horas da manhã
  {
    digitalWrite(Pin_Rele, HIGH); // Libera solenoide
    Status_Solenoide = false;
  }
  delay(2000);
}

void Setup_Parametros_LoRa()
{
  /*
    LoRa.setTxPower(txPower,RFOUT_pin);
    txPower -- 0 ~ 20
    RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
      - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
      - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
  */
  LoRa.setTxPower(14, RF_PACONFIG_PASELECT_PABOOST);
  LoRa.setSyncWord(0x34);          //
  LoRa.setSpreadingFactor(7);      // (6 ~ 12)
  LoRa.setSignalBandwidth(125000); //(12500 , 250000, 500000)
  LoRa.setCodingRate4(5);          // 5 (4/5)
}

void Setup_Date_Time()
{

  DateTime.setServer("a.ntp.br"); // Servidor NTP Brasileiro
  DateTime.setTimeZone("CST+3");
  DateTime.begin();
  if (!DateTime.isTimeValid())
  {
    Serial.println("Falha ao conectar servidor NTP..: ");
  }
  else
  {
    Serial.print("Data Atual:" + String(DateTime.toString()));
    Serial.printf("Timestamp: %ld\n", DateTime.now());
  }
  DateTimeParts data_hora = DateTime.getParts(); // estrutura de dados de data_hora
  Serial.printf("%02d:%02d:%02d", data_hora.getHours(), data_hora.getMinutes(), data_hora.getSeconds());
}

void Setup_WiFi()
{
  String IP;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando...");

  while (WiFi.status() != WL_CONNECTED)
  {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(10, 30, "Aguardando conexão...");
    display.display();
    Serial.print(WiFi.status());
    delay(500);
  }
  IP = String(WiFi.networkID());

  Serial.println(IP);
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(10, 30, "Conectado em: " + String(WiFi.SSID()));
  display.display();
  delay(1500);
}

void logos() // implementar logo IFPB-CG
{
  display.clear();
  display.drawXbm(0, 0, ifpb_frame_width, ifpb_frame_height, ifpb_frame_bits);
  display.display();
  delay(1500);
  display.clear();
  display.drawXbm(0, 0, gcompii_frame_width, gcompii_frame_height, gcompii_frame_bits);
  display.display();
  delay(1500);
}
