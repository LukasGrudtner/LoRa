#include "compiler.h"

#ifndef SENDER

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <math.h>

/*  Definição dos pinos LoRa. */
#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    19   // GPIO19 -- SX127x's MISO
#define MOSI    27   // GPIO27 -- SX127x's MOSI
#define SS      18   // GPIO18 -- SX127x's CS
#define RST     14   // GPIO14 -- SX127x's RESET
#define DI00    26   // GPIO26 -- SX127x's IRQ(Interrupt Request)

#define BAND    915E6  //Frequencia do radio - podemos utilizar ainda : 433E6, 868E6, 915E6
#define PABOOST true

String rssi = "RSSI --";
String packSize = "--";
String packet;

TinyGPSPlus gps;

Position position;



Position getPosition()
{
    Position position;
    position.lat = gps.location.lat();
    position.lng = gps.location.lng();
    position.alt = gps.altitude.feet() / 3.2808;
    position.satellites = gps.satellites.value();

    return position;
}

double toRadians(double degree)
{
    return (degree/180)*3.14;
}

// in meters
double getDistance()
{
    Position myPosition = getPosition();
    double dlon = toRadians(position.lng - myPosition.lng);
    double dlat = toRadians(position.lat - myPosition.lat);

    double R = 6373;
    double a = pow(sin(dlat/2), 2) + cos(toRadians(myPosition.lat)) * cos(toRadians(position.lat)) * pow(sin(dlon/2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = R * c;

    return d*1000;
}

void loraData()
{
    Position myPosition = getPosition();
    Serial.println(rssi);
    Serial.print("Longitude: ");
    Serial.println(position.lng, 5);

    Serial.print("Latitude: ");
    Serial.println(position.lat, 5);

    // Serial.print("Altitude: ");
    // Serial.println(position.alt);

    Serial.println("----------");

    /* Minha posição */
    Serial.print("Longitude: ");
    Serial.println(myPosition.lng, 5);

    Serial.print("Latitude: ");
    Serial.println(myPosition.lat, 5);

    Serial.print("Altitude: ");
    Serial.println(myPosition.alt);

    double distance = getDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print("m (");
    Serial.print(distance/1000);
    Serial.println("km)\n");
}

//função responsável por recuperar o conteúdo do pacote recebido
//parametro: tamanho do pacote (bytes)
void cbk(int packetSize) {
  packet ="";
  packSize = String(packetSize,DEC); //transforma o tamanho do pacote em String para imprimirmos
//   for (int i = 0; i < packetSize; i++) { 
//     packet += (char) LoRa.read(); //recupera o dado recebido e concatena na variável "packet"
//   }
    if (packetSize)
    {
        LoRa.readBytes((uint8_t *)&position, packetSize);
    }
  rssi = "RSSI=  " + String(LoRa.packetRssi(), DEC)+ "dB"; //configura a String de Intensidade de Sinal (RSSI)
  //mostrar dados em tela
  loraData();
}



void setup()
{
    Serial.begin(9600);
    Serial1.begin(9600, SERIAL_8N1, 12, 15);

    /*  Inicia a comunicação serial com o LoRa. */
    SPI.begin(SCK, MISO, MOSI, SS);
    
    /*  Configura os pinos que serão utilizados pela biblioteca
        (deve ser chamado antes do LoRa.begin()).
    */
    LoRa.setPins(SS, RST, DI00);

    /* Inicializa o LoRa com a frequencia especificada. */
    if (!LoRa.begin(BAND))
        while (1);

    LoRa.receive();
    LoRa.onReceive(cbk);
    delay(1000);
}

static void smart_delay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } 
  while (millis() - start < ms); 
}

void loop()
{
    int packetSize = LoRa.parsePacket();

    // Serial.printf("PacketSize: %d", packetSize);
    if (packetSize)
    {
        cbk(packetSize);
    }
    
    smart_delay(10);
}

#endif