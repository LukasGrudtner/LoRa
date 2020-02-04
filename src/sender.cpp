#include "compiler.h"

#ifndef RECEIVER

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;

/*  Definição dos pinos LoRa. */
#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    19   // GPIO19 -- SX127x's MISO
#define MOSI    27   // GPIO27 -- SX127x's MOSI
#define SS      18   // GPIO18 -- SX127x's CS
#define RST     14   // GPIO14 -- SX127x's RESET
#define DI00    26   // GPIO26 -- SX127x's IRQ(Interrupt Request)

#define BAND    915E6  //Frequencia do radio - podemos utilizar ainda : 433E6, 868E6, 915E6
#define PABOOST true

//variável responsável por armazenar o valor do contador (enviaremos esse valor para o outro Lora)
unsigned int counter = 0;

static void smart_delay(unsigned long ms);
Position getPosition();

void setup()
{
    Serial.begin(9600);
    Serial1.begin(9600, SERIAL_8N1, 12, 15);

    /*  Inicia comunicação serial com o LoRa. */
    SPI.begin(SCK, MISO, MOSI, SS);

    /*  Configura os pinos que serão utilizados pela biblioteca
        (deve ser chamado antes do LoRa.begin()).
    */
   LoRa.setPins(SS, RST, DI00);

   /*   Inicializa o LoRa com a frequência especificada. */
   if (!LoRa.begin(BAND))
   {
       while(1);
   }

   delay(1000);
}

void loop()
{
    /*  Abre um pacote para adicionarmos os dados para o envio. */
    LoRa.beginPacket();

    /*  Adiciona os dados no pacote. */
    Position position = getPosition();
    LoRa.write((uint8_t*)&position, sizeof(position));

    /*  Fecha o pacote e envia. */
    LoRa.endPacket();

    Serial.printf("Packet: %d\n", counter);
    // Serial.println(getPosition());

    counter++;

    smart_delay(1000);

    if (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS data received: check wiring"));
}

Position getPosition()
{
    Position position;
    position.lat = gps.location.lat();
    position.lng = gps.location.lng();
    position.alt = gps.altitude.feet() / 3.2808;
    position.satellites = gps.satellites.value();

    return position;
    
    //  position = "";
    // position += "Latitude    :";
    // position += gps.location.lat();
    // position += "\n";

    // position += "Longitude   : ";
    // position += gps.location.lng();
    // position += "\n";

    // position += "Satellites  : ";
    // position += gps.satellites.value();
    // position += "\n";

    // position += "Altitude    : ";
    // position += (gps.altitude.feet() / 3.2808);
    // position += "M\n";

    // position += "Time        : ";
    // position += gps.time.hour();
    // position += ":";
    // position += gps.time.minute();
    // position += ":";
    // position += gps.time.second();
    // position += "\n";

    // return position;   
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

#endif