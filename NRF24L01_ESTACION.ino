#include <SPI.h>  
#include "RF24.h"
#include <math.h>        // (no semicolon)

int Pin_Derecho = A2;
int Pin_Izquierdo = A0; 
int Rc_mode = 8;
int LED = 6;

RF24 myRadio (9, 10);
byte addresses[][6] = {"0"};

struct package {
  int id = 1;
  float temperature = 18.3;
  float longitud = 2334.543434;
  float latitud = 332.111111;
  float oxigeno = 222.1111;
  float ph = 32.000002;
};

struct estacion{
  boolean modo_rc=false;
  float m_izq= 0;
  float m_der= 0;
};

typedef struct estacion Estacion;


typedef struct package Package;
Package dataRecieve;
Estacion dataTransmit;

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  myRadio.begin();  
  myRadio.setChannel(115); 
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate( RF24_250KBPS );
  
  myRadio.openReadingPipe(1, addresses[0]);
  myRadio.startListening();
}

void loop() {
  if ( myRadio.available()) {
    while (myRadio.available()){
      myRadio.read( &dataRecieve, sizeof(dataRecieve) );
    }
    char Enviar[200];
    
    char texto_1[15];
    char texto_2[20];
    char texto_3[20];
    char texto_4[15];
    char texto_5[15];
       
  dtostrf(dataRecieve.temperature,4, 2, texto_1);  
  dtostrf(dataRecieve.longitud,7, 9, texto_2);  
  dtostrf(dataRecieve.latitud,7, 9, texto_3);  
  dtostrf(dataRecieve.oxigeno,7, 5, texto_4);  
  dtostrf(dataRecieve.ph,7, 5, texto_5);  
  Serial.println(dataRecieve.longitud,9); 
  Serial.println(dataRecieve.latitud,9); 
  sprintf(Enviar, "%s_%s_%s_%s_%s_", texto_1, texto_2, texto_3, texto_4, texto_5);
   Serial.println(Enviar); 
  }  
  delay(200);
    //Serial.println("taco"); 
  myRadio.stopListening();
  
  dataTransmit.m_izq = (((analogRead(Pin_Izquierdo))/4)-127)*2;
  if (dataTransmit.m_izq<0)   dataTransmit.m_izq= 0;
  dataTransmit.m_der = (analogRead(Pin_Derecho)) / (5.64);

  //dataTransmit.m_izq = analogRead(Pin_Izquierdo);
  //dataTransmit.m_der = analogRead(Pin_Derecho);
  
  if(digitalRead(Rc_mode) == HIGH ){
  dataTransmit.modo_rc = true;
  digitalWrite(6, HIGH);     
  }
  else{
    dataTransmit.modo_rc = false;
    digitalWrite(6, LOW);
    }
  myRadio.openWritingPipe(addresses[0]);
  myRadio.write(&dataTransmit, sizeof(dataTransmit));
  myRadio.openReadingPipe(1, addresses[0]);
  myRadio.startListening();
  delay(1800);
}
