#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#define NRFserial Serial
 
const char* ssid = "ARRIS-C082";
const char* password = "05ADE64EB01C048D";
const char* mqttServer = "postman.cloudmqtt.com";
const int mqttPort = 16444;
//const char* mqttUser = "richar";
//const char* mqttPassword = "cloudmqtt";
const char* mqttUser = "richar";
const char* mqttPassword = "cloudmqtt";

char *resultado = NULL;
char separador[] = "_";  
float valores[5];
char Texto[50];
 
WiFiClient espClient;
PubSubClient client(espClient);
 
void setup() {
 
  Serial.begin(9600);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.println("Connecting to WiFi..");
  }
  //Serial.println("Connected to the WiFi network");
 
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
 
  while (!client.connected()) {
    //Serial.println("Connecting to MQTT...");
 
    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
 
      //Serial.println("connected");  
 
    } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
 
    }
  }
 
  //client.publish("ph", "Hello from ESP8266");
  client.subscribe("ph");
 
}
 
void callback(char* topic, byte* payload, unsigned int length) {
 
  //Serial.print("Message arrived in topic: ");
  //Serial.println(topic);
 
  //Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    //Serial.print((char)payload[i]);
  }
 
  //Serial.println();
  //Serial.println("-----------------------");
 
}
 
void loop() {
  client.loop();
  if (NRFserial.available())
   {
      String myString = NRFserial.readStringUntil('\n');
      myString.toCharArray(Texto, 50);
      //Serial.println(myString);
      //Serial.println(Texto[0]);

      int index=0;
      resultado = strtok(Texto,"_");
      while((resultado != NULL) && (index < 5)){
        valores[index++] =  atof(resultado);        
        resultado = strtok(NULL,"_");     
        }
         float t = valores[0];
   float o = valores[1];
   float p = valores[2];
   float lo = valores[3];
   float la = valores[4];

   char temperatura[10]="";
   char oxigeno[10]="";
   char ph[10]="";
   char longitud[10]="";
   char latitud[10]="";

    dtostrf(t,7, 3, temperatura);    
    dtostrf(o,7, 3, oxigeno); 
    dtostrf(p,7, 3, ph); 
    dtostrf(lo,7, 3, longitud); 
    dtostrf(la,7, 3, latitud);                
        
if (t != 0.000 ){
  client.publish("time","time");
  client.publish("temperatura",temperatura);
  client.publish("ph",ph);
  client.publish("oxigeno_dis",oxigeno);
  client.publish("longitud",longitud);
  client.publish("latitud",latitud);
  client.publish("send","send");  
  }   
   } 
  delay(1500);
}
