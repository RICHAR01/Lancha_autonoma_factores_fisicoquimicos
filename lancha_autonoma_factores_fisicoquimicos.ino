#include <SPI.h>        //Libreria NRF24
#include "RF24.h"        //Libreria NRF24
#include <TimerOne.h>    //Libreria para Timer1
#include <TinyGPS.h>     //incluimos TinyGPS
#include <Wire.h>        //Habilitar  lectura y escritura analogica e I2C
#include "compass.h"     //Libreria burjula(compass) con calibracion
#include <math.h>        // (no semicolon)
#include <OneWire.h>     // Sensor temp              
#include <DallasTemperature.h> //Sensor Temperatura
#include <avr/pgmspace.h> //Funcion eprom
#include <EEPROM.h>       //Read Write EEPROM
#include <Servo.h>       //Control angulo Servomotor(Timon) 
 

/* Ruta en coordenadas UTM a seguir por la lancha*/
double RUTA_X[]={257809.51,257750.91,257703.30};
double RUTA_Y[]={2746336.54,2746338.38,2746338.56};
double x_actual;
double y_actual;
boolean cambiar_coordenada = false;
int ciclo_actual=0;

/* Parametros de sintonizacion PID */
double kp = 1;
double ki = 0;
double kd = 0;
double velocidad = 200;
double Centro = 90;
double calculo_PID;
double error_actual;

#define PhSensorPin  A1    //dissolved oxygen sensor analog output pin to arduino mainboard
#define DoSensorPin  A2    //dissolved oxygen sensor analog output pin to arduino mainboard
#define VREF 5000    //for arduino uno, the ADC reference is the AVCC, that is 5000mV(TYP)
float doValue;      //current dissolved oxygen value, unit; mg/L
float temperature = 0;    //default temperature is 25^C, you can use a temperature sensor to read it

#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength+1];    // store the serial command
byte receivedBufferIndex = 0;

#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the analog value in the array, readed from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;

#define SaturationDoVoltageAddress 12          //La dirección del voltaje de saturación de oxígeno almacenado en la EEPROM
#define SaturationDoTemperatureAddress 16      //La dirección de la temperatura de saturación almacenado en la EEPROM
float SaturationDoVoltage,SaturationDoTemperature;
float averageVoltage;

//Tabla de saturación de concentración máxima de oxígeno disuelto
const float SaturationValueTab[41] PROGMEM = {      
14.46, 14.22, 13.82, 13.44, 13.09,
12.74, 12.42, 12.11, 11.81, 11.53,
11.26, 11.01, 10.77, 10.53, 10.30,
10.08, 9.86,  9.66,  9.46,  9.27,
9.08,  8.90,  8.73,  8.57,  8.41,
8.25,  8.11,  7.96,  7.82,  7.69,
7.56,  7.43,  7.30,  7.18,  7.07,
6.95,  6.84,  6.73,  6.63,  6.53,
6.41,
};

//Se inicializa el transmisor NRF24 pro los pines 9 y 10
RF24 myRadio (9, 10);
byte addresses[][6] = {"0"};
/* Se declarara la clase package */
struct package {
  int id = 1;
  float temperature;
  float longitud;
  float latitud;
  float oxigeno;
  float ph;
};

/* Se declarara la clase estacion */
struct estacion{
  boolean modo_rc;
  float m_izq;
  float m_der;
};

typedef struct estacion Estacion;
typedef struct package Package;
/* la data Recibida se declara como objeto de la clase Estacion */
Estacion dataRecieve;
/* la data Transmitida se declara como objeto de la clase Package */
Package dataTransmit;


OneWire ourWire(2);                //Se establece el pin 2  como bus OneWire 
DallasTemperature sensors(&ourWire); //Se declara una variable u objeto para nuestro sensor

TinyGPS gps;  //Declaramos el objeto GPS
volatile double error_anterior;
volatile boolean rc_mode;
volatile float left_m;
volatile float right_m;

float declinacion = 7.73;

char mensaje1[15];//INICIAMOS UNA CADENA
float lon_lat;
int k;

int MOTOR_PROPELLA = 5;
int SERVO_TIMON = 6;

 float ANGULO_TIMON;

//longitud x latitud y de ejemplo
float longitud_deseada = -107.406012;
float latitud_deseada =  24.784226;

struct utm {
   double x;
   double y;
};

struct utm coordenadas_objetivo;
struct utm coordenadas_actual;

//longitud y latitud leidas por GPS
float latitude, longitude;

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards


void setup() {
    pinMode(DoSensorPin,INPUT);
    pinMode(PhSensorPin,INPUT);

    readDoCharacteristicValues();      //read Characteristic Values calibrated from the EEPROM
     
    Serial.begin(9600);            //Iniciamos el puerto serie
    myRadio.begin();  
    myRadio.setChannel(115); 
    myRadio.setPALevel(RF24_PA_MAX);
    myRadio.setDataRate( RF24_250KBPS );
  
    myRadio.openReadingPipe(1, addresses[0]);
    myRadio.startListening();
  
    pinMode(MOTOR_PROPELLA, OUTPUT); //MOTOR IZQ(PIN 5) COMO SALIDA
    pinMode(SERVO_TIMON, OUTPUT);   //MOTOR DER(PIN 6) COMO SALIDA 
    pinMode(4, OUTPUT);   //MOTOR DER(PIN 6) COMO SALIDA 
    pinMode(7, OUTPUT);   //MOTOR DER(PIN 6) COMO SALIDA  
    pinMode(8, OUTPUT);   //MOTOR DER(PIN 6) COMO SALIDA 

    digitalWrite(4, HIGH);
    digitalWrite(7, HIGH);

    /* Iniciar lectura de Compass i2c e sensores analogicos */
    Wire.begin();   
    /* Parametros de calibracion de compass */
    compass_x_offset = -7.48;        
    compass_y_offset = 38.32;
    compass_z_offset = 145.98;
    compass_x_gainError = 0.94;
    compass_y_gainError = 1;
    compass_z_gainError = 0.87;
 
    compass_init(2);
    compass_debug = 1;
    //compass_offset_calibration(3);       // (1) Calibrar offset (2) Calibrar Gain Error (3)Calibrar ambos
    sensors.begin();   //Se inicia el sensor de temperatura

    myservo.attach(SERVO_TIMON);  // attaches the servo on pin 6 to the servo object
}

void loop() {

 for (int i = 0; i<=3; i++)
 {
  ciclo_actual= i;
  do{
     if (dataRecieve.modo_rc == false)    
     { 
      for (int j = 0; j<=1; j++)
     { 
      Wire.requestFrom(1,14);//REALIZAMOS UNA PETICION AL CANAL 1 DE 19 CARACTERES (BITS)
      k=0;
      while(Wire.available())
      { 
       mensaje1[k] = Wire.read();//GUARDAMOS EL MENSAJE EN LA CADENA MESAJE 1     
       k++;
       delay(100); 
       }
       lon_lat =  atof(mensaje1); 
       if(lon_lat < 0)
        {latitude = lon_lat;}
        else
        {longitude = lon_lat;}
      }
        Serial.print("latitud: ");
        Serial.println(latitude,8);
        Serial.print("longitud: ");
        Serial.println(longitude,8);
     
     coordenadas_actual = conversionUTM(latitude, longitude);
     x_actual = coordenadas_actual.x;
     y_actual =coordenadas_actual.y;   
          

       compass_heading();   
       bearing = bearing - declinacion;
       if(bearing < 0)
       {bearing = 365 + bearing;}

       error_actual = error( x_actual, y_actual, RUTA_X[ciclo_actual], RUTA_Y[ciclo_actual], bearing);
       calculo_PID = kp*error_actual + kd*(error_actual - error_anterior) + ki*(error_actual + error_anterior);

      ANGULO_TIMON = Centro + (calculo_PID);
     
      if (ANGULO_TIMON<0)     ANGULO_TIMON= 0;
      if (ANGULO_TIMON>180)   ANGULO_TIMON= 180;  
     
       myservo.write(ANGULO_TIMON);
       print_gps_mode();
       }
      else
      {
       if(left_m >= 100){
        digitalWrite(MOTOR_PROPELLA, HIGH);
       }
       else{
        digitalWrite(MOTOR_PROPELLA, LOW);
        }
       myservo.write(right_m);
       print_rc_mode();
       }
      
       error_anterior = error_actual;
       
     //float ph = get_ph();
     //float oxigeno_dis = get_dissolved_oxygen();
     //sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
     //float temp= sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC
     noInterrupts();               // Suspende las interrupciones
     if ( myRadio.available()) {
      while (myRadio.available()){
      myRadio.read( &dataRecieve, sizeof(dataRecieve) );
      }
     //RECIBIR SEÑALES
     Serial.println(dataRecieve.modo_rc);
     Serial.println(dataRecieve.m_izq);
     Serial.println(dataRecieve.m_der);
     rc_mode = dataRecieve.modo_rc;
     left_m = dataRecieve.m_izq;
     right_m = dataRecieve.m_der;
     }
     delay(200);
     myRadio.stopListening();
     dataTransmit.id = dataTransmit.id + 1;
     //dataTransmit.temperature = temp;
     //dataTransmit.ph = ph;
     //dataTransmit.oxigeno = doValue;
     dataTransmit.temperature = 1;
     dataTransmit.ph = 2;
     dataTransmit.oxigeno = 3;
     dataTransmit.longitud = longitude;
     dataTransmit.latitud = latitude;  
     ///
     myRadio.openWritingPipe(addresses[0]);
     myRadio.write(&dataTransmit, sizeof(dataTransmit));
     myRadio.openReadingPipe(1, addresses[0]);
     myRadio.startListening();
 
     interrupts();                 // Autoriza las interrupciones
     cambiar_coordenada = change_point( coordenadas_actual.x, coordenadas_actual.y, RUTA_X[ciclo_actual], RUTA_Y[ciclo_actual], 3);
     }while(cambiar_coordenada == false);
 }   
}


void print_gps_mode() {
  Serial.println("GPS MODE ========================:");

  Serial.print("rc_mode: ");
  Serial.println(rc_mode);

  Serial.print("Angulo :");
  Serial.println(bearing);

  Serial.print("ciclo_actual :");
  Serial.println(ciclo_actual);
 
  Serial.print("error_actual :");
  Serial.println(error_actual);

  Serial.print("x_actual :");
  Serial.println(x_actual);
  Serial.print("y_actual :");
  Serial.println(y_actual);

  Serial.print("RUTA X :");
  Serial.println(RUTA_X[ciclo_actual]);
  Serial.print("RUTA Y :");
  Serial.println(RUTA_Y[ciclo_actual]);

  Serial.print("ANGULO_TIMON :");
  Serial.println(ANGULO_TIMON);
  
  Serial.print("Change Point :");
  Serial.println(cambiar_coordenada);
}
void print_rc_mode() {
  Serial.println("RC MODE *************************************:");
     
  Serial.print("error_actual :");
  Serial.println(error_actual);

  Serial.print("x_actual :");
  Serial.println(x_actual);
  Serial.print("y_actual :");
  Serial.println(y_actual);

  Serial.print("RUTA X :");
  Serial.println(RUTA_X[ciclo_actual]);
  Serial.print("RUTA Y :");
  Serial.println(RUTA_Y[ciclo_actual]);

  Serial.print("ANGULO_TIMON :");
  Serial.println(right_m);
}
//  error(Logitud_actual, Latitud_actual, Longitud_deseada,angulo_actual)
double error( double longitud_1, double latitud_1, double longitud_2, double latitud_2, double angulo_actual) {   
  double pendiente;
  double angulo_radianes;
  double angulo_grados;
  double angulo_deseado;
  double error;

  pendiente = (latitud_2 - latitud_1) / (longitud_2 - longitud_1);
  pendiente = fabs (pendiente);
  angulo_radianes = atan (pendiente);
  angulo_grados = (180*angulo_radianes)/PI;
  
  if (longitud_2 > longitud_1 && latitud_2 > latitud_1) { 
    angulo_deseado = 90 - angulo_grados;
  }
  if (longitud_2 > longitud_1 && latitud_2 < latitud_1) { 
    angulo_deseado = 90 + angulo_grados;
  }
  if (longitud_2 < longitud_1 && latitud_2 < latitud_1) { 
    angulo_deseado = 270 - angulo_grados;
  }
  if (longitud_2 < longitud_1 and latitud_2 > latitud_1) { 
    angulo_deseado = 270 + angulo_grados;
  }

  error = angulo_deseado - angulo_actual;

  if (error>180) {
    error= -(360 - angulo_deseado + angulo_actual);
  }
  if (error<-180) {
    error= 360 - angulo_actual + angulo_deseado;
  }    
  return error;
}

struct utm conversionUTM(double lati, double longi)
{
  struct utm coordenadas;
  /*!
   * Transformación de las coordenadas geográficas a UTM
   */
  /// Sobre la geometría del delipsoide WGS84
  double a = 6378137.0;
  double b = 6356752.3142;
  
  //  float e = sqrt((a*a) + (b*b))/a; ///< Excentricidad.
  double e = sqrt((a*a) - (b*b))/b; ///< Segunda excentricidad.
  double e2 = e * e; ///< al cuadrado. Usaremos esta directamente.
  
  double c = a*a / b; ///< Radio Polar de Curvatura.
  
  /// Sobre la longitud y latitud. Conversión de grados decimales a radianes.
  /*!
   * Cálculo del signo de la longitud:
   *      - Si la longitud está referida al Oeste del meridiano de Greenwich, 
   *        entonces la longitud es negativa (-).
   *      - Si la longitud está referida al Este del meridiano de Greenwich,
   *        entonces la longitud es positiva 8+).
   */  
  double latRad = lati * PI / 180.0; ///< Latitud en Radianes.
  double lonRad = longi * PI / 180.0; ///< Longitud en Radianes.

  /// Sobre el huso.
  float h = (longi / 6) + 31;  ///< Nos interesa quedarnos solo con la parte entera.
  int huso = int(h);
  int landa0 = (huso * 6) - 183; ///< Cálculo del meridiano central del huso en radianes.
  double Dlanda = lonRad - (landa0 * PI / 180.0);  ///< Desplazamiento o diferencial del punto a calcular con respecto al meridiano central del huso.(Calculo en Radianes)
  
  /*!
   * Ecuaciones de Coticchia-Surace para el paso de Geográficas a UTM (Problema directo);
   */
    
  /// Cálculo de Parámetros.
  double coslatRad = cos (latRad);
  double coslatRad2 = coslatRad * coslatRad;

  //double sinDlanda = sin (Dlanda);
    
  double A = coslatRad * sin (Dlanda);
  double xi = 0.5 * log ((1 + A) / (1 - A));
  double n = atan(tan(latRad) / cos(Dlanda)) - latRad;
  double v = (c / sqrt(1 + e2 * coslatRad2)) * 0.9996;
  double z = (e2/ 2.0) * xi * xi * coslatRad2;
  double A1 = sin (2 * latRad);
  double A2 = A1 * coslatRad2;
  double J2 = latRad + (A1 / 2.0);
  double J4 = (3.0 * J2 + A2) / 4.0;
  double J6 = (5.0 * J4 + A2 * coslatRad2) / 3.0;
  double alf = 0.75 * e2;
  double bet = (5.0 / 3.0) * alf * alf;
  double gam = (35.0 / 27.0) * alf * alf * alf;
  double Bfi = 0.9996 * c * (latRad - alf * J2 + bet * J4 - gam * J6);
   
  /*! 
   * Cálculo final de coordenadas UTM
   */ 
  coordenadas.x = xi * v * (1 + (z / 3.0)) + 500000; /*!< 500.000 es el retranqueo que se realiza en cada huso sobre el origen de
  coordenadas en el eje X con el objeto de que no existan coordenadas negativas. */
  coordenadas.y = n * v * (1 + z) + Bfi;  /*!< En el caso de latitudes al sur del ecuador, se sumará al valor de Y 10.000.000
  para evitar coordenadas negativas. */
  return coordenadas;
}


boolean change_point( float x1, float y1, float x2, float y2, float error_max)
{  
  float x1_abs = fabs (x1);
  float y1_abs = fabs (y1);
  float x2_abs = fabs (x2);
  float y2_abs = fabs (y2);
  float x_dif = x2_abs - x1_abs;
  float y_dif = y2_abs - y1_abs;
  float x_dif_abs = fabs (x_dif);
  float y_dif_abs = fabs (y_dif);

  double error = sqrt((x_dif_abs * x_dif_abs) + (y_dif_abs * y_dif_abs));

  if (error <= error_max) 
  { 
    return true;
   }
   else
   {
   return false;
    }
  }

  float get_ph()
  {
    unsigned long int avgValue;  //Store the average value of the sensor feedback
    float b;
    int buf[10],temp; 

    for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(PhSensorPin);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=3.5*phValue;
  return phValue;                     
  }

  boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while ( Serial.available() > 0 )
  {
    if (millis() - receivedTimeOut > 500U)
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer,0,(ReceivedBufferLength+1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex == ReceivedBufferLength)
    {
    receivedBufferIndex = 0;
    strupr(receivedBuffer);
    return true;
    }else{
        receivedBuffer[receivedBufferIndex] = receivedChar;
        receivedBufferIndex++;
    }
  }
  return false;
}

byte uartParse()
{
    byte modeIndex = 0;
    if(strstr(receivedBuffer, "CALIBRATION") != NULL)
        modeIndex = 1;
    else if(strstr(receivedBuffer, "EXIT") != NULL)
        modeIndex = 3;
    else if(strstr(receivedBuffer, "SATCAL") != NULL)
        modeIndex = 2;
    return modeIndex;
}

void doCalibration(byte mode)
{
    char *receivedBufferPtr;
    static boolean doCalibrationFinishFlag = 0,enterCalibrationFlag = 0;
    float voltageValueStore;
    switch(mode)
    {
      case 0:
      if(enterCalibrationFlag)
         Serial.println(F("Command Error"));
      break;

      case 1:
      enterCalibrationFlag = 1;
      doCalibrationFinishFlag = 0;
      Serial.println();
      Serial.println(F(">>>Enter Calibration Mode<<<"));
      Serial.println(F(">>>Please put the probe into the saturation oxygen water! <<<"));
      Serial.println();
      break;

     case 2:
      if(enterCalibrationFlag)
      {
         float  od = get_dissolved_oxygen();
         Serial.println();
         Serial.println(F(">>>Saturation Calibration Finish!<<<"));
         Serial.println();
         EEPROM_write(SaturationDoVoltageAddress, averageVoltage);
         EEPROM_write(SaturationDoTemperatureAddress, temperature);
         SaturationDoVoltage = averageVoltage;
         SaturationDoTemperature = temperature;
         doCalibrationFinishFlag = 1;
      }
      break;

        case 3:
        if(enterCalibrationFlag)
        {
            Serial.println();
            if(doCalibrationFinishFlag)
               Serial.print(F(">>>Calibration Successful"));
            else
              Serial.print(F(">>>Calibration Failed"));
            Serial.println(F(",Exit Calibration Mode<<<"));
            Serial.println();
            doCalibrationFinishFlag = 0;
            enterCalibrationFlag = 0;
        }
        break;
    }
}

int getMedianNum(int bArray[], int iFilterLen)
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
      bTab[i] = bArray[i];
      }
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

void readDoCharacteristicValues(void)
{
    EEPROM_read(SaturationDoVoltageAddress, SaturationDoVoltage);
    EEPROM_read(SaturationDoTemperatureAddress, SaturationDoTemperature);
    if(EEPROM.read(SaturationDoVoltageAddress)==0xFF && EEPROM.read(SaturationDoVoltageAddress+1)==0xFF && EEPROM.read(SaturationDoVoltageAddress+2)==0xFF && EEPROM.read(SaturationDoVoltageAddress+3)==0xFF)
    {
      SaturationDoVoltage = 1127.6;   //default voltage:1127.6mv
      EEPROM_write(SaturationDoVoltageAddress, SaturationDoVoltage);
    }
    if(EEPROM.read(SaturationDoTemperatureAddress)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+1)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+2)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+3)==0xFF)
    {
      SaturationDoTemperature = 25.0;   //default temperature is 25^C
      EEPROM_write(SaturationDoTemperatureAddress, SaturationDoTemperature);
    }
}

float get_dissolved_oxygen()
{
  boolean flag = false;
  while(flag == false)
  {
         static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 30U)     //every 30 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(DoSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT)
         analogBufferIndex = 0;
   }

   static unsigned long tempSampleTimepoint = millis();
   if(millis()-tempSampleTimepoint > 500U)  // every 500 milliseconds, read the temperature
   {  
      tempSampleTimepoint = millis();
      sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
      temperature= sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC
   }

   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 1000U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      {
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      }
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF  / 1024.0; // read the value more stable by the median filtering algorithm
      //Serial.print(F("Temperature:"));
      //Serial.print(temperature,1);
      //Serial.print(F("^C"));
      doValue = pgm_read_float_near( &SaturationValueTab[0] + (int)(SaturationDoTemperature+0.5) ) * averageVoltage / SaturationDoVoltage;  //calculate the do value, doValue = Voltage / SaturationDoVoltage * SaturationDoValue(with temperature compensation)
      //Serial.print(F(",  DO Value:"));
      //Serial.print(doValue,2);
      //Serial.println(F("mg/L")); 

      flag = true;
   }

   if(serialDataAvailable() > 0)
   {
      byte modeIndex = uartParse();  //parse the uart command received
      doCalibration(modeIndex);    // If the correct calibration command is received, the calibration function should be called.
   }
  }
  return doValue;
}
