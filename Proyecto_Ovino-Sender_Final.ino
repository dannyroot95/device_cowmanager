/********************************************************
 * This demo is only supported after RUI firmware version 3.0.0.13.X on RAK811
 * Master Board Uart Receive buffer size at least 128 bytes. 
 ********************************************************/

#include "RAK811.h"
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Timezone.h>
#include <TimeLib.h>

#define PERIODO 1500
#define LORA_RX 3
#define LORA_TX 2
#define GPS_RX 4
#define GPS_TX 5
#define WORK_MODE LoRaP2P   //  LoRaWAN or LoRaP2P
#define DebugSerial Serial

SoftwareSerial ATSerial(LORA_RX, LORA_TX);
SoftwareSerial GPSSerial(GPS_RX, GPS_TX);

RAK811 RAKLoRa(ATSerial,DebugSerial);
TinyGPS gps;

// Peru Time Zone
TimeChangeRule peMST = {"PET", Last, Sun, Mar, 1, -300};   // UTC - 5 hours
Timezone pe(peMST);


char buffer[32];
bool newData = false;
float flat, flon;
unsigned long age;
unsigned long chars;
unsigned short sentences, failed;
String data = "";
String mensaje = "";
tmElements_t timeUTC;

void setup() {
  GPSSerial.begin(9600);
  
  DebugSerial.begin(115200);
  while(DebugSerial.available())
  {
    DebugSerial.read(); 
  }
  DebugSerial.println("Inicializando sistema...");
  
   ATSerial.begin(9600); //set ATSerial baudrate:This baud rate has to be consistent with  the baud rate of the WisNode device.
  while(ATSerial.available())
  {
    ATSerial.read(); 
  }

  if(!RAKLoRa.rk_setWorkingMode(WORK_MODE))  //set WisNode work_mode to LoRaP2P.
  {
    DebugSerial.println(F("set work_mode failed, please reset module."));
    while(1);
  }
  
  RAKLoRa.rk_getVersion();  //get RAK811 firmware version
  DebugSerial.println(RAKLoRa.rk_recvData());  //print version number

  DebugSerial.println(F("Start init LoRaP2P parameters..."));  
  
  if (!RAKLoRa.rk_initP2P("869525000",12,0,1,8,20))  //init LoRaP2P
  {
    DebugSerial.println(F("Init error,please reset module.")); 
    while(1);
  }else DebugSerial.println(F("Init OK"));

  DebugSerial.println("Sistema inicializado !");
}

void loop() {
   GPSSerial.listen();
   // Intentar recibir secuencia durante un segundo
   for (unsigned long start = millis(); millis() - start < 1000;)
   {
      while (GPSSerial.available())
      {
         char c = GPSSerial.read();
         if (gps.encode(c)){ // Nueva secuencia recibida
            newData = true;
         }
      }
   }

   if (newData)
   {
      gps.f_get_position(&flat, &flon, &age);
      Serial.print("LAT=");
      Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
      Serial.print(" LON=");
      Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
      Serial.print(" SAT=");
      Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
      Serial.print(" PREC=");
      Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
      Serial.println();

      ImprimirFecha(gps);
      
      /*
      uint8_t* c = (uint8_t*)&flat;
      for(int i=0; i<4; i++){ 
        DebugSerial.println(*c, HEX);
        c++;
      }
      */
      
      mensaje = "";
      String data = DataHexadecimal((uint8_t*)&flat, sizeof(float));
      mensaje += data;
      data = DataHexadecimal((uint8_t*)&flon, sizeof(float));
      mensaje += data;
      data = DataHexadecimal(&timeUTC.Year, sizeof(uint8_t));
      mensaje += data;
      data = DataHexadecimal(&timeUTC.Month, sizeof(uint8_t));
      mensaje += data;
      data = DataHexadecimal(&timeUTC.Day, sizeof(uint8_t));
      mensaje += data;
      data = DataHexadecimal(&timeUTC.Hour, sizeof(uint8_t));
      mensaje += data;
      data = DataHexadecimal(&timeUTC.Minute, sizeof(uint8_t));
      mensaje += data;
      data = DataHexadecimal(&timeUTC.Second, sizeof(uint8_t));
      mensaje += data;

      mensaje.getBytes(buffer, 29); //Numero de caracteres + 1
      DebugSerial.print(F("Start send data: "));
      Serial.println(buffer);
       
      ATSerial.listen();
      if (RAKLoRa.rk_sendP2PData(buffer))
      {   
          DebugSerial.print("Mensaje enviado por LoRa: ");
          String ret = RAKLoRa.rk_recvP2PData();
          if(ret != NULL)
          {     
            DebugSerial.println(ret);
          }else{
            DebugSerial.println("NA");
          }
      }
      delay(PERIODO);
  
      newData = false;
   }

   gps.stats(&chars, &sentences, &failed);
   Serial.print(" CHARS=");
   Serial.print(chars);
   Serial.print(" SENTENCES=");
   Serial.print(sentences);
   Serial.print(" CSUM ERR=");
   Serial.println(failed);
}


String DataHexadecimal(uint8_t *n, uint8_t sizeN){
  String str_n = "";
  String hexByte = "";

  n += (sizeN - 1);
  for (int i = 0; i < sizeN; i++)
    {
        hexByte = String(*n, HEX);

        if (hexByte.length() == 1)
        {
            hexByte = "0" + hexByte;
        }
        str_n += hexByte;
        n --;
    }

  return str_n;
}


static void ImprimirFecha(TinyGPS &gps)
{
  int year;
  uint8_t month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  
  timeUTC.Year = year-1970;
  timeUTC.Month = month;
  timeUTC.Day = day;
  timeUTC.Hour = hour;
  timeUTC.Minute = minute;
  timeUTC.Second = second;
  time_t utc = makeTime(timeUTC);
  time_t local = pe.toLocal(utc);

  printDateTime(utc, " ");
  Serial.println();
  printDateTime(local, " ");
  Serial.println();


  /*
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("*******    *******    ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d   ",
        day, month, year, hour, minute, second);
    Serial.print(sz);
  }
  Serial.println();
  */
  //print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
}


// format and print a time_t value, with a time zone appended.
void printDateTime(time_t t, const char *tz)
{
    char buf[32];
    char m[4];    // temporary storage for month string (DateStrings.cpp uses shared buffer)
    strcpy(m, monthShortStr(month(t)));
    sprintf(buf, "%.2d:%.2d:%.2d %s %.2d %s %d %s",
        hour(t), minute(t), second(t), dayShortStr(weekday(t)), day(t), m, year(t), tz);
    Serial.print(buf);
}