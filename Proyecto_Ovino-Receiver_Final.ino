#include "RAK811.h"
#include "SoftwareSerial.h"
#include <Timezone.h>
#include <TimeLib.h>

#define PERIODO_ENVIAR_DATOS 5000
#define PERIODO_REINTENTAR 1000
#define URL_THINGSBOARD "http://demo.thingsboard.io/api/v1/IeFHuBT6RSyN35b1zaRu/telemetry"
#define WORK_MODE LoRaP2P
#define LORA_RX 3
#define LORA_TX 4
#define SIM_RX 7
#define SIM_TX 6
#define SIM_RST 5

#define DebugSerial Serial

SoftwareSerial ATSerial(LORA_RX,LORA_TX);
SoftwareSerial SIM800Serial(SIM_RX, SIM_TX);

RAK811 RAKLoRa(ATSerial,DebugSerial);

// Peru Time Zone

TimeChangeRule peMST = {"PET", Last, Sun, Mar, 1, -300};   // UTC - 5 hours
Timezone pe(peMST);

tmElements_t timeUTC;
time_t timePET, timeGlobal;
boolean datoRecibido = false;
float latitud=0.0f, longitud=0.0f;
uint32_t prevMillisTB, periodoTB = PERIODO_ENVIAR_DATOS;


void setup() {
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
  ATSerial.listen();
  if(RAKLoRa.rk_setWorkingMode(LoRaP2P))  //set WisNode work_mode to LoRaP2P.
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

  SIM800Serial.begin(9600);
  pinMode(SIM_RST, OUTPUT);
  digitalWrite(SIM_RST, LOW);
  delay(100);
  digitalWrite(SIM_RST, HIGH);
  
  DebugSerial.println("Sistema inicializado !");
  delay(1000);
  prevMillisTB = millis();
}

void loop() {
    ATSerial.listen();  
    String ret = RAKLoRa.rk_recvP2PData();
    if(ret != NULL)
    { 
      if (ATSerial.overflow()) {
        Serial.println("SoftwareSerial overflow!");
        while(ATSerial.available()!=0){
          uint8_t c = ATSerial.read();
        }
      }else{       
        DebugSerial.println(ret);
        int i = ret.indexOf(':'); //Obtiene el indice del caracter ':'
        i += 1; //Se descarta el primer caracter ':'
        String dataHexa = ret.substring(i);
        DebugSerial.println("Datos recibidos:" + dataHexa);
  
        String lat = dataHexa.substring(0, 8);
        String lon = dataHexa.substring(8, 16);
        latitud = getFloat(lat);
        longitud = getFloat(lon);
  
        DebugSerial.println(lat);      
        DebugSerial.println(latitud, 6);
        DebugSerial.println(lon);     
        DebugSerial.println(longitud, 6);
        
        uint8_t yearUTC = getUint8(dataHexa.substring(16, 18));
        uint8_t monthUTC = getUint8(dataHexa.substring(18, 20));
        uint8_t dayUTC = getUint8(dataHexa.substring(20, 22));
        uint8_t hourUTC = getUint8(dataHexa.substring(22, 24));
        uint8_t minuteUTC = getUint8(dataHexa.substring(24, 26));
        uint8_t secondUTC = getUint8(dataHexa.substring(26, 28));
        
        
        timeUTC.Year = yearUTC;
        timeUTC.Month = monthUTC;
        timeUTC.Day = dayUTC;
        timeUTC.Hour = hourUTC;
        timeUTC.Minute = minuteUTC;
        timeUTC.Second = secondUTC;
        timeGlobal = makeTime(timeUTC);
        timePET = pe.toLocal(timeGlobal);
        /*
        printDateTime(utc, " ");
        Serial.println();
        printDateTime(local, " ");
        Serial.println();
        */
        datoRecibido = true;
      }
    }

    if((millis() - prevMillisTB) > periodoTB){
      if(datoRecibido){
        DebugSerial.println("Enviando...");
        
        if(EnviarThingsboard()){
          DebugSerial.println("Mensaje SI enviado a Thingboard !");
          periodoTB = PERIODO_ENVIAR_DATOS;
        }else{
          DebugSerial.println("Mensaje NO enviado a Thingboard !");
          periodoTB = PERIODO_REINTENTAR;
        }
        
        prevMillisTB = millis();
        //datoRecibido = false;
      }
    }
}

float getFloat(String dataHexa){
  float sol;
  float* adr = &sol;
  byte b;
  byte data[4];
  for(int i=3; i>-1; i--){
    b = getByte(dataHexa.charAt(i*2), dataHexa.charAt(i*2+1));
    data[3-i] = b;
    //Serial.println(data[i], HEX);
  }
  //Serial.println(*data, HEX); //Inicia en data[0]
  memcpy(adr, data, 4);
  return sol;
}

uint8_t getUint8(String dataHexa){
  uint8_t sol;
  sol = (uint8_t)getByte(dataHexa.charAt(0), dataHexa.charAt(1));
  return sol;
}

byte getByte(char nibbleSup, char nibbleInf){
  char nibble;
  byte ret;
  byte sol;

  for(int i=0; i<2; i++){
    if(i == 0){
      nibble = nibbleSup;
    }else{
      nibble = nibbleInf;
    }
    
    switch(nibble){
      case '0':
        ret = 0x00;
        break;
      case '1':
        ret = 0x01;
        break;
      case '2':
        ret = 0x02;
        break;
      case '3':
        ret = 0x03;
        break;
      case '4':
        ret = 0x04;
        break;
      case '5':
        ret = 0x05;
        break;
      case '6':
        ret = 0x06;
        break;
      case '7':
        ret = 0x07;
        break;
      case '8':
        ret = 0x08;
        break;
      case '9':
        ret = 0x09;
        break;
      case 'A':
        ret = 0x0A;
        break;
      case 'B':
        ret = 0x0B;
        break;
      case 'C':
        ret = 0x0C;
        break;
      case 'D':
        ret = 0x0D;
        break;
      case 'E':
        ret = 0x0E;
        break;
      case 'F':
        ret = 0x0F;
        break;
      default:
        break;
    }
    
    if(i == 0){
      sol = ret << 4;
    }else{
      sol = sol | ret;
    }
  }
  return sol;
}
/*
// format and print a time_t value, with a time zone appended.
void printDateTime(time_t t, const char *tz)
{
  char bufTime[28];
  char m[4];    // temporary storage for month string (DateStrings.cpp uses shared buffer)
  strcpy(m, monthShortStr(month(t)));
  sprintf(bufTime, "%.2d:%.2d:%.2d %s %.2d %s %d %s",
      hour(t), minute(t), second(t), dayShortStr(weekday(t)), day(t), m, year(t), tz);
  Serial.print(bufTime);
}
*/
boolean EnviarThingsboard(void){

  boolean result = true;
  char dataHTTP[32];
  char body[128];
  char val1[12], val2[12];

  SIM800Serial.listen();

  Serial.println("AT+CGATT?");
  SIM800Serial.println("AT+CGATT?");
  if(!examinarRespuesta("+CGATT: 1", 1000, 1))
  {
    return false;
  }

  Serial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  SIM800Serial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  if(!examinarRespuesta("OK", 1000, 1)){
    return false;
  }

  Serial.println("AT+SAPBR=3,1,\"APN\",\"claro.pe\"");
  SIM800Serial.println("AT+SAPBR=3,1,\"APN\",\"claro.pe\"");
  if(!examinarRespuesta("OK", 1000, 1)){
    return false;
  }

  Serial.println("AT+SAPBR=1,1");
  SIM800Serial.println("AT+SAPBR=1,1");
  if(examinarRespuesta("OK", 1500, 1)){

    Serial.println("AT+HTTPINIT");
    SIM800Serial.println("AT+HTTPINIT");
    examinarRespuesta("OK", 1000, 1);

    Serial.println("AT+HTTPPARA=\"CID\",1");
    SIM800Serial.println("AT+HTTPPARA=\"CID\",1");
    examinarRespuesta("OK", 1000, 1);

    Serial.print("AT+HTTPPARA=\"URL\",\"");
    Serial.print(URL_THINGSBOARD);
    SIM800Serial.print("AT+HTTPPARA=\"URL\",\"");
    SIM800Serial.print(URL_THINGSBOARD);
    SIM800Serial.println("\"");
    examinarRespuesta("OK", 1000, 1);

    Serial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
    SIM800Serial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
    examinarRespuesta("OK", 1000, 1);
    
    dtostrf(latitud, 1, 7, val1);
    dtostrf(longitud, 1, 7, val2);

    char bufTime[28] = "Sunday";
    
    char m[4];    // temporary storage for month string (DateStrings.cpp uses shared buffer)
    strcpy(m, monthShortStr(month(timePET)));
    sprintf(bufTime, "%.2d:%.2d:%.2d %s %.2d %s %d ",
    hour(timePET), minute(timePET), second(timePET), dayShortStr(weekday(timePET)), day(timePET), m, year(timePET));
    
    sprintf(body, "{\"latitude\":%s,\"longitude\":%s,\"time\":\"%s\"}", val1, val2, bufTime);
    sprintf(dataHTTP,"AT+HTTPDATA=%d, 10000", strlen(body));

    Serial.println(dataHTTP);
    postToFirebase("XD");
    SIM800Serial.println(dataHTTP);
    if(examinarRespuesta("DOWNLOAD", 1000, 1)){
      Serial.println(body);
      SIM800Serial.println(body);
      examinarRespuesta("OK", 1000, 1);
    }

    Serial.println("AT+HTTPSSL=0");
    SIM800Serial.println("AT+HTTPSSL=0");
    examinarRespuesta("OK", 1000, 1);

    Serial.println("AT+HTTPACTION=1");
    SIM800Serial.println("AT+HTTPACTION=1");
    examinarRespuesta("200", 3000, 1);
    delay(20);

    Serial.println("AT+HTTPTERM");
    SIM800Serial.println("AT+HTTPTERM");
    examinarRespuesta("OK", 1000, 1);
  }else{
    result = false;
  }

  Serial.println("AT+SAPBR=0,1");
  SIM800Serial.println("AT+SAPBR=0,1");
  if(!examinarRespuesta("OK", 1000, 1)){
    result = false;
  }

  return result;
} 

boolean examinarRespuesta(char* label, long timeOut, boolean empty){
  boolean result;
  SIM800Serial.setTimeout(timeOut); //En milisegundos
  if(SIM800Serial.find(label)){
    Serial.print(label);
    Serial.println(" is True ! ");
    result = true;
  }else{
    Serial.print(label);
    Serial.println(" is False ! ");
    result = false;
  }
  if(SIM800Serial.overflow()){
    emptyBuffer();
  }
  if(empty){
    emptyBuffer();
  }
  return result;
}

void emptyBuffer(){
  while(SIM800Serial.available()!=0){
    uint8_t c = SIM800Serial.read();
  }
}



void postToFirebase(String data1)
{

boolean USE_SSL = true;
const String FIREBASE_HOST  = "https://cowiot-default-rtdb.firebaseio.com/";
const String FIREBASE_SECRET  = "kJdl471CcoCgHfs3Y50425hShfFXc6TFHwnRNpXL";

  String t = "30";
  String h = "80";
 
  data1 = "{";
  data1 += "\"temprature\":\"" + t + "\",";
  data1 += "\"humidity\":\"" + h + "\"";
  data1 += "}";
  //Start HTTP connection
  SIM800Serial.println("AT+HTTPINIT");
  //waitResponse();
  delay(500);
  //Enabling SSL 1.0
  if(USE_SSL == true){
    SIM800Serial.println("AT+HTTPSSL=1");
    //waitResponse();
    delay(500);
  }
  //Setting up parameters for HTTP session
  SIM800Serial.println("AT+HTTPPARA=\"CID\",1");
  //waitResponse();
  delay(500);
  //Set the HTTP URL - Firebase URL and FIREBASE SECRET
  SIM800Serial.println("AT+HTTPPARA=\"URL\","+FIREBASE_HOST+".json?auth="+FIREBASE_SECRET);
  //waitResponse();
  delay(500);
  //Setting up re direct
  SIM800Serial.println("AT+HTTPPARA=\"REDIR\",1");
  //waitResponse();
  delay(500);
  //Setting up content type
  SIM800Serial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  //waitResponse();
  delay(500);
  //Setting up Data Size
  //+HTTPACTION: 1,601,0 - error occurs if data length is not correct
  SIM800Serial.println("AT+HTTPDATA=" + String(data1.length()) + ",10000");
  //waitResponse("DOWNLOAD");
  delay(500);
  //Sending Data
  SIM800Serial.println(data1);
  //waitResponse();
  delay(500);
  //Sending HTTP POST request
  SIM800Serial.println("AT+HTTPACTION=1");
  
  for (uint32_t start = millis(); millis() - start < 20000;){
    while(!SIM800Serial.available());
    String response = SIM800Serial.readString();
    if(response.indexOf("+HTTPACTION:") > 0)
    {
      Serial.println(response);
      break;
    }
  }
    
  delay(500);
  //+HTTPACTION: 1,603,0 (POST to Firebase failed)
  //+HTTPACTION: 0,200,0 (POST to Firebase successfull)
  //Read the response
  SIM800Serial.println("AT+HTTPREAD");
  //waitResponse("OK",1000);
  delay(500);
  //Stop HTTP connection
  SIM800Serial.println("AT+HTTPTERM");
  //waitResponse("OK",1000);
  delay(500);
}

boolean waitResponse(String expected_answer, unsigned int timeout) //uncomment if syntax error (esp8266)
{
  uint8_t x=0, answer=0;
  String response;
  unsigned long previous;
    
  //Clean the input buffer
  while(SIM800Serial.available() > 0) SIM800Serial.read();
  previous = millis();
  do{
    //if data in UART INPUT BUFFER, reads it
    if(SIM800Serial.available() != 0){
        char c = SIM800Serial.read();
        response.concat(c);
        x++;
        //checks if the (response == expected_answer)
        if(response.indexOf(expected_answer) > 0){
            answer = 1;
        }
    }
  }while((answer == 0) && ((millis() - previous) < timeout));

  Serial.println(response);
  return answer;
}
