#include <HardwareSerial.h>
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
TaskHandle_t Tarea0;
TaskHandle_t Tarea1;
//////////////////////////////////////////
unsigned int digitalPins = 0;
int analogPins[7] = {0};
unsigned int rpm = 0; // NIVEL DE BATERIA DESDE EL BMS
unsigned int kpa = 0; // 99.2
unsigned int tps =0; //965; // 96.5
unsigned int clt = 0;  // 80 - 100
int clt1=0;
unsigned int textCounter = 0;
#define RXD2 16 //DEFINIR RX2 DEL ESP32
#define TXD2 17 //DEFINIR TX2 DEL ESP32
HardwareSerial SerialPort(2);
byte aux=0;
byte batteryLevel=0;
void setup() {
  Serial.begin(9600);
//SerialPort.begin(9600, SERIAL_8N1, rx, tx);
  SerialPort.begin(9600, SERIAL_8N1, 18, 19); //19=rx, 18=tx
  xTaskCreatePinnedToCore(loop0,"Tarea_0",1500,NULL,1,&Tarea0,0); // Core 0
  xTaskCreatePinnedToCore(loop1,"Tarea_1",10000,NULL,1,&Tarea1,1); // Core 1
  SerialBT.begin("qDash&BMS_wst");
  Serial1.begin(9600,SERIAL_8N1,RXD2,TXD2);//INICIAR TX2 Y RX2
  //Serial2.begin(9600,SERIAL_8N1,RXD3,TXD3);//INICIAR TX3 Y RX3
  Serial.setTimeout(50); 
  pinMode(14,INPUT);
  pinMode(27,INPUT);
  pinMode(26,INPUT);
  pinMode(25,INPUT);
  pinMode(33,INPUT);
  pinMode(32,INPUT);
}
void loop()
{
  ReadDigitalStatuses();
  ReadAnalogStatuses();
  SendCANFramesToSerial();
  rpm=aux;
  //rpm=Serial.read(); //   LECTURA DEL NIVEL DE BATERIA DESDE EL BMS
  clt1 =analogRead(2);
  clt=map(clt1,0,4095,0,1024);
  Serial.println(clt);
  if (textCounter++ > 4000)
  {
    textCounter = 0;
  }
  
}


void ReadDigitalStatuses()
{
  int a,b,c,d,e,f,g,h=0;
  int d1=digitalRead(14);//brake
  int d2=digitalRead(27);//izq
  int d3=digitalRead(26);//der
  int d4=digitalRead(25);///////luces
  int d5=digitalRead(33);//R
  int d6=digitalRead(32);//D
  if(d1==HIGH){a=0b00000001;}else{a=0;}
  if(d2==HIGH){b=0b00000100;}else{b=0;}
  if(d3==HIGH){c=0b00001000;}else{c=0;}
  if(d2==HIGH && d3==HIGH){h=0b00000010;}else{h=0;}//neutro
  if(d4==HIGH){d=0b00010000;}else{d=0;}
  if(d5==HIGH && d6==HIGH){e=0b01000000;}else{e=0;}//neutro
  if(d5==LOW && d6==HIGH){ f=0b00100000;}else{f=0;} //retro
  if(d5==HIGH && d6==LOW){g=0b10000000;}else{g=0;} //drive
  
  digitalPins=a+b+c+d+e+f+g+h;
  //Serial.println(digitalPins);
  // read status of digital pins (1-13)
   }
void ReadAnalogStatuses()
{
  // read analog pins (0-7)
  for (int i=0; i<7; i++)
  {
    analogPins[i] = analogRead(i);
  }
}


void SendCANFramesToSerial()
{
  byte buf[8];

  // build & send CAN frames to RealDash.
  // a CAN frame payload is always 8 bytes containing data in a manner
  // described by the RealDash custom channel description XML file
  // all multibyte values are handled as little endian by default.
  // endianess of the values can be specified in XML file if it is required to use big endian values

  // build 1st CAN frame, RPM, MAP, CLT, TPS (just example data)
  memcpy(buf, &rpm, 2);
  memcpy(buf + 2, &kpa, 2);/////
  memcpy(buf + 4, &clt, 2);
  memcpy(buf + 6, &tps, 2);
  
  //const byte buf2[8] = {1,1,1,1,1,1,1,1};
  // write first CAN frame to serial
  SendCANFrameToSerial(3200, buf);

  // build 2nd CAN frame, Arduino digital pins and 2 analog values
  memcpy(buf, &digitalPins, 2);
  memcpy(buf + 2, &analogPins[0], 2);
  memcpy(buf + 4, &analogPins[1], 2);
  memcpy(buf + 6, &analogPins[2], 2);

  // write 2nd CAN frame to serial

  SendCANFrameToSerial(3201,buf);

  // build 3rd CAN frame, rest of Arduino analog values
  memcpy(buf, &analogPins[3], 2);
  memcpy(buf + 2, &analogPins[4], 2);
  memcpy(buf + 4, &analogPins[5], 2);
  memcpy(buf + 6, &analogPins[6], 2);

  // write 3rd CAN frame to serial
  SendCANFrameToSerial(3202, buf);

  // build 4th frame, this is a text extension frame
  // only send once at 1000 loops
  if (textCounter == 0)
  {
    SendTextExtensionFrameToSerial(3203, "Hello RealDash, this is Arduino sending some text data");
  }
  else if (textCounter == 1000)
  {
    SendTextExtensionFrameToSerial(3203, "Tomorrow's forecast: Lots of sun and 30 degrees centigate");
  }
  else if (textCounter == 2000)
  {
    SendTextExtensionFrameToSerial(3203, "Now Playing: Insert your favorite song info here");
  }
  else if (textCounter == 3000)
  {
    SendTextExtensionFrameToSerial(3203, "Message from Arduino: All systems running at nominal efficiency");
  }
}


void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
  SerialBT.write(serialBlockTag, 4);//4

  // the CAN frame id number (as 32bit little endian value)
  SerialBT.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  SerialBT.write(frameData, 8);

}


void SendTextExtensionFrameToSerial(unsigned long canFrameId, const char* text)
{
  if (text)
  {
    // the 4 byte identifier at the beginning of each CAN frame
    // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
    const byte textExtensionBlockTag[4] = { 0x55, 0x33, 0x22, 0x11 };
    SerialBT.write(textExtensionBlockTag, 4);

    // the CAN frame id number (as 32bit little endian value)
    SerialBT.write((const byte*)&canFrameId, 4);

    // text payload
    Serial.write(text, strlen(text) + 1);
  }
}
void loop0(void *parameter){  // Tarea0 parpadeo LED 0,3 segundos
  while(1==1){
  Serial1.write(0xDD); // Enviar el primer byte
  Serial1.write(0xA5); // Enviar el segundo byte
  Serial1.write(0x03); // Enviar el tercer byte
  Serial1.write(0x00); // Enviar el cuarto byte
  Serial1.write(0xFF); // Enviar el quinto byte
  Serial1.write(0xFD); // Enviar el sexto byte
  Serial1.write(0x77); // Enviar el séptimo byte
  Serial1.write('\n');
  delay(500);
 } 
}
void loop1(void *parameter){ 
  while(1==1){
 //   if (Serial1.available ()>0){
 //     int data=Serial1.read();
 //     SerialPort.print(data,HEX);
 //     SerialPort.print(',');
 //     if(data==10){
 //      SerialPort.println(); // Imprimir la cadena en el monitor serial
 //     }
 //   }
    if(Serial1.available()>0){
      static int count=0;
      byte data = Serial1.read();
      if(count == 24){
        aux=data;
        SerialPort.print(aux);;
        SerialPort.print('\n');
      }
        SerialPort.print(aux);
        SerialPort.print('\n');  
      if(data == 10){count=0;}
      count++; 
    }
        SerialPort.print(aux);
        SerialPort.print('\n');
  }
  
}
