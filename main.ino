#include <Arduino.h>
#include "VSync.h"
#include <PrintEx.h>

#include "rf69_module.h"
#include "TimerOne.h"


void callback(void);

//ValueSender<4> sender;
//ValueReceiver<2> receiver;

PrintEx serial = Serial;


uint32_t lat;
uint32_t lg;
uint8_t h;
uint8_t m;
uint8_t s;
uint8_t gpspeed;
uint8_t gpstate;
uint8_t gpsQual;
const int ledPin =  13;
int ledState = LOW;

uint8_t cont = 0;
//flag Init 
uint8_t Init = 1;
uint8_t volatile canSend = 0;

uint8_t rcv = 65;
uint8_t ackn;

uint8_t ack_received = 0;
uint8_t velocidade =0;
uint8_t sentido = 1;

uint8_t velc = 0;
uint8_t carstate = 0;

int vel_ard = velc;
int sent_pro = sentido;
int vel_pro = velocidade;
int latt = lat;
int lgg = lg;
int gps_state = 0;
volatile uint8_t timeout = 0;

long tt=0;
uint8_t l;
int numb_packet = 0;

void setup() {

  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  while(!Serial);
  //configurar SPI
  configSPI();
 

  //Configurar o modulo
  configModule();
 
  // Ver se o Modulo responde
  while(checkModule() != 1)
  {Serial.print("HERE");}
  // Timer 1/3s para enviar pacotes
  Timer1.initialize(333333);
  Timer1.attachInterrupt(callback);
  velocidade = 0;
 // Serial.print("YO");
 while(!Serial);
  //Variaveis para GUI
  
  /*sender.observe(vel_ard);
  receiver.observe(vel_pro);
  receiver.observe(sent_pro);
  sender.observe(latt);
  sender.observe(lgg); 
  sender.observe(gps_state);*/
}

void loop(){

  if(canSend == 1)
  {          //                   Serial.print("?");
        // receiver.sync();
      velocidade = (uint8_t)vel_pro;
      sentido = (uint8_t) sent_pro;
      sendMessage(0,1); 
      canSend= 0;
      waiToReceive();
      if(ack_received == 0)
      {
        timeout++;
        if(timeout>10)
        {
         // Serial.println("notsent");
         resetPacketsSent();
         timeout = 0;
        }
      }
      else
      ack_received =0;

      //Serial.print("??");
  }

  else{
    if(receiveDone() == 1)
    {
        timeout = 0;
      readMessage(velc,carstate,lat,lg,h,m,s,gpspeed,gpstate,gpsQual);
      latt = (int)lat;
      lgg = (int)lg;
      vel_ard = (int) velc;
      gps_state = (int)gpspeed;
      

      //Serial.print("velocidade % : "); Serial.print(velc); Serial.print(" Estado : "); Serial.print(carstate); Serial.print("sentido: "); Serial.println( sentido); 

     
      Serial.print(carstate);
      Serial.print(",");
      serial.printf("%3i",velc);
      Serial.print(",");
      Serial.print(sentido);
      Serial.print(",");
      Serial.print(lat);
       Serial.print(",");
      Serial.print(lg);
       Serial.print(",");
       serial.printf("%2i",h);
       Serial.print(",");
      serial.printf("%2i",m);
       Serial.print(",");
      serial.printf("%2i",s);
       Serial.print(",");
       
      serial.printf("%3i",gpspeed);
      Serial.print(",");
      Serial.print(gpstate);
      Serial.print(",");
      serial.printf("%3i",gpsQual);
      Serial.println();   
   
    /*  Serial.print(s);
      Serial.print(" ");
      Serial.println(gpstate);
      Serial.println(lat);
      Serial.println(lg);  */ 
     
    
     // sender.sync();
      ack_received = 1;
    }
    
  }

}


void callback(void)
{
  canSend = 1;
}


