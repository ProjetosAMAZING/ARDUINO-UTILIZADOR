
#include <Arduino.h>
#include "atomic.h"

#include <SPI.h>
#include "rf69_module.h"
#include "QueueList.h"



void writeRegister(uint8_t address,uint8_t value);
uint8_t readRegister(uint8_t address);
void setMode(uint8_t state);
void isr0(void);
void waiToReceive (void);

uint8_t temp3 = 0;
uint8_t temp4 = 0;
uint32_t temp5 = 0;
uint32_t temp6 = 0;

uint8_t temp7 = 0;
uint8_t temp8 = 0;
uint32_t temp9 = 0;
uint32_t temp10 = 0;


uint8_t len = 0;
int sum_packet = 0;
uint8_t sum_sent = 0;
uint8_t contador = 0;
uint8_t npackets = 0;

static QueueList <int> velocityQueue;
static QueueList <uint8_t> carState;

unsigned long ttime ;
char msgACK [] = "OK";

int numb_packet_rcv;
int numb_packet_sent;

uint8_t i ;


uint8_t rf69_state = M_STDBY;
static volatile uint8_t payload = 0;
volatile uint8_t packetSent = 0;

void configSPI(void){
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();

  pinMode(slavePin, OUTPUT);
  digitalWrite(slavePin, HIGH);

}

void configModule(void)
{

writeRegister(RegOpMode,opMode_STDBY); // Sequencer ON - - Listen off - Mode stand by
setMode(M_STDBY);
writeRegister(RegDataModul, packetMode | FSK | shapeNONE); // PACKET-MODE - FSK - NO SHAPPING

writeRegister(RegBitrateMsb,0x02);
writeRegister(RegBitrateLsb, 0x40); // bitRate = 4.8kbs - defalt ver tabela pagina 20. 0x1A 0x0B

writeRegister(RegFdevMsb, 0x03); //35Khz -- depois alterar  = Fdev/Fstep // altertar
writeRegister(RegFdevLsb, 0x33);

writeRegister(RegFrMsb,Freq_MSB); // Colucar a frequencia a 433Mhz/434Mhz
writeRegister(RegFrMid,Freq_MID);
writeRegister(RegFrLsb,Freq_LSB);

//set OutputPower
writeRegister(RegPaLevel,  PA2_ON | PA1_ON | 0x1F); //13dBm
writeRegister(0x58,0x2D);

//writeRegister(RegOcp,ocpOFF |0b1010); //default value;

writeRegister(RegRxBw, RXBW_DCCFREQ_010| RXBW_MANT_16 | RXBW_EXP_2);

//interrupts;
  pinMode(7, INPUT);
attachInterrupt(digitalPinToInterrupt(7), isr0, RISING);
interrupts();
writeRegister(RegDioMapping2, CLK_OUT_OFF);
writeRegister(RegDioMapping1, DIO0_01);

writeRegister(RegIrqFlags2,FIFO_OVERRUN);

writeRegister(RegRssiThresh, 220);

writeRegister(RegSyncConfig,SyncON |AUTO_FIFOFILL| SyncSize_2 );
writeRegister(RegSyncValue1,0x2D);
writeRegister(RegSyncValue2,100);

writeRegister(RegPacketConfig1,0x80 | NONE_CODE | crcON | crc_autoclear_ON);// ver melhor isto

writeRegister(RegPayloadLength,64);

writeRegister(RegFifoThresh,0x01);
writeRegister(RegPacketConfig2,AUTORXRESTART_ON |AES_OFF);


}


int npackRcv(){
  return numb_packet_rcv;
}
uint8_t checkModule(void)
{
  if(readRegister(RegIrqFlags1)&&MODEREADY != 0)
  {
    Serial.println(readRegister(RegIrqFlags1));
  return 1;
  }
  else
  return 0;
}

void waiToReceive (void){
  setMode(M_STDBY);
  payload = 0;
 // Serial.println("start Receiving");

  writeRegister(RegDioMapping1, DIO0_01);

  setMode(M_RX);
/*
  while(payload==0);

  setMode(M_STDBY);

  payload=0;

  Serial.println("RECEBI");

  readFifo();
*/
}

/*
uint8_t sendwithRetry(uint8_t vel)
{
  for(uint8_t tries = 0; tries <= 5 ; tries ++)
  {
    sendMessage(vel,44);

    ttime = millis();

    while(millis()-ttime <= 2000)
    {
      if(receiveDone())
      {
        return 1;
      }
    }

  }
  return 0;
}
*/

uint8_t receiveDone()
{

  if(rf69_state == M_RX && payload== 1)
  {
    setMode(M_STDBY);
    payload=0;
    //Serial.print("recebi !");
    return 1;
  }
  else if(rf69_state == M_RX)
  return 0;
  else
  {
    waiToReceive();
    return 0;
  }
}

void readtoFifo()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
   {
     digitalWrite(8, LOW);
     SPI.transfer(0);

     //Serial.println(SPI.transfer(0));
     unsigned int numb_packet_rcv = SPI.transfer(0)<<7;
    // numb_packet_rcv |= SPI.transfer(0);

   //  Serial.print("packet loss: ");
     //Serial.println(numb_packet_sent/numb_packet_rcv);

     velocityQueue.push((uint8_t)SPI.transfer(0));
     carState.push((uint8_t)SPI.transfer(0));

     digitalWrite(8, HIGH);
   }



}

void resetPacketsSent()
{
  numb_packet_sent =0;
}

void readMessage(uint8_t & vel, uint8_t & carS,uint32_t &lat, uint32_t &lg, uint8_t &h, uint8_t &s, uint8_t &m , uint8_t & gpspeed, uint8_t &gpstate, uint8_t &gpsQual)
{

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
   {
     digitalWrite(8, LOW);
     SPI.transfer(0);
     len = SPI.transfer(0);
  
     //Serial.print(len);
     //Serial.print(" ");
     unsigned int numb_packet_rcv = SPI.transfer(0)<<7;
     numb_packet_rcv |= SPI.transfer(0);
/*
    
*/
     vel = SPI.transfer(0);
     carS = SPI.transfer(0);


    lat = 0x00;
    lg = 0x00;
        

      temp3 = SPI.transfer(0);
       temp4 = SPI.transfer(0);
       temp5 = SPI.transfer(0);
       temp6 = SPI.transfer(0);
      
      temp7 = SPI.transfer(0);
     temp8 = SPI.transfer(0);
         temp9 = SPI.transfer(0);
       temp10 = SPI.transfer(0);
      
       
        lat = (temp3&0x0FF)| ((temp4<<8)&0x00FF00)|((temp5<<16)&0x00FF0000)|((temp6<<24)&0x00FF000000);
         lg= (temp7&0x0FF)| ((temp8<<8)&0x00FF00)|((temp9<<16)&0x00FF0000)|((temp10<<24)&0x00FF000000);
       /*Serial.print(temp6,HEX);
      Serial.print(temp5,HEX);
      Serial.print(temp4,HEX);
      Serial.print(temp3),HEX;
      Serial.print(" , ");
      Serial.print(temp10,HEX);
      Serial.print(temp9,HEX);
      Serial.print(temp8,HEX);
      Serial.print(temp7,HEX);
      Serial.println();*/
     
      //Serial.print(lg);
      //Serial.print(" , ");
      //Serial.println(lat);


      
     /* Serial.println();
      Serial.print(temp3);
      Serial.print(temp4);
      Serial.print(temp5);
      Serial.print(temp6);
      Serial.print(" , ");
      Serial.print(temp7);
      Serial.print(temp8);
      Serial.print(temp9);
      Serial.print(temp10);
      Serial.println();
      */
    /*  lat = 0;
      lat = SPI.transfer(0);
      lat = lat | (SPI.transfer(0)>>8);
      lat = lat | (SPI.transfer(0)>>16);
      lat = lat | (SPI.transfer(0)>>24);
      Serial.println();
  Serial.println(lat);
  Serial.println();
  lg = 0;
      lg = SPI.transfer(0);
      lg = lg | (SPI.transfer(0)>>8);
      lg = lg | (SPI.transfer(0)>>16);
      lg = lg | (SPI.transfer(0)>>24);
*/
    //lat = SPI.transfer(0);
    //lg = SPI.transfer(0);
    
    h = SPI.transfer(0);
    m = SPI.transfer(0);
    
    s = SPI.transfer(0);
    gpspeed = SPI.transfer(0);
    
    gpstate = SPI.transfer(0);
  
    gpsQual = SPI.transfer(0);
     digitalWrite(8, HIGH);
   }

   writeRegister(RegIrqFlags2,FIFO_OVERRUN);

}

  void sendMessage(uint8_t vel, uint8_t car_state)
{


    setMode(M_STDBY);

   writeRegister(RegIrqFlags2,FIFO_OVERRUN);
   Serial.flush();

    writeRegister(RegDioMapping1, DIO0_00);

//   Serial.print("A enviar velocidade: ");
//    Serial.print(vel_pretendida);
  //  Serial.print(" seq_frame: ");
    //Serial.println(SEQ_FRAME);



   ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      digitalWrite(slavePin,LOW);
      SPI.transfer(RegFifo|SPI_WRITE);

      SPI.transfer(5);
      SPI.transfer((numb_packet_sent&0xFF00)>>7);
      SPI.transfer((numb_packet_sent&0xFF));

      //tamanho
     // Serial.print(vel);
      SPI.transfer(vel);
      SPI.transfer(car_state);


      digitalWrite(slavePin, HIGH);
    }

      
    setMode(M_TX);

    while(packetSent == 0);

    numb_packet_sent++;

    //Serial.println("PacketSent");
    writeRegister(RegIrqFlags2,FIFO_OVERRUN);

  packetSent = 0;

  return;
}


void setMode(uint8_t state)
{
  if(state != rf69_state)
  {
    uint8_t value = readRegister(RegOpMode);
    value &= MODE_BITS;

    switch (state) {
      case M_STDBY:
        value |= opMode_STDBY;
      //  Serial.println("STBY");
        break;
      case M_RX:
        value |= opMode_RX;
        //NORMAL MODE WITH RX
        writeRegister(RegTestPa1,0x55);
        writeRegister(RegTestPa2,0x70);
     //   Serial.println("RX");
        break;
      case M_TX:
        value |= opMode_TX;
        //MAX OUTPUT POWER 20DBM
        writeRegister(RegTestPa1,0x5D);
        writeRegister(RegTestPa2,0x7C);
       // Serial.println("TX");
        break;
        default:
          //Serial.println("Nhient");
          break;
    }

    writeRegister(RegOpMode,value);
    while(readRegister(RegIrqFlags1)&&MODEREADY == 0);


    rf69_state = state;
    return;
  }
  else
  return;
}


  void checkMessages(uint8_t &std , uint8_t &velc)
  {
    uint8_t vp=0;
    sum_packet = 0 ;
    contador = 0;
    sum_sent = 0;


    if(velocityQueue.isEmpty() && carState.isEmpty())
      velc = 0;
    else
    {
      npackets = velocityQueue.count();
      while(!velocityQueue.isEmpty())
      {
          if(contador == 0)
          {
            velc = velocityQueue.pop();
            contador ++;
          }
          else{
            vp = velocityQueue.pop();
            if(velc == vp)
              contador++;
            else
              contador--;
          }

          if(contador == 2)
          break;
      }
      contador = 0;

      while(!carState.isEmpty())
      {
          if(contador == 0)
          {
            std = carState.pop();
            contador ++;
          }
          else{
            vp = carState.pop();
            if(std == vp)
              contador++;
            else
              contador--;
          }

          if(contador == 2)
          break;
      }
    }
}
void isr0(void){
if(rf69_state == M_RX)
{ 
  payload = 1;
  
}
else if(rf69_state == M_TX)
{

    packetSent = 1;
   //Serial.println("SENT");
}

}

void writeRegister(uint8_t address, uint8_t value){
  digitalWrite(slavePin, LOW);
  SPI.transfer(SPI_WRITE | address);
  SPI.transfer(value);
  digitalWrite(slavePin,HIGH);
}

uint8_t readRegister(uint8_t address){
  digitalWrite(slavePin, LOW);
  SPI.transfer(address & SPI_READ);
  uint8_t value = SPI.transfer(RegFifo);
  digitalWrite(slavePin,HIGH);
  return value;
}

