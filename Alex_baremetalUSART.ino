#include <serialize.h>
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <math.h>

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      4

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.42

#define ALEX_LENGTH 26
#define ALEX_BREADTH 12.5
#define PI 3.141592654
float alexDiagonal = 0.0;
float alexCirc = 0.0;

//Colour Sensor
#define S0 22
#define S1 23
#define S2 24
#define S3 25
#define sensorOut A8

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long leftReverseTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long rightReverseTicks;
volatile unsigned long failsafe;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;
// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;
unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;
float currAng;
float targetAng;

//colour sensor data
unsigned long red = 0;
unsigned long green = 0;
unsigned long blue = 0;
char colour = 'U';
//float wallDist = 0;
//int stopDist = 13;

// Ultrasound Pins
int TRIG_PIN = 50;
//int BACK_TRIG = 51;
int ECHO_PIN = 48;
//int BACK_ECHO = 49;
float SPEED_OF_SOUND = 0.0345;

//for baremetal USART communication
char rxBuffer[256] = {0};
volatile unsigned int rxHead;
volatile unsigned int rxTail;
volatile unsigned int bytesRecvd;

char txBuffer[256] = {0};
volatile unsigned int txHead;
volatile unsigned int txTail;
//volatile unsigned int txBytes;

unsigned long computeDeltaTicks (float ang){
  unsigned long ticks = (unsigned long)((ang * alexCirc * COUNTS_PER_REV)/(360.0 * WHEEL_CIRC));
  return ticks;
}

void left(float ang, float speed){
  targetAng = check_angle() + ang;
  deltaTicks = 1;
  ccw(ang, speed);
}
void right(float ang, float speed){
  targetAng = check_angle() - ang;
  deltaTicks = 1;
  cw(ang, speed);
}
volatile TDirection dir;
/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  uint32_t inputParams[14] = {leftForwardTicks,rightForwardTicks,leftReverseTicks,rightReverseTicks,
  leftForwardTicksTurns,rightForwardTicksTurns,leftReverseTicksTurns,rightReverseTicksTurns,forwardDist,reverseDist,
  red, green, blue, (uint32_t)colour};
  for (int i = 0; i < 14; i++){
    statusPacket.params[i] = inputParams[i];
  }
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...){
  va_list args;
  char buffer [128];

  va_start(args,format);
  vsprintf(buffer,format,args);
  sendMessage(buffer);
}

void dbprint(char *format, ...){
  va_list args;
  char buffer [128];

  va_start(args,format);
  vsprintf(buffer,format,args);
  TPacket messagePacket;
  char message[128];
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  //sendResponse(&messagePacket);
  //sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 18 and 19
void enablePullups()
{
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR()
{
  if (dir == FORWARD){
    leftForwardTicks++; 
  } else if (dir == BACKWARD){
    leftReverseTicks++;    
  } else if (dir == LEFT){
    leftReverseTicksTurns++;
  } else if (dir == RIGHT){
    leftForwardTicksTurns++;
  }
}

void rightISR()
{
  if (dir == FORWARD){
    rightForwardTicks++;
    forwardDist = (unsigned long) ((float) rightForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
    failsafe = millis();
  } else if (dir == BACKWARD){
    rightReverseTicks++;
    reverseDist = (unsigned long) ((float) rightReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == LEFT){
    rightForwardTicksTurns++;
  } else if (dir == RIGHT){
    rightReverseTicksTurns++;
  }
}
//    Serial.print("Distance travelled: ");
//    Serial.println((float)leftForwardTicks * WHEEL_CIRC / (float)COUNTS_PER_REV);
//  Serial.print("RIGHT: ");
//  Serial.println(rightTicks);
//  Serial.print("Distance travelled: ");
//  Serial.println((float)rightTicks * WHEEL_CIRC / (float)COUNTS_PER_REV);

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  cli();
  EICRA = 0b10100000;
  EIMSK |= 0b00001100;
  sei();
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.

}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.
ISR(INT2_vect) {
  rightISR();
}

ISR(INT3_vect){
  leftISR();
}

// Implement INT2 and INT3 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  UCSR0A = 0;
  UCSR0C = 0b00000110;
  UBRR0L = 103;
  UBRR0H = 0;
  
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  UCSR0B = 0b10011000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

ISR(USART0_RX_vect) {
  rxBuffer[rxTail] = UDR0;
  rxTail = (rxTail + 1) % 256;
}

int readSerial(char *buffer)
{
  bytesRecvd = 0;
  while (rxHead != rxTail){
    buffer[bytesRecvd] = rxBuffer[rxHead];
    rxHead = (rxHead + 1) % 256;
    bytesRecvd++;
  }
  return bytesRecvd;
}

// Write to the serial port. Replaced later with
// bare-metal code
ISR(USART0_UDRE_vect) {
  if (txHead != txTail) {
    UDR0 = txBuffer[txHead];
    txHead = (txHead + 1) % 256;
  } else {
    UCSR0B &= 0b11011111;
  }
}


void writeSerial(const char *buffer, int len)
{
  //Serial.write(buffer, len);
  for (int curr = 0; curr < len; curr++) {
    txBuffer[txTail] = buffer[curr];
    txTail = (txTail + 1) % 256;
  }
  UCSR0B |= 0b00100000;
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

void clearColorCounters(){
  red = 0;
  green = 0;
  blue = 0;
  colour = 'U';
}
// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
  /*switch(which)
  {
    case 0:
      clearCounters();
      break;

    case 1:
      leftTicks=0;
      break;

    case 2:
      rightTicks=0;
      break;

    case 3:
      leftRevs=0;
      break;

    case 4:
      rightRevs=0;
      break;

    case 5:
      forwardDist=0;
      break;

    case 6:
      reverseDist=0;
      break;
  }*/
}
// Intialize Alex's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        failsafe = millis();
        forward((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
        sendOK();
        backward((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
        sendOK();
        left((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
        sendOK();
        right((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_STOP:
        sendOK();
        stop();
      break;        
    case COMMAND_GET_STATS:
        sendOK();
        clearColorCounters();
        sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
      break;
    case COMMAND_COLOR_SENSOR:
        sendOK();
        readColour();
        sendStatus();
      break;
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setupCSensor() {
 
  // Set S0 - S3 as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  // Set Sensor output as input
  pinMode(sensorOut, INPUT);
  
  // Set Pulse Width scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,HIGH);
}
 
// Function to read Red Pulse Widths
unsigned long getRedPW() {
 
  // Set sensor to read Red only
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  //PORTB = 0b00000100;
  // Define integer to represent Pulse Width
  unsigned long PW = 0;
  // Read the output Pulse Width
  for (int i = 0; i < 5; i++){
    PW += pulseIn(sensorOut, LOW);
    delay(20);
  }
  
  // Return the value
  return PW/5;
 
}
 
// Function to read Green Pulse Widths
unsigned long getGreenPW() {
 
  // Set sensor to read Green only
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  //PORTB = 0b00110100;
  // Define integer to represent Pulse Width
  unsigned long PW = 0;
  // Read the output Pulse Width
  for (int i = 0; i < 5; i++){
    PW += pulseIn(sensorOut, LOW);
    delay(20);
  }
  
  // Return the value
  return PW/5;
}
 
// Function to read Blue Pulse Widths
unsigned long getBluePW() {
 
  // Set sensor to read Blue only
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  //PORTB = 0b00100100;
  // Define integer to represent Pulse Width
  unsigned long PW = 0;
  // Read the output Pulse Width
  for (int i = 0; i < 5; i++){
    PW += pulseIn(sensorOut, LOW);
    delay(20);
  }
  
  // Return the value
  return PW/5;
}

char identifyColour(float Red, float Green, float Blue) 
{ 
  if (Green > 0.95 && Green < 1.10) //Red > 0.95 && Red < 1.05 && Green > 0.95 && Green < 1.05 && Blue < 0.85 
  { 
    dbprintf("White\n");
    return 'W'; 
  } 
  if(Green > 1.25) //Red > 0.95 && Red < 1.05 && Green > 1.95 && Green < 2.05 && Blue > 1.45
  {
    dbprintf("Red\n");
    return 'R';
  }
  if(Green > 0.6 && Green <= 0.95)//Red > 0.95 && Red < 1.05 && Green > 0.65 && Green < 0.85 && Blue > 0.65 && Blue < 0.85
  {
    dbprintf("Green\n");
    return 'G';
  } 
  if(Blue < 0.70) //Green > 0.65 && Green < 0.75 && Blue > 0.45 && Blue < 0.65
  {
    dbprintf("WALL\n");
    return 'U';
  }
  return '?';
} 

void readColour() {
 digitalWrite(S2,HIGH); 
 digitalWrite(S3,LOW); 
 //white = (double)pulseIn(sensorOut,LOW); 
 delay(20); 
  
  // Read Red Pulse Width 
  digitalWrite(S2,LOW); 
  digitalWrite(S3,LOW); 
  red = (double)pulseIn(sensorOut,LOW); 
  // Delay to stabilize sensor 
  delay(20); 
   
  // Read Green Pulse Width 
  digitalWrite(S2,HIGH); 
  digitalWrite(S3,HIGH); 
  green = (double)pulseIn(sensorOut,LOW); 
  // Delay to stabilize sensor 
  delay(20); 
   
  // Read Blue Pulse Width 
  digitalWrite(S2,LOW); 
  digitalWrite(S3,HIGH); 
  blue = (double)pulseIn(sensorOut,LOW); 
  // Delay to stabilize sensor 
  delay(20); 
 
  // Serial.print(red); 
  // Serial.print(" "); 
  // Serial.print(green); 
  // Serial.print(" "); 
  // Serial.print(blue); 
   
  // Print output to Serial Monitor 
  colour = identifyColour((float)red/(float)red, (float)green/(float)red, (float)blue/(float)red); 
  delay(500);
} 



void setupUltra() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}

float getDistance() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN , LOW);
  unsigned long microsecs = pulseIn(ECHO_PIN, HIGH, 3000);
  //wallDist = microsecs * SPEED_OF_SOUND / 2;
  float cms = (float)microsecs * SPEED_OF_SOUND/2;
  //dbprintf("Distance: %d\n", cms);
  return cms;
}

//float getBackDistance() {
//  digitalWrite(BACK_TRIG, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(BACK_TRIG, LOW);
//  unsigned long microsecs = pulseIn(BACK_ECHO, HIGH, 3000);
//  float cms = microsecs * SPEED_OF_SOUND/2;
//  //dbprintf("Distance: %d\n", cms);
//  return cms;
//}

void setup() {
  // put your setup code here, to run once:
  //setupGyro();
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));

  alexCirc = PI * alexDiagonal;

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  initializeState();
  setupCSensor();
  setupUltra();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

 //forward(0, 100);

// Uncomment the code below for Week 9 Studio 2


 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 
  if(deltaDist > 0) 
 { 
  if(dir==FORWARD) 
  { 
   float wallDist = getDistance();
   delay(1);
   //dbprintf("Distance: %d\n", (int)wallDist);
   //printf("Distance: %d\n", (int)wallDist);
   if(forwardDist > newDist || (wallDist != 0 && wallDist <= 13) || millis() - failsafe > 1000) 
   { 
    dbprintf("Distance: %d\n", (int)wallDist);
    deltaDist=0;
    newDist=0;
    stop(); 
   } 
  } 
  else 
   if(dir == BACKWARD)
   { 
    //double wallDist = getBackDistance();
    if(reverseDist > newDist/* || (wallDist != 0 && wallDist <= 12)*/) 
    {
     //dbprintf("Distance: %d\n", (int)wallDist); 
     deltaDist=0; 
     newDist=0; 
     stop(); 
    }
   } 
   else 
    
 if(dir == STOP) 
    { 
     deltaDist=0; 
     newDist=0; 
     stop(); 
    } 
 }

 if (deltaTicks > 0){
  if (dir == LEFT){
    float angle = check_angle();
    //dbprintf("%d\n",(int)angle);
    if (angle >= targetAng){
      deltaTicks = 0;
      //targetTicks = 0;
      stop();
    }
  } else if (dir == RIGHT){
    float angle = check_angle();
    //dbprintf("%d\n",(int)angle);
    if (angle <= targetAng){
      deltaTicks = 0;
      //targetTicks = 0;
      stop();
    }
  } else if (dir == STOP){
      deltaTicks = 0;
      targetTicks = 0;
      stop();
  }
 }      
}
