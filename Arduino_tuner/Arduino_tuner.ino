/*
 * AD5MQ TUNER driving an AT-897 board
 * commands:
 *  C - cap up - show current
 *  c - cap down  - show current
 *  L - ind up - show current
 *  l - ind down - show current
 *  d - display current setting
 *  s - show swr
 *  S - show fwd, ref counts
 *  t- tune
 * 
 */
#include <EEPROM.h>
#include <math.h>
#include "Arduino.h"

#define COMMAND_BUFFER_SIZE 50

int relayBridge = A4;
int LED = A5;
int CapRelay = A3;

int FWDPIN = A0;
int REFPIN = A1;

typedef struct 
{
  byte relay;
  byte state;
}relayState;

relayState L[7];
relayState C[7];
byte control_port_buffer[COMMAND_BUFFER_SIZE];
int control_port_buffer_index = 0;
unsigned long last_serial_receive_time = 0;
int fwd, ref;

void setup() {
  Serial.begin(9600);
  
  #ifdef DEBUG
    Serial.println("Tuner setup");
  #endif
  
  delay(500);
  
  //change bridge pin to output
  pinMode(relayBridge, OUTPUT);
  //change LED and bridge pin to output
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW); //LED on
  //change input relay to input to tri-state
  pinMode(CapRelay, INPUT);

  //setup all digital relay pins as input (tri-stateed)
  for (int i = 0; i < 14; i++)
  {
    //initialize relay states
    digitalWrite(relayBridge, LOW); //able to "RESET" relays to NC position bypassing inductor, disconnecting capacitor, bridge high
    if (i < 7)
    {
      L[i].relay = i;
      L[i].state = 0;
      pinMode(i, OUTPUT);
      delay(20);
      digitalWrite (i, LOW); //reset relay
      delay(100); //give relay time to settle
      digitalWrite (i, HIGH);
    }
    else  //capacitor relays
    {
      C[i-7].relay = i;
      C[i-7].state = 0;
      pinMode(i, OUTPUT);
      delay(20);
      digitalWrite (i, LOW); //reset relay
      delay(100);
      digitalWrite (i, HIGH);
    }
    //deactivate relay pin drive
    pinMode(i, INPUT);
  }//for
  
}//setup()

void loop() {
  // put your main code here, to run repeatedly:
  fwd = analogRead(FWDPIN);
  ref = analogRead(REFPIN);
  checkSerial();
  delay(100);
  
}//loop()

void ToggleRelay (relayState *CurrentState)
{
  if (CurrentState->state > 0) //relay set - then reset
  {
    digitalWrite(relayBridge, LOW); //able to "RESET" relays to NC position 
    pinMode(CurrentState->relay, OUTPUT);
    delay(20);
    digitalWrite (CurrentState->relay, LOW); //reset relay
    delay(100); //give relay time to settle
    digitalWrite (CurrentState->relay, HIGH); 
    delay(20);
    pinMode(CurrentState->relay, INPUT);
    CurrentState->state = 0;
  }
  else
  {
    digitalWrite(relayBridge, HIGH); //able to "SET" relays to NO position 
    pinMode(CurrentState->relay, OUTPUT);
    delay(20);
    digitalWrite (CurrentState->relay, HIGH); //set relay
    delay(100); //give relay time to settle
    digitalWrite (CurrentState->relay, LOW); 
    delay(20);
    pinMode(CurrentState->relay, INPUT);
    CurrentState->state = 1;
  }
 
}//ToggleRelay

void clear_command_buffer(){
  control_port_buffer_index = 0;
  control_port_buffer[0] = 0;
}//clear_command_buffer

void checkSerial()
{
  int incomingByte = 0;
  if (Serial.available() > 0){
    incomingByte = Serial.read();
    last_serial_receive_time = millis();
    if ((incomingByte != 10) && (incomingByte != 13)) { 
       control_port_buffer[control_port_buffer_index] = incomingByte;
       control_port_buffer_index++;
    }
    else{
      processCommand();
      clear_command_buffer();
    }
  }//if a byte is there
  
}//checkSerial

void processCommand(){
  int cNow, lNow;
  switch (control_port_buffer[0]){
    case 'C':
      cNow = getCurrentValue (C);
      cNow++;
      if (cNow > 127)
        cNow = 0;
      setStates (C, cNow);
      Serial.print ("Current capacitance : ");
      Serial.print (cNow*10);
      Serial.println (" pF");
      break;
    
    case 'c':
      cNow = getCurrentValue (C);
      cNow--;
      if (cNow < 0)
        cNow = 127;
      setStates (C, cNow);
      Serial.print ("Current capacitance : ");
      Serial.print (cNow*10);
      Serial.println (" pF");
      break;

    case 'L':
      lNow = getCurrentValue (L);
      lNow++;
      if (lNow > 127)
        lNow = 0;
      setStates (L, lNow);
      Serial.print ("Current inductance : ");
      Serial.print ((float)lNow*0.10);
      Serial.println (" uH");
      break;
    
    case 'l':
      lNow = getCurrentValue (L);
      lNow--;
      if (lNow < 0)
        lNow = 127;
      setStates (L, lNow);
      Serial.print ("Current capacitance : ");
      Serial.print ((float)lNow*0.10);
      Serial.println (" uH");
      break;

    case 'd':
      cNow = getCurrentValue (C);
      lNow = getCurrentValue (L);
      Serial.print ("Current capacitance : ");
      Serial.print (cNow*10);
      Serial.println (" pF");
      
      Serial.print ("Current inductance : ");
      Serial.print ((float)lNow*0.10);
      Serial.println (" uH");
       break;

    case 'S':
      Serial.print ("Current fwd counts : ");
      Serial.println (fwd);
      Serial.print ("Current ref counts : ");
      Serial.println (ref);
      break;
      
    default:
    break;
  }//switch
}//processCommand

byte getCurrentValue (relayState *stateArray){
  byte retval = 0;
  for (int i = 0; i < 7; i++){
    retval |= stateArray[i].state << i;
  }//for
  
  return retval;
}//getCurrentValue

void setStates (relayState *stateArray, byte newValue){
  byte retval = 0;
  for (int i = 0; i < 7; i++){
    stateArray[i].state = ((newValue >> i)&0x01);
    ToggleRelay (&stateArray[i]);
  }//for
  
}//setCurrentValue
