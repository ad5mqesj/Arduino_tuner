/*
 * AD5MQ TUNER driving an AT-897 board
 * commands:
 *  C - cap up - show current
 *  c - cap down  - show current
 *  a - c on INPUT side
 *  A - c on OUTPUT side
 *  L - ind up - show current
 *  l - ind down - show current
 *  R - reset all to 0
 *  d - display current setting
 *  s - show swr
 *  S - show fwd, ref counts
 *  t- tune
 * 
 */
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <math.h>

#include "prototypes.h"


int CapRelay = 8; //GPA7 expander
int relayBridge = 15; //GPB7 expander

int LED = 2;        //D2 pin main board
int tuneButton = 3;//D3 pin 

int FWDPIN = A0;
int REFPIN = A1;

byte control_port_buffer[COMMAND_BUFFER_SIZE];
int control_port_buffer_index = 0;
unsigned long last_serial_receive_time = 0;
int fwd, ref;
float coeffs[3] = { 0.65, 0.0205, 0.0001 };

relayState cIn;

relayState L[7];
relayState C[7];
int tuneInProgress = 0;
int loopcount = 0;
int debugTune = 1;

Adafruit_MCP23017 mcp;
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); // Change to (0x27,20,4) for 20x4 LCD.

void setup() {
    //change LED pin to output
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH); //LED off
    
    Serial.begin(38400);
    mcp.begin();

    memset(control_port_buffer, 0, sizeof(control_port_buffer));
  
    #ifdef DEBUG
    Serial.println("Tuner setup");
    #endif
  
    delay(100);
  
    //setup analog inputs
    pinMode(FWDPIN, INPUT);
    pinMode(REFPIN, INPUT);

    //setup tune button in
    pinMode(tuneButton, INPUT_PULLUP);

    //change bridge pin to output
    mcp.pinMode(relayBridge, OUTPUT);
    mcp.digitalWrite(relayBridge, LOW); //Bridge output High initially


    //change input relay to input to tri-state after establishing state
    cIn.relay = CapRelay;
    cIn.state = 0;
    ToggleRelay(&cIn, 0);

    //setup all digital relay pins as input (tri-stateed) after establishing known state for relays
    initializeRelayStates();
  
    lcd.init();
    lcd.backlight();
    digitalWrite(LED, LOW); //LED on
}//setup()

float avswr = 0.0, fwPwr = 0.0, refPwr = 0.0;
float avgfwd = 0.0, avgRef = 0.0;

void loop() {
  char strBuf[17];
  char temp[8], temp1[8];
  
  checkSerial();
  delay(100);
  //read tune button and initiate cycle if pressed
  int val = digitalRead(tuneButton);
  if (val == 0  && tuneInProgress == 0)
  {
      tune();
  }
  avswr += getSwr();
  avgfwd += fwd;
  avgRef += ref;

  loopcount++;

  if (loopcount == 10)
  {
      //smoothed averages
      avswr = avswr / 10.0;
      avgfwd = avgfwd / 10.0;
      avgRef = avgRef / 10.0;
      //convert to watts
      if (avgfwd > 5)
      {
          fwPwr = coeffs[2] * avgfwd * avgfwd + coeffs[1] * avgfwd + coeffs[0];
      }
      else
          fwPwr = 0.0;

      if (avgRef > 5)
      {
          refPwr = coeffs[2] * avgRef * avgRef + coeffs[1] * avgRef + coeffs[0];
      }
      else
      {
          refPwr = 0.0;
      }
      dtostrf(fwPwr, 3, 0, temp);
      dtostrf(refPwr, 3, 0, temp1);
      sprintf(strBuf, "FWD %s REF %s", temp, temp1);

      lcd.setCursor(0, 0);
      lcd.print(strBuf);

      lcd.setCursor(0, 1); //second line
      dtostrf(avswr, 4, 1, temp);
      sprintf(strBuf, "SWR %s", temp);
      lcd.print(strBuf);
      loopcount = 0;
      avswr = 0.0;
      avgfwd = 0.0;
      avgRef = 0.0;
  }
}//loop()


void clear_command_buffer()
{
  control_port_buffer_index = 0;
  control_port_buffer[0] = 0;
}//clear_command_buffer

void checkSerial()
{
  int incomingByte = 0;
  int av = Serial.available();

  if (av > 0)
  {
    incomingByte = Serial.read();
    last_serial_receive_time = millis();
    if ((incomingByte != '\n') && (incomingByte != '\r') && (incomingByte != ' ')) {
       control_port_buffer[control_port_buffer_index] = incomingByte;
       control_port_buffer_index++;
    }
    else{
#ifdef DEBUG
        Serial.println("Calling processCommand()");
#endif
        processCommand();
      clear_command_buffer();
    }
  }//if a byte is there
  else
  {
#ifdef DEBUG
//      Serial.println("no input");
#endif
  }
}//checkSerial

void processCommand()
{
  int cNow, lNow;

#ifdef DEBUG
  Serial.print("process command : ");
  Serial.println((char)control_port_buffer[0]);
#endif

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

    case 'a':
        cIn.state = 1;
        ToggleRelay(&cIn, 0);
        break;

    case 'A':
        cIn.state = 0;
        ToggleRelay(&cIn, 0);
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

    case 'R':
        setStates(L, 0);
        setStates(C, 0);
        break;

    case 'd':
    case 'D':
        printCurrentVals();
        break;

    case 'S':
        fwd = analogRead(FWDPIN);
        ref = analogRead(REFPIN);
        Serial.print ("Current fwd counts : ");
        Serial.println (fwd);
        Serial.print ("Current ref counts : ");
        Serial.println (ref);
        break;

    case 's':
        float swr = getSwr();
        Serial.print("Current swr : ");
        Serial.println(swr);
        break;

    case 't':
        tune();
        printCurrentVals();
        break;

    default:
        break;
  }//switch
}//processCommand

void tune()
{
    float swr, minswr;
    int lCoarse, cCoarse, lNow, cNow, dir;
    char strBuf[24];
    char temp[14];

    /*
    * First set cap to output side, reset tuner L,C settings
    * step through inductors 1 at a time to find best match
    * then step through caps 1 at a time
    * if no coarse match (swr < 3:1) set cap to input side and try again
    * once coarse match found refine till swr < 1.5 : 1
    */
    tuneInProgress = 1;
    minswr = 10;
    swr = getSwr();
    //if we have an adequate match exit
    if (swr <= 1.5) {
        tuneInProgress = 0;
        return;
    }
    //set initial conditions
    Serial.print("Tune - reset all");
    cIn.state = 0;
    ToggleRelay(&cIn, 0);
    setStates(L, 0);
    setStates(C, 0);

    //coarse L
    if (debugTune > 0) Serial.print("Tune - Coarse L set");
    for (int i = 0; i < 7; i++)
    {
        lNow = 1 << i;
        setStates(L, lNow);
        swr = getSwr();
        if (minswr > swr)
        {
            minswr = swr;
            lCoarse = lNow;
        }
        if (debugTune > 0)
        {
            Serial.print("Current inductance : ");
            Serial.print((float)lNow * 0.10);
            Serial.println(" uH");

            dtostrf(swr, 4, 1, temp);
            sprintf(strBuf, "SWR %s", temp);
            Serial.println(strBuf);
        }
    }
    setStates(L, lCoarse);  //set to best inductor
    //coarse C
    if (debugTune > 0) Serial.print("Tune - Coarse C set");
    for (int i = 0; i < 7; i++)
    {
        cNow = 1 << i;
        setStates(C, cNow);
        swr = getSwr();
        if (minswr > swr)
        {
            minswr = swr;
            cCoarse = cNow;
        }
        if (debugTune > 0)
        {
            Serial.print("Current capacitance : ");
            Serial.print(cNow * 10);
            Serial.println(" pF");

            dtostrf(swr, 4, 1, temp);
            sprintf(strBuf, "SWR %s", temp);
            Serial.println(strBuf);
        }
    }
    setStates(C, cCoarse);  //set to best capacitor

    if (minswr > 2.0)
    {
        if (debugTune > 0) Serial.print("Tune - change C to input side");
        //try again with C on input side
        cIn.state = 1;
        ToggleRelay(&cIn, 0);

        setStates(C, 0);
        //coarse C
        if (debugTune > 0) Serial.print("Tune - Coarse C set");
        for (int i = 0; i < 7; i++)
        {
            cNow = 1 << i;
            setStates(C, cNow);
            swr = getSwr();
            if (minswr > swr)
            {
                minswr = swr;
                cCoarse = cNow;
            }
        }
        setStates(C, cCoarse);  //set to best capacitor
        if (debugTune > 0)
        {
            Serial.print("Current capacitance : ");
            Serial.print(cNow * 10);
            Serial.println(" pF");

            dtostrf(swr, 4, 1, temp);
            sprintf(strBuf, "SWR %s", temp);
            Serial.println(strBuf);
        }
    }
    setStates(C, cCoarse);  
    //set to best capacitor
    //now refine tuning if necessary
    swr = getSwr();
    if (swr <= 1.5) 
    {
        if (debugTune > 0)
        {
            Serial.print("Current inductance : ");
            Serial.print((float)lNow * 0.10);
            Serial.println(" uH");

            Serial.print("Current capacitance : ");
            Serial.print(cNow * 10);
            Serial.println(" pF");

            dtostrf(swr, 4, 1, temp);
            sprintf(strBuf, "SWR %s", temp);
            Serial.println(strBuf);
            Serial.println("Tuning complete");
        }
        tuneInProgress = 0;
        return;
    }

    //first L
    if (minswr > swr)
    {
        minswr = swr;
    }
    //determine if increase or decrease improves swr
    if (debugTune > 0) Serial.print("Tune - Fine L set");
    dir = 1;
    lNow = lCoarse + dir;
    setStates(L, lNow);
    swr = getSwr();
    if (minswr < swr)
    {
        dir = -1;
        lNow = lCoarse - 2; //remove earlier increment and decreemnt instead
        setStates(L, lNow);
    }
    for (; minswr > swr; lNow += dir) //loop until swr stops improving
    {
        minswr = swr;
        setStates(L, lNow);
        swr = getSwr();
    }
    //backup inductor by 1 since we stepped past to see increase
    lNow += (dir > 0 ? -1 : 1);
    setStates(L, lNow);
    swr = getSwr();
    minswr = swr;

    if (debugTune > 0)
    {
        Serial.print("Current inductance : ");
        Serial.print((float)lNow * 0.10);
        Serial.println(" uH");

        dtostrf(swr, 4, 1, temp);
        sprintf(strBuf, "SWR %s", temp);
        Serial.println(strBuf);
    }

    //now refine C if necessary
    if (swr <= 1.5) {
        tuneInProgress = 0;
        Serial.println("Tuning complete");
        return;
    }

    if (debugTune > 0) Serial.print("Tune - Fine C set");
    dir = 1;
    cNow = cCoarse + dir;
    setStates(C, cNow);
    swr = getSwr();
    if (minswr < swr)
    {
        dir = -1;
        cNow = cCoarse - 2; //remove earlier increment and decreemnt instead
        setStates(C, cNow);
    }
    for (; minswr > swr; cNow += dir) //loop until swr stops improving
    {
        minswr = swr;
        setStates(C, cNow);
        swr = getSwr();
    }

    //backup inductor by 1 since we stepped past to see increase
    cNow += (dir > 0 ? -1 : 1);
    setStates(C, cNow);
    swr = getSwr();
    minswr = swr;
    if (debugTune > 0)
    {
        Serial.print("Current capacitance : ");
        Serial.print(cNow * 10);
        Serial.println(" pF");

        dtostrf(swr, 4, 1, temp);
        sprintf(strBuf, "SWR %s", temp);
        Serial.println(strBuf);
        Serial.println("Tuning complete");
    }
    tuneInProgress = 0;
}

void printCurrentVals()
{
    int cNow = getCurrentValue(C);
    int lNow = getCurrentValue(L);
    float swr = getSwr();

    Serial.print("Current capacitance : ");
    Serial.print(cNow * 10);
    Serial.println(" pF");

    Serial.print("Current inductance : ");
    Serial.print((float)lNow * 0.10);
    Serial.println(" uH");

    Serial.print("Current swr : ");
    Serial.println(swr);
}

float getSwr()
{
    fwd = analogRead(FWDPIN);
    ref = analogRead(REFPIN);
    if (fwd < 2)    //allow some slop in A/D
        return 1.0;
    float num = 1.0 + sqrt((float)ref / (float)fwd);
    float denom = 1.0 - sqrt((float)ref / (float)fwd);
    return num / denom;
}

byte getCurrentValue (relayState *stateArray)
{
  byte retval = 0;
  for (int i = 0; i < 7; i++){
    retval |= (stateArray[i].state & 0x01) << i;
  }//for
  
  return retval;
}//getCurrentValue

void setStates (relayState *stateArray, byte newValue)
{
  byte retval = 0;
  for (int i = 0; i < 7; i++){
    stateArray[i].state = ((newValue >> i)&0x01);
    ToggleRelay (&stateArray[i], 0);
  }//for
  
}//setStates

void ToggleRelay(relayState* CurrentState, int toggle = 1)
{
    if ((CurrentState->state > 0 && toggle > 0) || (CurrentState->state == 0 && toggle == 0)) //relay set - then reset
    {
        mcp.digitalWrite(relayBridge, LOW); //able to "RESET" relays to NC position 
        mcp.pinMode(CurrentState->relay, OUTPUT);
        delay(20);
        mcp.digitalWrite(CurrentState->relay, LOW); //reset relay
        delay(100); //give relay time to settle
        mcp.digitalWrite(CurrentState->relay, HIGH);
        delay(20);
        mcp.pinMode(CurrentState->relay, INPUT);
        CurrentState->state = 0;
#ifdef DEBUGRELAY
        Serial.print("ToggleRelay CurrentState->state = ");
        Serial.println(CurrentState->state);
#endif
    }
    else
    {
        mcp.digitalWrite(relayBridge, HIGH); //able to "SET" relays to NO position 
        mcp.pinMode(CurrentState->relay, OUTPUT);
        delay(20);
        mcp.digitalWrite(CurrentState->relay, HIGH); //set relay
        delay(100); //give relay time to settle
        mcp.digitalWrite(CurrentState->relay, LOW);
        delay(20);
        mcp.pinMode(CurrentState->relay, INPUT);
        CurrentState->state = 1;
#ifdef DEBUGRELAY
        Serial.print("ToggleRelay CurrentState->state = ");
        Serial.println(CurrentState->state);
#endif
    }

}//ToggleRelay

void initializeRelayStates()
{
    int relayMapping[] = {0,8,1,9,2,10,3,11,4,12,5,13,6,14};
    for (int i = 0; i < 14; i+=2)
    {
        //establish relay data line correspondance
        L[i].relay = relayMapping[i];
        C[i].relay = relayMapping[i+1];
    }

    //set everything to "on" so Toggle will reset to off
    setStates(L, 0);
    setStates(C, 0);
}

