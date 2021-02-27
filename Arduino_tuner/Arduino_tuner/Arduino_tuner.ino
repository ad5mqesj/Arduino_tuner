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

//map L/C to port ID for port expander
int LrelayMapping[] = { 0,8,1,9,2,10,3 };
int CrelayMapping[] = { 11,4,12,5,13,6,14 };

int CapRelay = 7;     //GPA7 expander
int relayBridge = 15; //GPB7 expander

int LED = 2;          //D2 pin main board
int tuneButton = 3;   //D3 pin 

int FWDPIN = A0;
int REFPIN = A1;

char control_port_buffer[COMMAND_BUFFER_SIZE];
int control_port_buffer_index = 0;
unsigned long last_serial_receive_time = 0;
float fwd, ref;
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
    lcd.setCursor(0, 0);
    lcd.print("AD5MQ");
    digitalWrite(LED, LOW); //LED on
    loopcount = 0;
}//setup()

float avswr = 0.0, fwPwr = 0.0, refPwr = 0.0;
float avgfwd = 0.0, avgRef = 0.0;
float  peakswr = 1.0;

void loop() {

  char strBuf[17];
  char temp[8], temp1[8];
  checkSerial();

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

  if (loopcount == 5)
  {
      //smoothed averages
      avswr = avswr / 5.0;
      avgfwd = avgfwd / 5.0;
      avgRef = avgRef / 5.0;
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
      if (avswr > 1.15)
          peakswr = avswr;
      dtostrf(peakswr, 4, 1, temp);
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
  memset(control_port_buffer, 0, sizeof(control_port_buffer));
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
       control_port_buffer[control_port_buffer_index++] = (char)incomingByte;
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

  switch (control_port_buffer[0]) {
   case 'C':
        if (control_port_buffer[1] == 0)
        {
            cNow = getCurrentValue(C);
            cNow++;
            if (cNow > 127)
                cNow = 0;
        }
        else
        {
            int shf = atoi(&(control_port_buffer[1]));
            cNow = 1 << shf-1;
        }
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
        if (control_port_buffer[1] == 0)
        {
            lNow = getCurrentValue(L);
            lNow++;
            if (lNow > 127)
                lNow = 0;
        }
        else
        {
            int shf = atoi(&(control_port_buffer[1]));
            lNow = 1 << shf - 1;
        }
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
    case 'D':
        printCurrentVals();
        break;

    case 'r':
    case 'R':
        if (DEBUG) Serial.println("reset all");
        cIn.state = 0;
        ToggleRelay(&cIn, 0);
        setStates(L, 0);
        setStates(C, 0);
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
    case 'T':
        Serial.println("Tune");
        tune();
        printCurrentVals();
        break;

    default:
        break;
  }//switch
}//processCommand

void tune()
{
    float swr, minswr, mincOutswr;;
    int lCoarse, cCoarse, lNow, cNow, dir, minCOut;
    char strBuf[24];
    char temp[14];
    peakswr = 1.0;
    /*
    * First set cap to output side, reset tuner L,C settings
    * step through inductors 1 at a time to find best match
    * then step through caps 1 at a time
    * if no coarse match (swr < 2:1) set cap to input side and try again
    * once coarse match found refine till swr < 1.5 : 1
    */
    tuneInProgress = 1;

    minswr = 10;
    swr = getSwr();
    //if we have an adequate match exit
    if (swr <= 1.5) {
        peakswr = swr;
        tuneInProgress = 0;
        printCurrentVals();
        return;
    }

    //set initial conditions
    if (debugTune > 0) Serial.println("Tune - reset all");
    cIn.state = 0;
    ToggleRelay(&cIn, 0);
    setStates(L, 0);
    setStates(C, 0);

    //coarse L
    if (debugTune > 0) Serial.println("Tune - Coarse L set");
    for (int i = 0; i <7; i++)
    {
        lNow = 1 << i;
        setStates(L, lNow);
        swr = getSwr();
        if (minswr > swr)
        {
            minswr = swr;
            lCoarse = lNow;
        }
        else if (swr > minswr + 1.0)    //exit if we are getting significantly worse
            break;

        if (debugTune > 0)
        {
            printVals(0, lNow, swr);
        }
    }
    setStates(L, lCoarse);  //set to best inductor

    if (debugTune > 0)
    {
        printVals(0, lCoarse, swr);
    }

    //coarse C
    if (debugTune > 0) Serial.println("Tune - Coarse C set");
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
        else if (swr > minswr + 1.0)    //exit if we are getting significantly worse
            break;
        if (debugTune > 0)
        {
            printVals(cNow, lCoarse, swr);
        }
    }
    setStates(C, cCoarse);  //set to best capacitor
    swr = getSwr();
    if (debugTune > 0)
    {
        printVals(cCoarse, lCoarse, swr);
    }

    mincOutswr = swr;
    minCOut = cCoarse;

    if (swr > 2.0)
    {
        if (debugTune > 0) Serial.println("Tune - change C to input side and try again");
        //try again with C on input side
        cIn.state = 1;
        ToggleRelay(&cIn, 0);
        setStates(C, 0);

         //coarse C
        if (debugTune > 0) Serial.println("Tune - Coarse C set");
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
            else if (swr > minswr + 1.0)    //exit if we are getting significantly worse
                break;
            if (debugTune > 0)
            {
                printVals(cNow, lCoarse, swr);
            }
        }
        setStates(C, cCoarse);  //set to best capacitor
        swr = getSwr();

        if (debugTune > 0)
        {
            printVals(cCoarse, lCoarse, swr);
        }
    }
    //if best swr is on c out side switch back
    if (mincOutswr < swr)
    {
        if (debugTune > 0) Serial.println("Best cIn worse than best cOut - reverting");
        ToggleRelay(&cIn);
        setStates(C, minCOut);  //set to best capacitor
        swr = getSwr();
        cCoarse = minCOut;
        printVals(cCoarse, lCoarse, swr);
    }

    //now refine tuning if necessary
    if (swr <= 1.5) 
    {
        if (debugTune > 0)
        {
            Serial.println("Tuneing complete");
        }
        tuneInProgress = 0;
        peakswr = swr;
        return;
    }

    //first L
    if (minswr > swr)
    {
        minswr = swr;
    }
    //determine if increase or decrease improves swr
    if (debugTune > 0) Serial.println("Tune - refine L set");
    dir = 1;
    lNow = lCoarse + dir*2;
    setStates(L, lNow);
    swr = getSwr();
    if (minswr < swr)
    {
        dir = -1;
        lNow = lCoarse - 3; //remove earlier increment and decreemnt instead
        setStates(L, lNow);
    }
    for (; minswr >= swr && (lNow < 32768 && lNow >=0); lNow += dir) //loop until swr stops improving
    {
        minswr = swr;
        setStates(L, lNow);
        swr = getSwr();
        if (debugTune > 0)
        {
            printVals(cCoarse, lNow, swr);
        }
    }

    //backup inductor by 1 since we stepped past to see increase
    lNow += (dir > 0 ? -1 : 1);
    setStates(L, lNow);
    swr = getSwr();
    minswr = swr;
    if (debugTune > 0)
    {
        Serial.println("fine adjust final inductance adjust");
        printVals(cCoarse, lNow, swr);
    }

    //now refine C if necessary
    if (swr <= 1.5) {
        tuneInProgress = 0;
        Serial.println("Tuning complete");
        return;
    }

    if (debugTune > 0) Serial.println("Tune - refine C set");
    dir = 1;
    cNow = cCoarse + dir*2;
    setStates(C, cNow);
    swr = getSwr();
    if (minswr < swr)
    {
        dir = -1;
        cNow = cCoarse - 3; //remove earlier increment and decreemnt instead
        setStates(C, cNow);
    }
    for (; minswr >= swr && (cNow < 32768 && cNow >= 0); cNow += dir) //loop until swr stops improving
    {
        minswr = swr;
        setStates(C, cNow);
        swr = getSwr();
        if (debugTune > 0)
        {
            printVals(cNow, lNow, swr);
        }
    }

    //backup inductor by 1 since we stepped past to see increase
    cNow += (dir > 0 ? -1 : 1);
    setStates(C, cNow);
    swr = getSwr();
    minswr = swr;
    if (debugTune > 0)
    {
        printVals(cNow, lNow, swr);
    }
    peakswr = swr;
    tuneInProgress = 0;
}

void printCurrentVals()
{
    int cNow = getCurrentValue(C);
    int lNow = getCurrentValue(L);
    float swr = getSwr();

    printVals(cNow, lNow, swr);
}

void printVals(int c, int l, float swr)
{
    Serial.print("Current capacitance : ");
    Serial.print(c * 10);
    Serial.println(" pF");

    Serial.print("Current inductance : ");
    Serial.print((float)l * 0.10);
    Serial.println(" uH");

    Serial.print("Current swr : ");
    Serial.println(swr);
}

float getSwr()
{
    int fwdavg = 0, revavg = 0;
    for (int i = 0; i < 5; i++)
    {
        fwdavg += analogRead(FWDPIN);
        revavg += analogRead(REFPIN);
        delay(2);
    }
    fwd = (float)fwdavg / 5.0;
    ref = (float)revavg / 5.0;

    if (fwd < 2)    //allow some slop in A/D
        return 1.0;
//    float num = 1.0 + sqrt((float)ref / (float)fwd);
//    float denom = 1.0 - sqrt((float)ref / (float)fwd);

    //detector output is in square law region - proportional to sqrt(power) rather than power
    float num = fwd + ref;
    float denom = fwd - ref;
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
        delay(2);
        mcp.digitalWrite(CurrentState->relay, LOW); //reset relay
        delay(5); //give relay time to settle
        mcp.digitalWrite(CurrentState->relay, HIGH);
        delay(2);
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
        delay(2);
        mcp.digitalWrite(CurrentState->relay, HIGH); //set relay
        delay(5); //give relay time to settle
        mcp.digitalWrite(CurrentState->relay, LOW);
        delay(2);
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
    for (int i = 0; i < 7; i++)
    {
        //establish relay data line correspondance
        L[i].relay = LrelayMapping[i];
        C[i].relay = CrelayMapping[i];
    }

    //set everything to "on" so Toggle will reset to off
    setStates(L, 0);
    setStates(C, 0);
}

