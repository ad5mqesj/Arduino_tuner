#pragma once
#define COMMAND_BUFFER_SIZE 50
#define DEBUG 1
//#define DEBUGRELAY 1

typedef struct
{
	byte relay;
	byte state;
}relayState;

extern int relayBridge;
extern int LED;
extern int CapRelay;

extern int FWDPIN;
extern int REFPIN;


extern relayState L[7];
extern relayState C[7];
extern relayState cIn;

extern byte control_port_buffer[COMMAND_BUFFER_SIZE];
extern int control_port_buffer_index;
extern unsigned long last_serial_receive_time;
extern int fwd, ref;

void ToggleRelay(relayState* CurrentState, int toggle = 1);
void setStates(relayState* stateArray, byte newValue);
byte getCurrentValue(relayState* stateArray);
void processCommand();
void checkSerial();
void clear_command_buffer();
void initializeRelayStates();
float getSwr();
void printCurrentVals();
void tune();

