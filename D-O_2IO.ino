/*

  D-O MK1 Head LSS 2IO command converter - V0.1

  written by Neil Hutchison

  Set the IDE to Arduino Nano, 328P


*/

#include <SoftwareSerial.h>
#include <Servo.h>

#define HEAD_TURN_ID 2
#define HEAD_TILT_ID 3

#define ANT_TL 4 // Top left antenna
#define ANT_TR 5 // Top right antenna
#define ANT_BT 6 // Bottom antenna
#define ANT_ALL 7 // All antenna


// the hardware pins of the LSS Software Serial port
const unsigned short hw_pin_lss_rx = 15;
const unsigned short hw_pin_lss_tx = 16;
const unsigned short hw_pin_lss_tx_enable = 14;
const unsigned short hw_pin_usb_tx_enable = 7;

// Setup the Software serial for the Dynamixel translator
// No need to use RX, so set it to a pin that's not used.
SoftwareSerial dynaSerial(10, 11); // RX, TX

// handle to the Serial object
Stream* serialPort;

Servo servTL; // Top Left Servo
Servo servTR; // Top Right Servo
Servo servBT; // Bottom Servo

// This should be off by default.  
//Enabling it will switch the USB on, and stop the normal commands from working!
//#define DEBUG


/*
 * Enable the transmission of data on the LSS Bus
 * 
 * The transmission of data is only enabled for the short duration we are actually transmitting, otherwise the
 * tri-state buffer is kept in a high impedance state so other bus devices can transmit.
 */
void lss_tx_enable(bool en)
{
  #ifndef DEBUG
    digitalWrite(hw_pin_lss_tx_enable, en ? HIGH : LOW);
  #endif
}

/*
 * Send a string or manual packet reply with a string value.
 * Required because LynxPacket for simplicity doesnt support string packet values.
 */
void lss_transmit(String s) {
  // The handler may update the packet for reply transmission
  // if so indicated, print the packet back to master
  //lss_tx_enable(true);
  Serial.print('*');
  Serial.print(s);
  Serial.print('\r');
  Serial.flush();
  //lss_tx_enable(false);
}

void setup() {

  pinMode (11, OUTPUT);

  #ifndef DEBUG
  // configure the TX enable tr-state buffer
  //pinMode (hw_pin_lss_tx_enable, OUTPUT);
  //lss_tx_enable(false);

  // disable USB tx input line so we only receive LSS bus serial data
  pinMode (hw_pin_usb_tx_enable, OUTPUT);
  digitalWrite(hw_pin_usb_tx_enable, LOW);
  #endif

  // Define pins A3-A5 as OUTPUT
  // These will be used for the servos.
  pinMode (A3, OUTPUT);
  pinMode (A4, OUTPUT);
  pinMode (A5, OUTPUT);

  Serial.begin(115200);
  serialPort=&Serial;

  // Start the dynamixel serial
  dynaSerial.begin(115200);

  // Attach the servos on the pins above:
  servTL.attach(A3);  // Top Left
  servTR.attach(A4);  // Top Right
  servBT.attach(A5);  // Bottom

  servTL.writeMicroseconds(1500); // Center the Servo
  servTR.writeMicroseconds(1500); // Center the Servo
  servBT.writeMicroseconds(1500); // Center the Servo

  // Servo range is 0 (clockwise full) to 1023 (counter clockwise)
  // 512 is centered.
  moveDyna(HEAD_TURN_ID, 512);
  moveDyna(HEAD_TILT_ID, 512);
  

/*
  // configure LED pins as outputs
  led_standard_output(LssLedOff);
  pinMode (hw_pin_led[0], OUTPUT);
  pinMode (hw_pin_led[1], OUTPUT);
  pinMode (hw_pin_led[2], OUTPUT);

  analogReference(DEFAULT);

  // hold a servo pin LOW to indicate wish for a factory reset
  test_factory_reset();

  // start with default config
  // copies config from program memory
  memcpy_P(&config, &default_config, sizeof(default_config));
  restore_config();   // will read from EEPROM, or write to it if it doesnt exist

  // configure the hardware serial port
  Serial.begin( baudrate_is_supported(config.io.baudrate)
      ? config.io.baudrate      // eeprom contained valid baudrate
      : 115200L                 // baudrate was invalid, use default
  );

  // restore LED color
  led_standard_output(config.led.color);
  */
  
}

bool flip = true;
unsigned long test_time;
uint16_t servo_pulse;
int dyna_pos;

unsigned char Checksum;


// Set the servo.  Valid values between 0 and 1023
void moveDyna(unsigned char ID, int Position)
{
    char Position_H,Position_L;
    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;

    const unsigned int length = 9;
    unsigned char packet[length];

    //Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H)) & 0xFF;
    Checksum = (~(ID + 5 + 3 + 30 + Position_L + Position_H)) & 0xFF;

    packet[0] = 255;
    packet[1] = 255;
    packet[2] = ID;
    packet[3] = 5;
    packet[4] = 3;
    packet[5] = 30;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Checksum;

    dynaSerial.write(packet, length);

}

// Set the servo ID.  Valid values between 0 and 253??
// We only need ID 2/3 in D-O
// Used as part of the setup code
void setDynaID(unsigned char ID, unsigned char newID)
{
    const unsigned int length = 8;
    unsigned char packet[length];

    //Checksum = (~(ID + AX_ID_LENGTH + AX_WRITE_DATA + AX_ID + newID)) & 0xFF;
    Checksum = (~(ID + 4 + 3 + 3 + newID)) & 0xFF;

    packet[0] = 255;
    packet[1] = 255;
    packet[2] = ID;
    packet[3] = 4;
    packet[4] = 3;
    packet[5] = 3;
    packet[6] = newID;
    packet[7] = Checksum;

    dynaSerial.write(packet, length);

}

// Mega simple test loop.
void loop() {

  // Don't actually do anything in the main loop.
  // The Serial command parser does actual work
  // When a valid command is received.
  
  // Test code can be added here if needed.

   
}

// Command processing stuff
// maximum number of characters in a command (63 chars since we need the null termination)
#define CMD_MAX_LENGTH 64 

// memory for command string processing
char cmdString[CMD_MAX_LENGTH];

/*
   SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEventRun(void)
{
  if (serialPort->available()) serialEvent();
}

void serialEvent() {

   #ifdef DEBUG
   Serial.println("Serial In");
   #endif
   bool command_available;

  while (serialPort->available()) {  
    char ch = (char)serialPort->read();  // get the new byte

    // New improved command handling
    command_available=buildCommand(ch, cmdString);  // build command line
    if (command_available) 
    {
      parseCommand(cmdString);  // interpret the command
    }
  }
  sei();
}


////////////////////////////////////////////////////////
// Command language - JawaLite emulation
///////////////////////////////////////////////////////


////////////////////////////////
// command line builder, makes a valid command line from the input
byte buildCommand(char ch, char* output_str)
{
  static uint8_t pos=0;
  switch(ch)
 {
    case '\r':                          // end character recognized
      output_str[pos]='\0';   // append the end of string character
      pos=0;        // reset buffer pointer
      return true;      // return and signal command ready
      break;
    default:        // regular character
      output_str[pos]=ch;   // append the  character to the command string
      if(pos<=CMD_MAX_LENGTH-1)pos++; // too many characters, discard them.
      break;
  }
  return false;
}

///////////////////////////////////
// command parser and switcher, 
// breaks command line in pieces, 
// rejects invalid ones, 
// switches to the right command
void parseCommand(char* inputStr)
{
  byte hasArgument=false;
  int argument;
  int address;
  int timing;
  byte pos=0;
  byte endArg=0;
  byte length=strlen(inputStr);
  if(length<2) goto beep;   // not enough characters

  #ifdef DEBUG
  Serial.print(" Here's the input string: ");
  Serial.println(inputStr);
  #endif

  
  // get the adress, one or two digits
  //char addrStr[2];
  char servo_addr[2];
  //if(!isdigit(inputStr[pos])) goto beep;  // invalid, first char not a digit
  //  addrStr[pos]=inputStr[pos];
  //  pos++;                            // pos=1
  //if(isdigit(inputStr[pos]))          // add second digit address if it's there
  //{  
  //  addrStr[pos]=inputStr[pos];
  //  pos++;                            // pos=2
  //}
  if(inputStr[pos] != ':') {
    // Good command start char.
    goto beep;
  }
  pos++;
  servo_addr[pos-1]=inputStr[pos];
  pos++;
  servo_addr[pos-1]='\0';                  // add null terminator
  
  address= atoi(servo_addr);        // extract the address

  //Serial.print(" I think this is the address! ");
  //Serial.println(address);
  
  // check for more
  if(length<=pos) goto beep;            // invalid, no command after address
  
  // other commands, get the numerical argument after the command character

  pos++;                             // need to increment in order to peek ahead of command char
  if(length<=pos) {hasArgument=false;}// end of string reached, no arguments
  else
  {
    for(byte i=pos; i<length; i++)
    {
      if(!isdigit(inputStr[i])) goto beep; // invalid, end of string contains non-numerial arguments
    } 
    argument=atoi(inputStr+pos);    // that's the numerical argument after the command character
    hasArgument=true;

    #ifdef DEBUG
    Serial.print(" I think this is the address! ");
    Serial.println(address);
    Serial.print(" I think this is the Command! ");
    Serial.println(inputStr[pos-1]);
    Serial.print(" I think this is the Command Value! ");
    Serial.println(argument);
    #endif
    
  }
  
  // switch on command character
  switch(inputStr[pos-1])               // 2nd or third char, should be the command char
  {
    case 'M':                           // A command does the same as D command, so just fall though.
      moveServo(address, argument);
      break;
    case 'S':
      // This is a setup command ... it will do config stuff
      setDynaID(address, argument);
    default:
      goto beep;                        // unknown command
      break;
  }
  
  return;                               // normal exit
  
  beep:                                 // error exit
    // Dont know what this does ... idnoring it for now!
    //serialPort->write(0x7);             // beep the terminal, if connected
    return;
}

void moveServo(int ID, int pos)
{
  // We need to move the right servo...
  if ((ID == HEAD_TURN_ID) || (ID == HEAD_TILT_ID)){
    // If the pos value is valid, we can move the servo.
    if (pos >=0 && pos <= 1023)
    {
      #ifdef DEBUG
      Serial.print("Move Dynamixel Servo ");
      Serial.print(ID);
      Serial.print(" to ");
      Serial.println(pos);
      #endif
      // Move a dynamixel servo
      moveDyna(ID, pos);
    }
    #ifdef DEBUG
    else
    {
      Serial.println("Invalid position received");
    }
    #endif
  }
  else if ((ID == ANT_TL) || (ID == ANT_TR) || (ID == ANT_BT) || (ID == ANT_ALL)){
    // Check for a valid position argument.
    if (pos >=1000 && pos <= 2000)
    {  
      switch(ID)
      {
        case ANT_TL:
          servTL.writeMicroseconds(pos);
          break;
        case ANT_TR:
          servTR.writeMicroseconds(pos);
          break;
        case ANT_BT:
          servBT.writeMicroseconds(pos);
          break;
        case ANT_ALL:
          //  All 3 do the same thing.  (This is a fast way to move all with one command.
          servTL.writeMicroseconds(pos);
          servTR.writeMicroseconds(pos);
          servBT.writeMicroseconds(pos);
          break;
      }
    }
    #ifdef DEBUG
    else
    {
      Serial.println("Invalid position received");
    }    
    #endif
  }
  
}
