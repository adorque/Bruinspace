/**
 * Flight software for MiniSat.
 * Functionality:
 *      Transception with ground station
 *      Filtering command words
 *      Processing commands and executing instructions
 *      Collecting and transmitting data on pressure, temperature, and motion
 *      Change operability based on flight phase
 *      Trigger cutdown in emergency demooring
 *      
 * @author  Sahil Gosain (architecture and skeleton code)
 * Group members:
 * @author  Andy Huy Nguyen
 * @author  August Dalton
 * @author  Alyssa Leung
 * @author  Glen Troy San Diego
 * @author  Ryan Wong
 * 
 * @version 3.0 [UPDATE to 4.0 when you finish!]
 * @since   January 5, 2025
 */

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//Including all necessary libraries
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include "Adafruit_BMP280.h" //Use BMP3XX library if BMP3XX

//Defining all the pin connections to Arduino
// #define GroundLED                       //LED indicating successful reception of communication on ground
#define AirLED 06                           //LED indicating successful reception of communication in air
#define CutdownSignal 07                    //LED indicating whether cutdown has been triggered or not
// #define Relay/*Enter pin*/                   //Relay controlling togglable current to nichrome wire cutdown mechanism
#define RX 04                     //Receive pin for HC12 comms module
#define TX 03                   //Transmit pin for HC12 comms module

//Instantiating objects corresponding to sensors
Adafruit_BMP280 bmp; //BMP name
Adafruit_MPU6050 mpu /*MPU name*/;
SoftwareSerial HC12(TX, RX); // HC-12 TX Pin to Ar11, RX to Ar10

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//System variable definitions
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
const String satID = "GROUP 1 >:)";            //ID of the system, for the sake of distinguishing communication
const int numCommands = 4;                //Number of valid commands
int flightMode = 1;                       //Stores the phase of the mission, affects the data transmitted and the commands viable
//[0] = Emergency phase
//[1] = Ground Testing phase
//[2] = Ascent
//[3] = Cruise
//[4] = Descent & Landing
//[5] = Post-mission
//String diagnostics;                     //Stores information of all the actions performed by the system, for analysis
//String data;                            //Stores sensor data, for comparison to the data transmitted
bool hasCutdown = false;                  //Flags whether the system has received the command to cutdown
String commands[numCommands] = {          //Stores the list of all valid commands, each command requires a programmed response
  "DIAG",                                     //Command to transmit diagnostic information to ground station
  "CUTDOWN",                                  //Command to trigger cutdown mechanism, which detaches system from balloon, initiating descent
  "RESETCUT",                                 //Command to reset cutdown state variable, allows cutdown command to be called again
  "PING"
};

int cutdownTime;
int launchTime;
int initialAlt;
int cruiseAlt;

String inCommand = "";
char inChar;
int commandID = 0;
bool messageEnd = false;

const int P0 = (1013.25);            //in hPa (needed for readAltitude function)

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//Essential functions
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

/**
   Returns the timestamp formatted as "T+[M]:[S]"
   @return  Formatted timestamp string
*/
String getTimestamp() {
  int t = millis() / 1000;
  String stamp = "T+";
  stamp.concat(String(t / 60) + "m:");
  stamp.concat(String(t % 60) + "s\t");
  return stamp;
}


/**
   Transmits message always formatted with this satellite's ID
   @param msg The message to be transmitted
*/
void sendMessage(String msg) {
  HC12.print(satID + "::" + getTimestamp() + "--");
  while (msg != "") {
    HC12.print(msg.substring(0, 8));
    msg = msg.substring(8);
  }
  HC12.println();
}


/**
 * Seeks initial message from ground station, defines starting altitude, and sends first data values
*/
void calibrate() {
  for (int att = 1; att < 11; att++) {      //Attempts 10 times to search for command from ground station
    while (HC12.available()) {
      inChar = HC12.read();
      if (inChar == ":") {
        break;
      }
      inCommand += inChar;
    }
    if (inCommand.substring(0, 6) == "GROUND") {
      // digitalWrite(GroundLED, HIGH);
      sendMessage("Test received! LED on!");
      break;
    }
    inCommand = "";
    inChar = NULL;
    delay(1000);
  }

  inCommand = "";
  inChar = NULL;

  //Flushes out first three readings, since BMP tends to start with faulty value
  // for (int c = 1; c < 4; c++) {
  //   bmp.performReading();
  // }

  //Sets the initial altitude as a reference point for later comparisons
  initialAlt = bmp.readAltitude(P0);

  sendMessage("Temperature: " + String(bmp.readTemperature()));
  sendMessage("\tPressure: " +  String(bmp.readPressure()));
  sendMessage("\tInitial Alt: " + String(initialAlt));
  sendMessage("####### Flight Mode: Ground Test (1) #######");
}


/**
   Reads command in HC12 data buffer and returns whether String received by module is a valid command.
   If it is not, the command variable is cleared and false is returned.
   @return  true  if the command is valid (found within commands list)
            false if the string is not a valid command
*/
bool receiveCommand() {
  while (HC12.available()) {
    //inCommand = HC12.readStringUntil('\n');
    inChar = HC12.read();
    inCommand += char(inChar);
    //if (inChar == '\n') { messageEnd = true; }
  }
  inChar = NULL;
  while (commandID < numCommands) {
    if (inCommand == ("GROUND::" + commands[commandID])) {
      break;
    }
    commandID++;
  }
  if (commandID == numCommands) {
    inCommand = "";
    commandID = 0;
    return false;
  }

  return true;
}


/**

*/
void assessPhase() {
  switch (flightMode) {
    case 1:
      if ( bmp.readAltitude(P0) - 2.5 > initialAlt ) {        //If 2.5 m higher than initially
        flightMode++;
        sendMessage("####### Flight mode: Ascent (2) #######");
      }
      break;
    case 2:
      if ( (millis() - 1000) > launchTime && (bmp.readAltitude(P0) > initialAlt - 0.5 && bmp.readAltitude(P0) < initialAlt + 0.5)) { // makes a 1m range to see if the position is constant.
        flightMode++;
        sendMessage("####### Flight mode: Cruise (3) #######");
      }
      break;
    case 3:
      if ((millis() - 1000) > launchTime && (bmp.readAltitude(P0) + 0.5 < initialAlt)) { //Checks if the payload has started to descend
        flightMode++;
        sendMessage("####### Flight mode: Descent and Landing (4) #######");
      }
      break;
    case 4:
      if ((millis() - 1000) > launchTime && (bmp.readAltitude(P0) > initialAlt - 0.5  && bmp.readAltitude(P0) < initialAlt + 0.5)){
        flightMode++;
      } // same logic as phase 3, checking for active movement from max height to ground
      break;
    case 5:
      //phaseCheck = false;
      //any additional end functions that need to be run
      break;
    default:        //Invalid phase value
      //diagnostics += getTimestamp() + "ERR--\t Invalid phase value stored.\n";
      break;
  }

  //Cuts down in case the balloon gets too high
  if (bmp.readAltitude(P0) > 15 + initialAlt) {
    resetCutdown();
    cutdown();
    resetCutdown();
    cutdown();
    flightMode = 0;
  }
}

//Returns data from the accelerometer and bmp unit
void transmitData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // bmp.performReading();

  sendMessage("aX: " + String(a.acceleration.x)); //acceleration in the x-axis m/s^2
  sendMessage("aY: " + String(a.acceleration.y)); //acceleration in the y-axis m/s^2
  sendMessage("aZ: " + String(a.acceleration.z)); //acceleration in the z-axis m/s^2
  sendMessage("T: " + String(bmp.readTemperature())); //temperature (C)
  sendMessage("P: " + String(bmp.readPressure())); //pressure in Pa
  sendMessage("H: " + String(bmp.readAltitude(P0))); //Height in meters
}


//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//Command functions
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

/**
   Triggers cutdown mechanism when valid. Invalid when cutdown flag is true and before descent phase of mission.
*/
void cutdown() {
  //Prevents cutdown if already triggered
  if (hasCutdown) {
    //diagnostics += getTimestamp() + "RDD--\t Attempted to cutdown when previously triggered\n";
    return;
  }

  //Prevents cutdown command while in or after descent
  if (flightMode >= 4) {
    //diagnostics += getTimestamp() + "ERR--\t Attempted to cutdown when in descent\n";
    return;
  }

  cutdownTime = millis();
  String logCut = getTimestamp();
  // digitalWrite(Relay, HIGH);
  digitalWrite(CutdownSignal, HIGH);    //Turn on cutdown LED
  hasCutdown = true;                    //Indicate cutdown has been called for
  delay(600);
  // digitalWrite(Relay, LOW);
  sendMessage("++++++++++++Cutdown++++++++++++");
  //diagnostics += logCut + "CMD " + String(commandID) + " --\t Triggered cutdown\n";
}


/**
   Resets cutdown flag, unless the flag is already false
*/
void resetCutdown() {
  //Informs if command was called despite value being already default (false)
  if (!hasCutdown) {
    //diagnostics += getTimestamp() + "RDD--\t Attempted to reset cutdown variable when already default value\n";
    return;
  }
  hasCutdown = false;
  digitalWrite(CutdownSignal, LOW);
  sendMessage("++++++++++++Reset Cutdown++++++++++++");
  //diagnostics += getTimestamp() + "CMD " + String(commandID) + " --\t Reset cutdown value\n";
}

/**
 * Responds with a "pong." Used to test two way communication between satellite and ground station
 */
void ping() {
  sendMessage("pong");
  boolean state = (digitalRead(AirLED) == HIGH);
  digitalWrite(AirLED, state ? LOW : HIGH); 
}


//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//Command response function
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

/**
   Performs an action based on the command received, corresponding to the position of the command in the commands structure.
   @param cmd The command received by the system which prompts a response.
*/
void respond(String cmd) {
  switch (commandID) {
    case 0:
      //sendMessage(diagnostics);
      break;
    case 1:
      cutdown();
      break;
    case 2:
      resetCutdown();
      break;
    case 3:
      ping();
      break;
    case 4:
    case 5:
    default:    //Invalid command
      //diagnostics += getTimestamp() + "ERR--\t Invalid command \"" + cmd + "\" processed through receiveCommand function.\n";
      break;
  }
  commandID = 0;
  inCommand = "";
  inChar = "";
}


//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//Operation (Arduino default setup and loop functions)
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//Runs when the Arduino is booted up
void setup() {

  //Begin the comms module with a baud rate of 9600 signals/sec
  HC12.begin(9600);

  //Defining pin types
  // pinMode(GroundLED, OUTPUT);
  pinMode(AirLED, OUTPUT);
  pinMode(CutdownSignal, OUTPUT);
  // pinMode(Relay, OUTPUT);

  //Setting initial values for pins, all LOW
  // digitalWrite(GroundLED, LOW);
  digitalWrite(AirLED, LOW);
  digitalWrite(CutdownSignal, LOW);
  // digitalWrite(Relay, LOW);

  //Assess if the MPU and BMP sensors are able to communicate with Arduino
  bool mpuFound = mpu.begin();
  bool bmpFound = bmp.begin();
  if (!mpuFound || !bmpFound) {
    if (!mpuFound) {
      sendMessage("MPU not found, check wiring");
    }
    if (!bmpFound) {
      sendMessage("BMP not found, check wiring");
    }
  }

  calibrate();

  delay(1000);

  sendMessage("Begin");
}

//will continuously run as long as the arduino is still powered and recieving data
void loop() {
  if (receiveCommand()) {
    respond(inCommand);
  }
  assessPhase();
  transmitData();
  delay(500);
  
  //Uncomment to test transmit and receive
  //sendMessage("hi");
  //String message = HC12.readString();
  //sendMessage(message);
}
