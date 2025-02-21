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
#define GroundLED /*Enter pin*/               //LED indicating successful reception of communication on ground
#define AirLED /*Enter pin*/                  //LED indicating successful reception of communication in air
#define CutdownSignal /*Enter pin*/           //LED indicating whether cutdown has been triggered or not
#define Relay /*Enter pin*/                   //Relay controlling togglable current to nichrome wire cutdown mechanism
#define RX /*Enter pin*/                     //Receive pin for HC12 comms module
#define TX /*Enter pin*/                     //Transmit pin for HC12 comms module

//Instantiating objects corresponding to sensors
Adafruit_BMP280 /*BMP name*/;
Adafruit_MPU6050 /*MPU name*/;
SoftwareSerial HC12(TX, RX); // HC-12 TX Pin to Ar11, RX to Ar10

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//System variable definitions
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
const String satID = "/*Enter your ID here!*/";            //ID of the system, for the sake of distinguishing communication

/*You don't need to edit below, but feel free to read through and add commands if you would like to!*/

const int numCommands = 3;                //Number of valid commands
String commands[numCommands] = {          //Stores the list of all valid commands, each command requires a programmed response
  "CUTDOWN",                                  //Command to trigger cutdown mechanism, which detaches system from balloon, initiating descent
  "RESETCUT",                                 //Command to reset cutdown state variable, allows cutdown command to be called again
  "PING"
};

bool hasCutdown = false;                  //Flags whether the system has received the command to cutdown
String inCommand = "";                    //Temporarily stores the incoming command message
char inChar;                              //Momentarily stores the incoming character when reading HC12 data
int commandID = 0;                        //Temporarily stores the number position of the command
bool messageEnd = false;                  //Flags whether the end of the message has been reached

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//Essential functions
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

/**
   Returns the timestamp formatted as "T+[M]:[S]"
   @return  Formatted timestamp string
*/
String getTimestamp() {
  /* WRITE FUNCTION */
  unsigned long elapsedTime = millis() / 1000;
  int minutes = elapsedTime / 60;
  int seconds = elapsedTime % 60;

  return "T+" + string(minutes) + ":" + (seconds < 10 ? "0" : "") + string(seconds); 
}

/**
   Transmits message always formatted with this satellite's ID
   @param msg The message to be transmitted
*/
void sendMessage(String msg) {
  /* WRITE FUNCTION */
  String timestamp = getTimestamp();
  String fullMessage = timestamp + " " + msg;

  HC12.println(fullMessage);
  Serial.println("Sent: " + fullMessage);
}

/**
 * Seeks initial message from ground station, defines starting altitude, and sends first data values
*/
void calibrate() {
  for (int att = 1; att < 11; att++) {      //Attempts 10 times to search for command from ground station

    /* WRITE: Reads in message from HC12 character by character and stores the command in inCommand */
    
    if (inCommand.substring(0, 6) == "GROUND") {
      digitalWrite(GroundLED, HIGH);
      sendMessage("Test received! LED on!");
      break;                                //Breaks out of the for loop
    }

    /* WRITE: Reset the holding variables so you can search for the command in the next loop go-through */
    inCommand = "";
    delay(1000);


  }

  inCommand = "";
  inChar = NULL;

  //Flushes out first three readings, since BMP tends to start with faulty values
  for (int c = 1; c < 4; c++) {
    bmp.performReading();
  }

  sendMessage("Beginning sensor readings...");
  float temperature = bmp.temperature;
  float pressure = bmp.pressure;
  float altitude = bmp.altitude;

  sendMessage("Temperature: " + String(temperature) + " °C");
  sendMessage("Pressure: " + String(pressure) + " Pa");
  sendMessage("Altitude: " + String(altitude) + " m");

  /* WRITE: Send pressure, temperature, and altitude readings */
  /* WRITE: Send a clear message that indicates the beginning of transmission */
}


/**
   Reads command in HC12 data buffer and returns whether String received by module is a valid command.
   If it is not, the command variable is cleared and false is returned.
   @return  true  if the command is valid (found within commands list)
            false if the string is not a valid command
*/
bool receiveCommand() {
  while (Serial.available()) {
    char inChar = Serial.read();
    if (inChar == '\n'){
      break;
    }
    commandID += inChar;
  }
  /* WRITE: Reads in message from HC12 character by character and stores the command in inCommand */
  

  inChar = NULL;
  bool commandFound = true;

  while (commandID < numCommands) {
    if ( commandID == commands[commandID]) { /* WRITE: If the command matches the command in commands data structure... */ 
      break;
    }
    commandID++;      //Iterate search index
  }
  
  if (!commandFound) {  /* WRITE: If the command is not found in commands data structure */ 
    /* WRITE: Clear variable values */
    Serial.println("Command not found: " + inCommand);

    inCommand = "";
    commandID = 0;
    
    return false;
  }

  return true;
}


/**

*/
void assessPhase() {
  //Cuts down in case the balloon gets too high
  if (bmp.readAltitude(P0) > 15 + initialAlt) {
    resetCutdown();
    cutdown();
    resetCutdown();
    cutdown();
    flightMode = 0;
  }
}

void transmitData() {
  String data = "Hello from Group 1 Ground!!!";
  Serial.println(data);
}

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//Command functions
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

/**
   Triggers cutdown mechanism when valid. Invalid when cutdown flag is true and before descent phase of mission.
*/
void cutdown() {
  //Prevents cutdown if already triggered
  if (!hasCutdown){
    int relayTime = random(400, 1400);
    digitalWrite(CutdownSignal, HIGH);
    delay(relayTime);
    digitalWrite(CutdownSignal, LOW);
    digitalWrite(CutdownSignal, HIGH);
    hasCutdown = true;
    Serial.println("Cutdown triggered.");
  }
  /* WRITE: If the hasCutdown flag is true, do not execute anything */
  /* WRITE: Send a signal to the relay for a brief amount of time (400-1400 ms) */
  /* WRITE: Turn on the CutdownSignal LED */
  /* WRITE: Update the flag */
  /* WRITE: Send a clear message saying that cutdown was triggered */
}


/**
   Resets cutdown flag, unless the flag is already false
*/
void resetCutdown() {
  if (!hasCutdown){
    Serial.println("Cutdown flag already reset");
    return;
  }

  hasCutdown = false;
  digitalWrite(CutdownSignal, LOW);
  Serial.println("Cutdown has been reset.");
  /* WRITE: If the cutdown flag is already reset, send a message saying so*/
  /* WRITE: Reset the flag and indicator LED to their FALSE, LOW states */
  /* WRITE: Send a clear message saying that cutdown was reset */
}

/**
 * Responds with a "pong" and flashes the AirTest LED. Used to test two way communication between satellite and ground station
 */
void ping() {
  Serial.println("pong");

  digitalWrite(AirLED, HIGH);
  delay(200);
  digitalWrite(AirLED, LOW);
  delay(200);
  digitalWrite(AirLED, HIGH);
  delay(200);
  digitalWrite(AirLED, LOW);
  delay(200);
  digitalWrite(AirLED, HIGH);
  delay(200);
  digitalWrite(AirLED, LOW);
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
      cutdown();
      break;
    case 1:
      resetCutdown();
      break;
    case 2:
      ping();
      break;
  }
  commandID = 0;
  inCommand = "";
  inChar = "";
}

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//Operation (Arduino default setup and loop functions)
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

void setup() {

  //Begin the comms module with a baud rate of 9600 signals/sec
  HC12.begin(9600);

  //Defining pin types
  pinMode(GroundLED, OUTPUT);
  pinMode(AirLED, OUTPUT);
  pinMode(CutdownSignal, OUTPUT);
  pinMode(Relay, OUTPUT);

  //Setting initial values for pins, all LOW
  digitalWrite(GroundLED, LOW);
  digitalWrite(AirLED, LOW);
  digitalWrite(CutdownSignal, LOW);
  digitalWrite(Relay, LOW);

  //Assess if the MPU and BMP sensors are able to communicate with Arduino
  bool mpuFound = mpu.begin();
  bool bmpFound = bmp.begin_I2C();
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

void loop() {
  if (receiveCommand()) {
    respond(inCommand);
  }
  assessPhase();
  transmitData();
  delay(500);
}
