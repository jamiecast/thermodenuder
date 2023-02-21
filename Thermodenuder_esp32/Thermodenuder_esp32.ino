/********************************
  Notes
********************************/

// nothin in notes so far


#include <SPI.h>
#include <PID_v1.h>
#include "config.h"
#include "Adafruit_MCP9600.h"
#include "UbidotsEsp32Mqtt.h"
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>

// THERMODENUDER VARIABLES //
bool readBypass;
bool readHeated;

int bypassValvePin = 17;
int heatedValvePin = 18;

int relayPin = 19;
double pulseWidth = 0;
int t_relay = 1000; // ms full width of the pwm signal for the relay

// THERMOCOUPLE VARIABLES //
Adafruit_MCP9600 tc1, tc2;
double calculatedTemp = 0;

// TIMING VARIABLES //
// set dotMil really low to immediately check Ubidots when we power on
int const dotRate = 15 * 1000; // how often we check in with ubidots
int dotMil = -1 * dotRate; // the last time we've checked in with ubidots
float dotQueue = 0;

int valveMil = 0; // last time we've swapped valves
float bypassTime; // how long we stay on the bypass valve
float heatedTime; // how long we stay on the heated valve

// HEATING VARIRABLES //
float minTemp = 100; // our default min temp for a cycles
float maxTemp = 200; // same but for max temp
float tempInterval = 25; // the spacing between our temps for each round of the cycle
double targetTemp = 0;
bool readTemp = false;

float minHeat = 0.125; // max amount of % our relay can be on, keep between 0 and 1
float maxHeat = 0.925; // the max amount of % on for the relay, keep between 0 and 1

double Kp = 6, Ki = 0.8, Kd = 60;
float slowdownInterval = 7.5; // in degrees C this is where we start to reduce the output values

PID heatPID(&calculatedTemp, &pulseWidth, &targetTemp, Kp, Ki, Kd, DIRECT);

bool cycleRunning = false; // let us know if we're doing a cycle or not
bool heating = false; // if we need to activate the heating element or not

unsigned long relayStartTime = 0;

char const *SERVER = "things.ubidots.com";
//Replace the above line if you are an Educational user char const *SERVER="things.ubidots.com";
Ubidots ubidots(TOKEN);

bool attemptReconnect = true; // if connections keep failing, we just give up for a time before reattempting to connect to the wifi
int reconnectMilli;
int reconnectWait = 5 * 60 * 1000;

int LEDMilli;
float LED_pw = 0.5; // pulse width of our LED blinks
float t_LED = 2 * 1000;

int status = WL_IDLE_STATUS;

bool serverWait;


/********************************
   Auxiliar Functions
 *******************************/

void callback(char *topic, byte *payload, unsigned int length)
{
  String variable = "";
  String message = "";
  int slashcount = 0;
  for (int i = 0; i < strlen(topic); i++) {

    if (topic[i] == '/' ) {
      slashcount++;
    }
    if (slashcount == 4 && topic[i] != '/') {
      variable.concat(topic[i]);
    }
  }
  Serial.print(variable + ": ");
  for (int i = 0; i < length; i++)
  {
    // process data in here
    message += (char)payload[i];
  }

  // look I know if chains are bad but I spent an hour trying to figure out enums and maps but it was too much work
  // I don't care if that's not crazy robust, it still works and relatively fast too.
  if (variable == "queue-cycle") {
    dotQueue = message.toFloat();
    if (dotQueue) {
      dotMil = -1 * dotRate;
    }
  }
  if (dotQueue == 1) {
    if (variable == "bypass-time") {
      bypassTime = message.toFloat();
    }
    if (variable == "heated-time") {
      heatedTime = message.toFloat();
    }
    if (variable == "min-temp") {
      minTemp = message.toFloat();
      targetTemp = minTemp;
    }
    if (variable == "temp-int") {
      tempInterval = message.toFloat();
    }
    if (variable == "max-temp") {
      maxTemp = message.toFloat();
    }
  }
  Serial.println(message);
}


void checkCycle() {
  // gets the current state of the valves and sees if they need to change
  if (cycleRunning) {
    if (millis() >= valveMil + (bypassTime * 60 * 1000) && readBypass) { // use bypass time to control our bypass valve
      valveMil = millis(); // reset our timer
      // switch to reading from heated
      readBypass = false;
      readHeated = true;
    } else if (millis() >= valveMil + (heatedTime * 60 * 1000) && readHeated) { // same for heated
      valveMil = millis(); // reset our timer
      // our time for heated is over! let's switch to reading from bypass
      readBypass = true;
      readHeated = false;

      // let's also adjust our target temp when we switch back to bypass to make sure that's preheating for our next round
      if (targetTemp != maxTemp) {
        targetTemp += tempInterval;
        targetTemp = min((float)targetTemp, maxTemp); // if we went above our max temp based on interval size, shoot back down to max temp
      } else {
        // if our temp was actually set to our max, we're done with the cycle!
        // let's turn everything off for now
        heating = false;
        targetTemp = 0;
        cycleRunning = false;
      }
    }
  }
}

bool checkTime() {
  // checks to see how our dots timer is doing, then updates the timer if needed
  if (millis() >= dotMil + dotRate) {
    dotMil = millis();
    return true;
  } else {
    return false;
  }
}

void runRelay() {
  // We're gonna do a funny little thing here where we dynamically adjust the output settings of our PID
  // The heating element kept drastically overshooting the set temp, so we need to try and stop that before it happens

  // as we get higher and higher, we need to incrementally adjust our minheat term
  float calcMinHeat = minHeat + ((calculatedTemp / float(250)) * 0.4);

  // simply find the ratio between out error term and slowdownInterval
  float errorSlowdown = float((targetTemp - calculatedTemp)) / slowdownInterval;
  errorSlowdown = min(max(calcMinHeat, errorSlowdown), maxHeat); // constrain our value to a minimum and maximum so no matter what we have some heating, and we don't ever exceed our maximum heating
  heatPID.SetOutputLimits(0, t_relay * errorSlowdown);

  // run the PID
  heatPID.Compute();

  // Generates the PWM wave based on the pulseWidth
  if (millis() - relayStartTime > t_relay) {
    relayStartTime = millis();
  }
  if (pulseWidth > millis() - relayStartTime) {
    digitalWrite(relayPin, HIGH);
  } else {
    digitalWrite(relayPin, LOW);
  }

  // look, this is just a failsafe, but if we're trying to turn on the relay and we're above target what are we doing
  if (calculatedTemp > targetTemp) {
    digitalWrite(relayPin, LOW);
  }
}

void setupThermocouple(Adafruit_MCP9600 tc) {
  // runs through the setup to get any of our thermocouples ready

  tc.setThermocoupleType(MCP9600_TYPE_K); // change this depending on your thermocouple type
  tc.setADCresolution(MCP9600_ADCRESOLUTION_18);
  tc.setFilterCoefficient(3);
  tc.enable(true);

}

void connectThermocouples() {
  if (! tc1.begin(TC1_ADD)) {
    Serial.println("Error connecting to thermocouple 1");
    readTemp = false;
  } else {
    Serial.println("Thermocouple 1 successfully connected");
    setupThermocouple(tc1);
    readTemp = true;
  }
  if (! tc2.begin(TC2_ADD)) {
    Serial.println("Error connecting to thermocouple 2");
    readTemp = false;
  } else {
    Serial.println("Thermocouple 2 successfully connected");
    setupThermocouple(tc2);
  }
  if (readTemp) {
    Serial.println("Thermocouples are clean, reading temp.");
  } else {
    Serial.println("Unable to read thermocouples");
  }
}

void sendAllData() {
  // current temp
  sendData("temperature", calculatedTemp);
  // target temp
  sendData("target-temp", targetTemp);
  // valve states
  sendData("bypass-state", readBypass);
  sendData("heated-state", readHeated);
}

void sendData(char *label, float val) {
  ubidots.add(label, val);
  ubidots.publish(DEVICE_LABEL);
}

/********************************
   Main Functions
 *******************************/

void setup() {
  pinMode(relayPin, OUTPUT);
  //pinMode(LED_BUILTIN,OUTPUT);
  //digitalWrite(LED_BUILTIN, HIGH);

  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  Serial.print("Beginning setup");

  ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(SSID_NAME, SSID_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();

  // THERMODENUDER SETUP //
  readBypass = true; // read from bypass by default
  readHeated = false;

  bypassTime = 2;
  heatedTime = 2;

  heatPID.SetOutputLimits(0, t_relay * maxHeat);
  heatPID.SetMode(AUTOMATIC);

  pinMode(bypassValvePin, OUTPUT);
  pinMode(heatedValvePin, OUTPUT);

  // setup our thermocouples using the class we made
  connectThermocouples();


  Serial.println("Setup complete.");
}

void loop() {
  ubidots.loop();

  // calculate our temps
  if (readTemp) {
    float temp1 = tc1.readThermocouple();
    float temp2 = tc2.readThermocouple();
    if (isnan(temp1)) {
      Serial.println("Trouble with thermocouple 1! Attempting to restart\r\n");
      tc1.enable(false);
      setupThermocouple(tc1);
    }
    if (isnan(temp2)) {
      Serial.println("Trouble with thermocouple 2! Attempting to restart\r\n");
      tc2.enable(false);
      setupThermocouple(tc2);
    }
    calculatedTemp = (temp1 + temp2) / 2;
  }

  if (!serverWait && attemptReconnect) {
    // we want things on a timer so that we can control how often we send a recieve data from ubidots, independent of other things
    // the timing is controlled from a global variable at the top of this script
    // if we've given up on reconnecting to the server, we can just ignore all of this stuff and run on our own
    if (checkTime()) {
      // see if we can't get our thermocouples working
      if (!readTemp) {
        connectThermocouples();
      }

      // sending
      sendAllData();

      // make sure our relay is off
      digitalWrite(relayPin, LOW);
      //digitalWrite(LED_BUILTIN,HIGH);
      // recieving
      ubidots.subscribeLastValue(DEVICE_LABEL, "queue-cycle");

      if (dotQueue == 1) { // a cycle has been queued from Ubidots! let's pull all the other info about this cycle and then start running it
        // first let's pull all our info
        Serial.println("Cycle queued! Pulling extra info from ubidots");

        ubidots.subscribeLastValue(DEVICE_LABEL, "bypass-time");
        ubidots.subscribeLastValue(DEVICE_LABEL, "heated-time");
        ubidots.subscribeLastValue(DEVICE_LABEL, "min-temp");
        ubidots.subscribeLastValue(DEVICE_LABEL, "temp-int");
        ubidots.subscribeLastValue(DEVICE_LABEL, "max-temp");


        // okay! our data has been loaded, let's let ubidots know the cycle is started and get everything good to go

        sendData("queue-cycle", 0.0); // turn off the button
        ubidots.reconnect();
        Serial.println("Extra data successfully pulled! Initiating cycle");

        valveMil = millis(); // reset our timer
        readBypass = true; // start with bypass to give the heat tape time to get warm
        readHeated = false;
        heating = true;
        cycleRunning = true; // let the rest of our code see that the cycle is running

      }
    }
  }
  checkCycle(); // check in on the timing of our cycles
  // activate our valves
  digitalWrite(bypassValvePin, !readBypass);
  digitalWrite(heatedValvePin, !readHeated);


  if (heating && !serverWait) {
    runRelay();
  }
}
