#ifndef CONFIG_h
#define CONFIG_h


// use this to define a lot of important variables that set up connection and such

#define DEVICE_LABEL "thermodenuder-esp32"
#define TOKEN "BBFF-JDOc0fGgNlYIxwMUujPKu48J8flkyf"

char const *  SSID_NAME = "engr_labs"; // put your wifi network name here
char const * SSID_PASS = "mikelokamo77"; // wifi password here

//char const * SSID_NAME = "Jamiesphone";
//char const * SSID_PASS = "CSUguestsucks";

// address of our two thermocouples
// for the MCP9600 to get an address of 0x60, connect the ADDR pin to GND

#define TC1_ADD (0x65)
#define TC2_ADD (0x67)

#endif
