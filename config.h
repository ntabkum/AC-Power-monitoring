
 
#define IO_USERNAME   "amvxroads" // adafruit.io username, pw = amvcrossroads / amvxroads@gmail.com
#define IO_KEY   "f3e00f7184a842b98a523bd6e2c6906e" // xroads adafruit io


/******************************* WIFI **************************************/


// #define WIFI_SSID     "AMVEPIC"
// #define WIFI_PASS     "amvepic3d"

// #define WIFI_SSID     "Crossroads"
// #define WIFI_PASS     "amvcrossroads"

// #define WIFI_SSID     "Sterling"
// #define WIFI_PASS     "amvsterling"

// #define WIFI_SSID     "Revolution"
// #define WIFI_PASS     "amvrevolution"

// #define WIFI_SSID     "AMVTRUCK"
// #define WIFI_PASS     "nyla1234"


// #define WIFI_SSID     "BATMAN"
// #define WIFI_PASS     "1a2b3c4d5e"

#define WIFI_SSID     ""
#define WIFI_PASS     ""
 

 #include "AdafruitIO_WiFi.h"
 AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
