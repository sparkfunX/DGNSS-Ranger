#include <WiFi.h>
#include "secrets.h"

#include <Wire.h> // Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> // http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS GNSS_Forward;
SFE_UBLOX_GNSS GNSS_Rear;

#include <SparkFun_Qwiic_OLED.h> // http://librarymanager/All#SparkFun_Qwiic_OLED
QwiicMicroOLED myOLED;

// The ESP32 core has a built in base64 library but not every platform does
// We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" // Built-in ESP32 library
#else
#include <Base64.h> // nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

#define triggerButton 13

long lastReceivedRTCM_ms = 0; // 5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 20000; // If we fail to get a complete RTCM frame after 10s, then disconnect from caster
double RELPOSN = 0;
double RELPOSE = 0;
double RELPOSD = 0; 
double LAT = 0;
double LON = 0;
double ALT = 0;

const double degreesToRadians = PI/180; 
const double radiansToDegrees = 180/PI; 
const double WGS84_A = 6378137;           // https://geographiclib.sourceforge.io/html/Constants_8hpp_source.html
const double WGS84_E = 0.081819190842622; // http://docs.ros.org/en/hydro/api/gps_common/html/namespacegps__common.html
                                          // and https://gist.github.com/uhho/63750c4b54c7f90f37f958cc8af0c718

// The following come from https://gist.github.com/govert/1b373696c9a27ff4c72a                                          
const double WGS84_B = 6356752.314245;                  // Derived Earth semiminor axis (m)
const double WGS84_F = (WGS84_A - WGS84_B) / WGS84_A;   // Ellipsoid Flatness
const double WGS84_F_inv = 1.0 / WGS84_F;               // Inverse flattening

const double WGS84_A_sq = WGS84_A * WGS84_A;
const double WGS84_B_sq = WGS84_B * WGS84_B;
const double WGS84_E_sq = WGS84_F * (2 - WGS84_F);      // Square of Eccentricity


void setup() 
{

  Serial.begin(115200); // Start UART for debug/config

  // 5 seconds on startup to enter the ublox module config
  while (Serial.available()) Serial.read(); // Trash any incoming chars
  Serial.println(F("Press Any Key to run u-blox config"));
  for (int d = 0; d < 5; d++)
  {
    if(Serial.available()) ubloxConfig();
    delay(1000);
  }

  Serial.println(F("u-blox config skipped"));

  // Start I2C
  Wire.begin();
  delay(50);
  Wire.setClock(400000);

  // Initalize the OLED device and related graphics system
  if (myOLED.begin() == false)
  {
      Serial.println("OLED begin failed. Freezing...");
      while(1);
  }

  printOLED(String("u-blox wait"));

  // Connect to rear (moving base) u-blox module
  if (GNSS_Rear.begin(Wire, 0x3F) == false)
  {
    printOLED(String("u-blox error"));
    Serial.println(F("Rear GNSS module not detected at 0x3F. Please check wiring and configuration. Freezing."));
    while(1);
  }

  // Connect to forward (rover) u-blox module
  if (GNSS_Forward.begin() == false)
  {
    printOLED(String("u-blox error"));
    Serial.println(F("Forward GNSS module not detected at 0x42. Please check wiring and configuration. Freezing."));
    while(1);
  }  

  GNSS_Rear.setI2COutput(COM_TYPE_UBX); //Turn off NMEA noise
  GNSS_Rear.setI2CInput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.

  GNSS_Forward.setAutoRELPOSNEDcallbackPtr(&updateRELPOSNED); // Enable automatic NAV RELPOSNED messages with callback to updateRELPOSNED
  
  printOLED("u-blox OK");

  delay(1000);

  // Start WiFi for NTRIP 
  printOLED("wait for WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  printOLED("WiFi OK");
  delay(3000);
  //printOLED(String(WiFi.localIP()));  

  // Set pinMode for trigger input
  pinMode(triggerButton, INPUT_PULLUP);  

  // Trash any incoming chars
  while (Serial.available()) Serial.read();

}

//------------------------------------------
// Main Loop
// This is just a place to wait for user input 
// and a place to return when NTRIP times out
//------------------------------------------
void loop() 
{
    
  printOLED("Start?");
  while(digitalRead(triggerButton) == 1){};
  beginClient();

}

//------------------------------------------
// beginClient()
// Connect to NTRIP Caster, receive RTCM, 
// and push to rear ublox module over I2C.
// Check forward ublox module for updated
// relative positioning information.
//------------------------------------------
void beginClient()
{
  WiFiClient ntripClient;
  long rtcmCount = 0;

  Serial.println(F("Subscribing to Caster. Press key to stop"));
  delay(10); //Wait for any serial to arrive
  while (Serial.available()) Serial.read(); //Flush

  while (Serial.available() == 0)
  {
    // Connect if we are not already. Limit to 5s between attempts.
    if (ntripClient.connected() == false)
    {
      Serial.print(F("Opening socket to "));
      Serial.println(casterHost);

      if (ntripClient.connect(casterHost, casterPort) == false) // Attempt connection
      {
        Serial.println(F("Connection to caster failed"));
        return;
      }
      else
      {
        Serial.print(F("Connected to "));
        Serial.print(casterHost);
        Serial.print(F(": "));
        Serial.println(casterPort);

        Serial.print(F("Requesting NTRIP Data from mount point "));
        Serial.println(mountPoint);

        const int SERVER_BUFFER_SIZE  = 512;
        char serverRequest[SERVER_BUFFER_SIZE];

        snprintf(serverRequest, SERVER_BUFFER_SIZE, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
                 mountPoint);

        char credentials[512];
        if (strlen(casterUser) == 0)
        {
          strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
        }
        else
        {
          //Pass base64 encoded user:pw
          char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; //The ':' takes up a spot
          snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);

          Serial.print(F("Sending credentials: "));
          Serial.println(userCredentials);

#if defined(ARDUINO_ARCH_ESP32)
          //Encode with ESP32 built-in library
          base64 b;
          String strEncodedCredentials = b.encode(userCredentials);
          char encodedCredentials[strEncodedCredentials.length() + 1];
          strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
          snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
#else
          //Encode with nfriendly library
          int encodedLen = base64_enc_len(strlen(userCredentials));
          char encodedCredentials[encodedLen]; //Create array large enough to house encoded data
          base64_encode(encodedCredentials, userCredentials, strlen(userCredentials)); //Note: Input array is consumed
#endif
        }
        strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
        strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

        Serial.print(F("serverRequest size: "));
        Serial.print(strlen(serverRequest));
        Serial.print(F(" of "));
        Serial.print(sizeof(serverRequest));
        Serial.println(F(" bytes available"));

        Serial.println(F("Sending server request:"));
        Serial.println(serverRequest);
        ntripClient.write(serverRequest, strlen(serverRequest));

        //Wait for response
        unsigned long timeout = millis();
        while (ntripClient.available() == 0)
        {
          if (millis() - timeout > 5000)
          {
            Serial.println(F("Caster timed out!"));
            ntripClient.stop();
            return;
          }
          delay(10);
        }

        // Check reply
        bool connectionSuccess = false;
        char response[512];
        int responseSpot = 0;
        while (ntripClient.available())
        {
          if (responseSpot == sizeof(response) - 1) break;

          response[responseSpot++] = ntripClient.read();
          if (strstr(response, "200") > 0) // Look for 'ICY 200 OK'
            connectionSuccess = true;
          if (strstr(response, "401") > 0) // Look for '401 Unauthorized'
          {
            Serial.println(F("Hey - your credentials look bad! Check you caster username and password."));
            connectionSuccess = false;
          }
        }
        response[responseSpot] = '\0';

        Serial.print(F("Caster responded with: "));
        Serial.println(response);

        if (connectionSuccess == false)
        {
          Serial.print(F("Failed to connect to "));
          Serial.print(casterHost);
          Serial.print(F(": "));
          Serial.println(response);
          return;
        }
        else
        {
          Serial.print(F("Connected to "));
          Serial.println(casterHost);
          lastReceivedRTCM_ms = millis(); // Reset timeout
        }
      } //End attempt to connect
    } //End connected == false

    if (ntripClient.connected() == true)
    {
      // Check if user is pressing the trigger
      if(digitalRead(triggerButton) == 0)
      { 
        delay(500);
        makeMeasurement();
      }
      
      uint8_t rtcmData[512 * 4]; // Most incoming data is around 500 bytes but may be larger
      rtcmCount = 0;

      //Print any available RTCM data
      while (ntripClient.available())
      {
        //Serial.write(ntripClient.read()); // Pipe to serial port is fine but beware, it's a lot of binary data
        rtcmData[rtcmCount++] = ntripClient.read();
        if (rtcmCount == sizeof(rtcmData)) break;
      }

      if (rtcmCount > 0)
      {
        lastReceivedRTCM_ms = millis();

        // Push RTCM to rear ublox module over I2C
        GNSS_Rear.pushRawData(rtcmData, rtcmCount, false);
        Serial.print(F("RTCM pushed to ZED: "));
        Serial.println(rtcmCount);

        // Check for new RELPOSNED data
        GNSS_Forward.checkUblox(); 
        GNSS_Forward.checkCallbacks();

        // Check for new HPPOS data
        if (GNSS_Rear.getHPPOSLLH())
        {
          checkHPPos();
        }
      }
    }

    // Close socket if we don't have new data for 10s
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
    {
      Serial.println(F("RTCM timeout. Disconnecting..."));
      if (ntripClient.connected() == true)
        ntripClient.stop();
      return;
    }

    delay(10);
  }

  Serial.println(F("User pressed a key"));
  Serial.println(F("Disconnecting..."));
  ntripClient.stop();
}

//------------------------------------------
// ubloxConfig()
// Reset ublox modules to factory settings and 
// then load the correct configuration for 
// moving base operation.
// Because the ublox modules will have the same 
// I2C address by default, we require the technician
// to unplug everything after the first module 
// for part of this configuration routine
//------------------------------------------
void ubloxConfig()
{
  while (Serial.available()) Serial.read(); // Trash any incoming chars
  Serial.println(F("Connect ONLY the rear u-blox module to the Qwiic Bus and press any key"));
  while (Serial.available() == false) ; // Wait for user to send character

  Wire.begin();
  Wire.setClock(400000); // Increase I2C clock speed to 400kHz
  delay(50);

  if (GNSS_Rear.begin() == false) // Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Checking second address."));
    if (GNSS_Rear.begin(Wire, 0x3F) == false) // Connect to the u-blox module using Wire port
    {
      Serial.println(F("u-blox GNSS not detected at either expected address. Please check wiring. Freezing."));
      while(1);
    }      
  }  

  Serial.println(F("Restoring rear module to clean configuration (factory reset) please wait..."));

  GNSS_Rear.factoryDefault(); // Reset everything: baud rate, I2C address, update rate, everything.

  delay(5000); // Wait while the module restarts

  while (GNSS_Rear.begin() == false) //Attempt to re-connect
  {
    delay(1000);
    Serial.print(F("Attempting to re-connect to rear module after reset... "));
  }

  Serial.println(F(" Success."));
  Serial.print(F("Attempting to change rear module address"));

  GNSS_Rear.setI2CAddress(0x3F, VAL_LAYER_RAM_BBR);

  delay(2000); // Allow time for the change to take

  GNSS_Rear.end();

  while (GNSS_Rear.begin(Wire, 0x3F) == false) // Attempt to re-connect
  {
    delay(1000);
    Serial.print(F("Attempting to re-connect to rear module at new address... "));
  }  

  Serial.println(F(" Success."));
  Serial.println(F("Configuring rear module for Moving Base Operation. Please wait..."));

  bool setValueSuccess = true;

  //------------------------------------------
  // MOVING BASE CONFIGURATION
  //------------------------------------------

  //Begin with newCfgValset
  setValueSuccess = GNSS_Rear.newCfgValset(); // Defaults to configuring the setting in RAM and BBR

  // Add KeyIDs and Values
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_UART2INPROT_NMEA, 0); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_UART2INPROT_RTCM3X, 0); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_UART2INPROT_UBX, 0); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_UART2OUTPROT_NMEA, 0); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_UART2OUTPROT_RTCM3X, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_UART2OUTPROT_UBX, 0); 

  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_I2CINPROT_RTCM3X, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_I2COUTPROT_UBX, 1); 
  
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART2, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART2, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART2, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART2, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART2, 1); 

  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_0_USB, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_USB, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_USB, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_USB, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_USB, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_USB, 1); 

  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_UBX_NAV_PVT_USB, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_MSGOUT_UBX_NAV_SVIN_USB, 1); 
  setValueSuccess &= GNSS_Rear.addCfgValset(UBLOX_CFG_UART2_BAUDRATE, 460800); 

  // Send the packet using sendCfgValset
  setValueSuccess &= GNSS_Rear.sendCfgValset();

  if (setValueSuccess == true)
  {
    Serial.println("Moving Base Config successfully set");
  }
  else
  {
    Serial.println("Moving Base Config set failed. freezing");  
    while(1);
  }

  Serial.println(F("Ending connection with rear module"));
  GNSS_Rear.end();
  Wire.end();  

  while (Serial.available()) Serial.read(); // Trash any incoming chars
  Serial.println(F("Connect the forward u-blox module and the remainder of the Qwiic Bus and press any key"));
  while (Serial.available() == false) ; // Wait for user to send character

  Wire.begin();
  Wire.setClock(400000); // Increase I2C clock speed to 400kHz
  delay(50);

  if (GNSS_Forward.begin() == false) // Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }  

  Serial.println(F("Restoring forward module to clean configuration (factory reset) please wait..."));

  GNSS_Forward.factoryDefault(); // Reset everything: baud rate, I2C address, update rate, everything.

  delay(5000); // Wait while the module restarts

  while (GNSS_Forward.begin() == false) // Attempt to re-connect
  {
    delay(1000);
    Serial.print(F("Attempting to re-connect to forward module after reset... "));
  }

  Serial.print(F("Success."));
  Serial.println(F("Configuring forward module for Rover Operation. Please wait..."));

  setValueSuccess = true;

  //------------------------------------------
  // ROVER CONFIGURATION
  //------------------------------------------

  // Begin with newCfgValset
  setValueSuccess = GNSS_Forward.newCfgValset(); // Defaults to configuring the setting in RAM and BBR

  // Add KeyIDs and Values
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_UART2INPROT_NMEA, 0); 
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_UART2INPROT_RTCM3X, 1); 
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_UART2INPROT_UBX, 0); 
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_UART2OUTPROT_NMEA, 0); 
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_UART2OUTPROT_RTCM3X, 0); 
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_UART2OUTPROT_UBX, 0); 

  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_I2C, 1); 
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_USB, 0);   
  
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_USBINPROT_NMEA, 1); 
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_USBINPROT_RTCM3X, 1); 
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_USBINPROT_UBX, 1); 
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_USBOUTPROT_NMEA, 1); 
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_USBOUTPROT_RTCM3X, 0); 
  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_USBOUTPROT_UBX, 1);   

  setValueSuccess &= GNSS_Forward.addCfgValset(UBLOX_CFG_UART2_BAUDRATE, 460800); 

  // Send the packet using sendCfgValset
  setValueSuccess &= GNSS_Forward.sendCfgValset();

  if (setValueSuccess == true)
  {
    Serial.println("Rover Config successfully set");
  }
  else
  {
    Serial.println("Rover Config set failed. freezing");  
    while(1);
  } 

  Serial.println(F("Ending connection with forward module"));
  GNSS_Forward.end();
  Wire.end();  

  Serial.println(F("Configuration successful"));

  return;
}

//------------------------------------------
// printOLED(String)
// Just a simple helper function that takes
// a String and prints it to the center of
// the OLED module
//------------------------------------------
void printOLED(String outString)
{
  myOLED.erase();

  // starting x position - screen width minus string width  / 2
  int x0 = (myOLED.getWidth() - myOLED.getStringWidth(outString)) / 2;

  // starting y position - screen height minus string height / 2 
  int y0 = (myOLED.getHeight() - myOLED.getStringHeight(outString)) / 2;

  // Draw the text - color of black (0)
  myOLED.text(x0, y0, outString, 4);

  // There's nothing on the screen yet - Now send the graphics to the device
  myOLED.display();
}

//------------------------------------------
// updateRELPOSNED(UBX_NAV_RELPOSNED_data_t)
// Callback to retreive and populate the 
// relative position data from the forward 
// ublox module
//------------------------------------------
void updateRELPOSNED(UBX_NAV_RELPOSNED_data_t *ubxDataStruct)
{
  Serial.println("Fresh RELPOS");
  RELPOSN = ((double)ubxDataStruct->relPosN / 100) + ((double)ubxDataStruct->relPosHPN / 10000);
  RELPOSE = ((double)ubxDataStruct->relPosE / 100) + ((double)ubxDataStruct->relPosHPE / 10000);
  RELPOSD = ((double)ubxDataStruct->relPosD / 100) + ((double)ubxDataStruct->relPosHPD / 10000);
}

//------------------------------------------
// makeMeasurement()
// Get the relative position data, use trig
// to augment it by the laser rangefinder 
// distance and translate to ECEF
//------------------------------------------
void makeMeasurement()
{
  // LIDAR measurement
  double lidarMeas = 40.00;

  Serial.print("RELPOSN: ");
  Serial.println(RELPOSN);
  Serial.print("RELPOSE: ");
  Serial.println(RELPOSE);
  Serial.print("RELPOSD: ");
  Serial.println(RELPOSD);
  
  
  // we'll lose our signness when we square these numbers later 
  // so we need to remember and re-apply them afterward
  bool Npositive = RELPOSN > 0;
  bool Epositive = RELPOSE > 0;
  bool Dpositive = RELPOSD > 0;

  // set up some doubles for our target RELPOS
  double targetRELPOSN;
  double targetRELPOSE;
  double targetRELPOSD;

  // doubles for debugging target LLA
  double targetLAT;
  double targetLON;
  double targetALT;

  // set up some doubles for our target ECEF
  double targetx;
  double targety;
  double targetz;

  // set up some doubles for our origin ECEF for testing
  double originx;
  double originy;
  double originz;

  // we need to solve two right triangles (effectively heading and altitude)

  // get the hypotenuse of our NE triangle
  double NEhyp = hypot(RELPOSN, RELPOSE);
  // get the hypotenuse/N angle of NE triangle
  double HNang = acos(abs(RELPOSN)/NEhyp);
  // get hypotenuse of altitude triangle made up of NE hypotenuse and D (this is inter-antenna distance)  
  double ALThyp = hypot(NEhyp, RELPOSD);
  // get hypotenuse/D angle of altitude triangle
  double HDang = acos(abs(RELPOSD)/ALThyp);
  // add LIDAR measurement to new hypotenuse and solve for new D and NE hypotenuse using hypotenuse/D angle and rules of sines
  double InterANTDistance = ALThyp;
  ALThyp += lidarMeas;
  targetRELPOSD = ALThyp*cos(HDang);
  NEhyp = ALThyp*sin(HDang);
  // solve for new N and E using hypotenuse/N angle and rule of sines
  targetRELPOSN = NEhyp*cos(HNang);
  targetRELPOSE = NEhyp*sin(HNang);
  // re-sign new NED
  if (!Npositive) targetRELPOSN = targetRELPOSN*(-1);
  if (!Epositive) targetRELPOSE = targetRELPOSE*(-1);
  if (!Dpositive) targetRELPOSD = targetRELPOSD*(-1);   
  // convert new NED to ECEF
  enuToEcef(targetRELPOSE, targetRELPOSN, targetRELPOSD*(-1), LAT, LON, ALT, &targetx, &targety, &targetz);
  // get target LLA for testing
  ecefToGeodetic(targetx, targety, targetz, &targetLAT, &targetLON, &targetALT);
  // get reference origin ecef for testing
  geodeticToEcef(LAT, LON, ALT, &originx, &originy, &originz);
  // print some stuff for testing
  Serial.print("ANT-ANT Distance: ");
  Serial.println(InterANTDistance);
  Serial.println("Origin LLA: ");
  Serial.print(LAT, 8); Serial.print(",");
  Serial.print(LON, 8); Serial.print(",");
  Serial.println(ALT, 8);
  Serial.println("Target LLA: ");
  Serial.print(targetLAT, 8); Serial.print(",");
  Serial.print(targetLON, 8); Serial.print(",");
  Serial.println(targetALT, 8);
  Serial.println("Origin ECEF: ");
  Serial.println(originx);
  Serial.println(originy);
  Serial.println(originz);
  Serial.println("Target ECEF: ");
  Serial.println(targetx);
  Serial.println(targety);
  Serial.println(targetz);  
  Serial.println();
}

// From: https://stackoverflow.com/questions/19478200/convert-latitude-and-longitude-to-ecef-coordinates-system
void geodeticToEcef(double lat, double lon, double alt, double *x, double *y, double *z)
{
    double clat = cos(lat * degreesToRadians);
    double slat = sin(lat * degreesToRadians);
    double clon = cos(lon * degreesToRadians);
    double slon = sin(lon * degreesToRadians);

    double N = WGS84_A / sqrt(1.0 - WGS84_E * WGS84_E * slat * slat);

    *x = (N + alt) * clat * clon;
    *y = (N + alt) * clat * slon;
    *z = (N * (1.0 - WGS84_E * WGS84_E) + alt) * slat;
}

// From: https://danceswithcode.net/engineeringnotes/geodetic_to_ecef/geodetic_to_ecef.html
void ecefToGeodetic(double x, double y, double z, double *lat, double *lon, double *alt)
{
    double a = 6378137.0;              // WGS-84 semi-major axis
    double e2 = 6.6943799901377997e-3; // WGS-84 first eccentricity squared
    double a1 = 4.2697672707157535e+4; // a1 = a*e2
    double a2 = 1.8230912546075455e+9; // a2 = a1*a1
    double a3 = 1.4291722289812413e+2; // a3 = a1*e2/2
    double a4 = 4.5577281365188637e+9; // a4 = 2.5*a2
    double a5 = 4.2840589930055659e+4; // a5 = a1+a3
    double a6 = 9.9330562000986220e-1; // a6 = 1-e2

    double zp, w2, w, r2, r, s2, c2, s, c, ss;
    double g, rg, rf, u, v, m, f, p;

    zp = abs(z);
    w2 = x * x + y * y;
    w = sqrt(w2);
    r2 = w2 + z * z;
    r = sqrt(r2);
    *lon = atan2(y, x); // Lon (final)

    s2 = z * z / r2;
    c2 = w2 / r2;
    u = a2 / r;
    v = a3 - a4 / r;
    if (c2 > 0.3)
    {
        s = (zp / r) * (1.0 + c2 * (a1 + u + s2 * v) / r);
        *lat = asin(s); // Lat
        ss = s * s;
        c = sqrt(1.0 - ss);
    }
    else
    {
        c = (w / r) * (1.0 - s2 * (a5 - u - c2 * v) / r);
        *lat = acos(c); // Lat
        ss = 1.0 - c * c;
        s = sqrt(ss);
    }

    g = 1.0 - e2 * ss;
    rg = a / sqrt(g);
    rf = a6 * rg;
    u = w - rg * c;
    v = zp - rf * s;
    f = c * u + s * v;
    m = c * v - s * u;
    p = m / (rf / g + f);
    *lat = *lat + p;        // Lat
    *alt = f + m * p / 2.0; // Altitude
    if (z < 0.0)
    {
        *lat *= -1.0; // Lat
    }

    *lat *= radiansToDegrees; // Convert to degrees
    *lon *= radiansToDegrees;
}

    // Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to 
    // East-North-Up coordinates in a Local Tangent Plane that is centered at the 
    // (WGS-84) Geodetic point (lat0, lon0, h0).
    // https://gist.github.com/govert/1b373696c9a27ff4c72a
    void ecefToEnu(double x, double y, double z, double lat0, double lon0, double h0, double *xEast, double *yNorth, double *zUp)
    {
        // Convert to radians in notation consistent with the paper:
        double lambda = degreesToRadians*lat0;
        double phi = degreesToRadians*lon0;
        double s = sin(lambda);
        double N = WGS84_A / sqrt(1 - WGS84_E_sq * s * s);

        double sin_lambda = sin(lambda);
        double cos_lambda = cos(lambda);
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);

        double x0 = (h0 + N) * cos_lambda * cos_phi;
        double y0 = (h0 + N) * cos_lambda * sin_phi;
        double z0 = (h0 + (1 - WGS84_E_sq) * N) * sin_lambda;

        double xd, yd, zd;
        xd = x - x0;
        yd = y - y0;
        zd = z - z0;

        // This is the matrix multiplication
        *xEast = -sin_phi * xd + cos_phi * yd;
        *yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
        *zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;
    }

    // Inverse of EcefToEnu. Converts East-North-Up coordinates (xEast, yNorth, zUp) in a
    // Local Tangent Plane that is centered at the (WGS-84) Geodetic point (lat0, lon0, h0)
    // to the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
    // https://gist.github.com/govert/1b373696c9a27ff4c72a
    void enuToEcef(double xEast, double yNorth, double zUp, double lat0, double lon0, double h0, double *x, double *y, double *z)
    {
        // Convert to radians in notation consistent with the paper:
        double lambda = degreesToRadians*lat0;
        double phi = degreesToRadians*lon0;
        double s = sin(lambda);
        double N = WGS84_A / sqrt(1 - WGS84_E_sq * s * s);

        double sin_lambda = sin(lambda);
        double cos_lambda = cos(lambda);
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);

        double x0 = (h0 + N) * cos_lambda * cos_phi;
        double y0 = (h0 + N) * cos_lambda * sin_phi;
        double z0 = (h0 + (1 - WGS84_E_sq) * N) * sin_lambda;

        double xd = -sin_phi * xEast - cos_phi * sin_lambda * yNorth + cos_lambda * cos_phi * zUp;
        double yd = cos_phi * xEast - sin_lambda * sin_phi * yNorth + cos_lambda * sin_phi * zUp;
        double zd = cos_lambda * yNorth + sin_lambda * zUp;

        *x = xd + x0;
        *y = yd + y0;
        *z = zd + z0;
    }

    void checkHPPos()
    {
      Serial.println("fresh HPPOSLLH");
      // First, let's collect the position data
      int32_t latitude = GNSS_Rear.getHighResLatitude();
      int8_t latitudeHp = GNSS_Rear.getHighResLatitudeHp();
      int32_t longitude = GNSS_Rear.getHighResLongitude();
      int8_t longitudeHp = GNSS_Rear.getHighResLongitudeHp();
      int32_t ellipsoid = GNSS_Rear.getElipsoid();
      int8_t ellipsoidHp = GNSS_Rear.getElipsoidHp();
      int32_t msl = GNSS_Rear.getMeanSeaLevel();
      int8_t mslHp = GNSS_Rear.getMeanSeaLevelHp();
      uint32_t accuracy = GNSS_Rear.getHorizontalAccuracy();
  
      // Assemble the high precision latitude and longitude
      LAT = ((double)latitude) / 10000000.0; // Convert latitude from degrees * 10^-7 to degrees
      LAT += ((double)latitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )
      LON = ((double)longitude) / 10000000.0; // Convert longitude from degrees * 10^-7 to degrees
      LON += ((double)longitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )
      
      // Calculate the height above ellipsoid in mm * 10^-1
      ALT = (ellipsoid * 10) + ellipsoidHp;
      // Now convert to m
      ALT = ALT / 10000.0; // Convert from mm * 10^-1 to m  
   }
