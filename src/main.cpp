// modified to allow the use of the very miniature Ebyte E77 mcu & radio module all in one: STM32WLE5CCU6 
// and the miniature Ublox CAM-M8Q gps module for a very small (and light) form factor


//#include "SparkFun_Ublox_Arduino_Library.h" //https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <LOLIN_HP303B.h> //https://github.com/hakkican/HP303B_Library
#include <SPI.h>
#include <SubGhz.h>
#include <Wire.h> //Needed for I2C to GPS
#include "STM32RTC.h"
#include "STM32LowPower.h"
#include <RadioLib.h>  //https://github.com/jgromes/RadioLib
//#include "SparkFun_Ublox_Arduino_Library.h" // this is older library and is no longer supported
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS this is the newer library


#define BattPin A5
#define GpsPwr  12  // this turns on/off gps using mosfet switch
#define GpsON  digitalWrite(GpsPwr, LOW);
#define GpsOFF digitalWrite(GpsPwr, HIGH);

//SFE_UBLOX_GPS myGPS;
SFE_UBLOX_GNSS myGPS;
LOLIN_HP303B hp303b;


STM32WLx radio = new STM32WLx_Module(); 

//APRSClient aprs(&radio);  // had this in my code, but its not in this code. May not be needed

// set RF switch configuration for EBytes E77 dev board
// PB3 is an LED - activates while transmitting
// NOTE: other boards may be different!
//       Some boards may not have either LP or HP.
//       For those, do not set the LP/HP entry in the table.
static const uint32_t rfswitch_pins[] =
                         {PA6,  PA7,  PB3, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},
  {STM32WLx::MODE_RX,    {LOW, HIGH, LOW}},
  {STM32WLx::MODE_TX_LP, {HIGH, LOW, HIGH}},
  {STM32WLx::MODE_TX_HP, {HIGH, LOW, HIGH}},
  END_OF_MODE_TABLE,
};


//#define DEVMODE // Development mode. Uncomment to enable for debugging.

//************************** General Settings ********************
String callSign ="K6ATV-7";
String symbolCode = "O"; //Balloon symbol
String symbolTable = "/";
String statusMessage = "LoRa APRS";
String aprsComment = "LoRa APRS";
const unsigned txInterval = 60000;  // Schedule TX every this many miliseconds.
bool digipeaterMode = false; //This mode is experimental, works as a digi along with the tracker if enabled
float digiMinVolt=4.5; //Required minimum voltage to work as digipeater
//************************** LoRa APRS Default (Global) Settings ********************
float loraFrequency = 433.775;
float loraBandWith = 125.0f;
uint8_t spreadingFactor = 12; 
uint8_t codingRate = 5;
int8_t outputPower = 22;
uint16_t preambleLength = 8;
int8_t myCRC = 1;
String header = "<\xff\x01";//Header for https://github.com/lora-aprs/LoRa_APRS_iGate compatibility
String wide ="WIDE1-1";
//************************** uBlox GPS  Settings ********************
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.
bool gpsFix=false; //do not change this.
bool ublox_high_alt_mode_enabled = false; //do not change this.
bool gpsSetup=false; //do not change this.
//********************************* Power Settings ******************************
int   battWait=60;    //seconds sleep if super capacitors/batteries are below battMin (important if power source is solar panel) 
float battMin=3.5;    // min Volts to TX. (Works with 3.3V too but 3.5V is safer) 
float gpsMinVolt=4.5; //min Volts for GPS to wake up. (important if power source is solar panel) //do not change this
//********************************* Misc Settings ******************************
uint16_t txCount = 1;
uint16_t digipeated_packet_count=0;
float voltage;
bool packetQueued = false;
bool  aliveStatus = true; //for tx status message on first wake-up just once.
uint32_t lastPacket = 0; //do not change this. Timestamp of last packet sent.

uint32_t GEOFENCE_APRS_frequency = 433775000; //temp frequency before geofencing. This variable will be updated based on GPS location.
uint16_t GEOFENCE_APRS_spreading_factor = 12; //temp spreading factor before geofencing. This variable will be updated based on GPS location.
uint16_t GEOFENCE_APRS_coding_rate = 5; //temp spreading factor before geofencing. This variable will be updated based on GPS location.
uint16_t GEOFENCE_no_tx = 1;

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}


void sendAPRSMessage(String aprsMessage){
  radio.clearPacketReceivedAction();//Disable RX interrupt.
  receivedFlag = false;
  Serial.println(aprsMessage);
  int state = radio.transmit(aprsMessage);
  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("The packet was successfully transmitted..."));
  
    // print measured data rate
    Serial.print(F("[SX1268] Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));
  
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("The supplied packet was longer than 256 bytes!"));
  
  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Serial.println(F("Timeout occured while transmitting packet!"));
  
  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  
  }
}  


String getTrackerStatusAPRSMessage(){
  String message;
  String path = "";
  if (wide != "") {
    path = "," + wide;
  }

  message += callSign +">APLIGP"+path+":>"+statusMessage;
  return message;
}

void sendStatusMessage(){
  String aprsMessage = header + getTrackerStatusAPRSMessage();
  sendAPRSMessage(aprsMessage);
  txCount++;
}

void setupGPS() {
  GpsON;
  delay(100);
  Wire.begin();
  hp303b.begin(0x76);

  Wire.setClock(400000);

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // do not overload the buffer system from the GPS, disable UART output
  myGPS.setUART1Output(0); //Disable the UART1 port output 
  myGPS.setUART2Output(0); //Disable Set the UART2 port output
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  //myGPS.enableDebugging(); //Enable debug messages over Serial (default)

  myGPS.setNavigationFrequency(1);//Set output to 2 times a second. Max is 10
  byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
  Serial.print("Current update rate for GPS: ");
  Serial.println(rate);

  myGPS.saveConfiguration(); //Save the current settings to flash and BBR  
  
}

void digiTX(){

  String packet;
  int state = radio.readData(packet);
          
  if (state == RADIOLIB_ERR_NONE) {
               
    if(packet.length() >0) {
      // packet was successfully received
          
      String aprsMessage;
          
      if ((packet.substring(0, 3) == "\x3c\xff\x01") && (packet.indexOf("TCPIP") == -1) && (packet.indexOf("NOGATE") == -1)) {
        String sender = packet.substring(3,packet.indexOf(">"));
        if ((packet.indexOf("WIDE1-1") > 10) && (callSign != sender)) {
          aprsMessage = packet.substring(3);                         
          aprsMessage.replace("WIDE1-1", callSign + "*");
          Serial.print(F("Digipeating: "));
          sendAPRSMessage(header + aprsMessage);
          digipeated_packet_count++;
        }
      } else {
        Serial.println(packet);
        Serial.println("LoRa Packet Ignored (first 3 bytes or TCPIP/NOGATE/RFONLY)");
      }            
    }
        
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));
  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

void enableRX(){
  if (digipeaterMode && gpsFix && voltage > digiMinVolt && GEOFENCE_no_tx != 1) {
    // set the function that will be called
    // when new packet is received
    radio.setPacketReceivedAction(setFlag);

    Serial.print(F("[SX1268] Starting to listen ... "));
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      //while (true);
    }          
  }
}

void printGPSandSensorData(){
 
  lastTime = millis(); //Update the timer

  byte fixType = myGPS.getFixType();

  Serial.print(F("FixType: "));
  Serial.print(fixType);    

  int SIV = myGPS.getSIV();
  Serial.print(F(" Sats: "));
  Serial.print(SIV);

  float flat = myGPS.getLatitude() / 10000000.f;
    
  Serial.print(F(" Lat: "));
  Serial.print(flat);    

  float flong = myGPS.getLongitude() / 10000000.f;    
  Serial.print(F(" Long: "));
  Serial.print(flong);        

  float altitude = myGPS.getAltitude() / 1000;
  Serial.print(F(" Alt: "));
  Serial.print(altitude);
  Serial.print(F(" (m)"));

  //float speed = myGPS.getGroundSpeed() * 0.0036f;
  //Serial.print(F(" Speed: "));
  //Serial.print(speed);
  //Serial.print(F(" (km/h)"));

  //long heading = myGPS.getHeading() / 100000;
  //Serial.print(F(" Heading: "));
  //Serial.print(heading);
  //Serial.print(F(" (degrees)"));
        
  Serial.print(" Time: ");    
  Serial.print(myGPS.getYear());
  Serial.print("-");
  Serial.print(myGPS.getMonth());
  Serial.print("-");
  Serial.print(myGPS.getDay());
  Serial.print(" ");
  Serial.print(myGPS.getHour());
  Serial.print(":");
  Serial.print(myGPS.getMinute());
  Serial.print(":");
  Serial.print(myGPS.getSecond());
    
  int16_t oversampling = 7;
  int16_t ret;
  int32_t temperature;
  int32_t pressure;
  ret = hp303b.measureTempOnce(temperature, oversampling);
  if(ret !=0){
    Serial.print("hp303b fail! ret = ");
    Serial.println(ret);
  }
    
  Serial.print(" Temp: ");
  Serial.print(temperature);
  Serial.print(" C");  

  ret = hp303b.measurePressureOnce(pressure, oversampling);
  if(ret !=0){
    Serial.print("hp303b fail! ret = ");
    Serial.println(ret);
  }
       
  Serial.print(" Press: ");    
  Serial.print(pressure / 100.0);
  Serial.print(" hPa");

  Serial.println();
  delay(500);
}  

float readBatt() {

  float R1 = 560000.0; // 560K
  float R2 = 100000.0; // 100K
  float value = 0.0f;

  do {    
    value =analogRead(BattPin);
    value +=analogRead(BattPin);
    value +=analogRead(BattPin);
    value = value / 3.0f;
    value = (value * 3.3) / 1024.0f;
    value = value / (R2/(R1+R2));
  } while (value > 20.0);
    return value ;

}


void setupUBloxDynamicModel() {
  // If we are going to change the dynamic platform model, let's do it here.
  // Possible values are:
  // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
  //DYN_MODEL_AIRBORNE4g model increases ublox max. altitude limit from 12.000 meters to 50.000 meters. 
  if (myGPS.setDynamicModel(DYN_MODEL_AIRBORNE1g) == false) {   // Set the dynamic model to DYN_MODEL_AIRBORNE4g
  
    Serial.println(F("***!!! Warning: setDynamicModel failed !!!***"));
  }
  else
  {
    ublox_high_alt_mode_enabled = true;
    #if defined(DEVMODE)
      Serial.print(F("Ublox dynamic platform model (DYN_MODEL_AIRBORNE4g) changed successfully! : "));
      Serial.println(myGPS.getDynamicModel());
    #endif  
  }
  
} 

void setupLoRa() {  

  // initialize SX1262 with default settings
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);  // set up for internal SX1262 transceiver 
  Serial.print(F("[SX1268] LoRa Radio Module Initializing ... "));  
  int state = radio.begin();

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  if (radio.setFrequency(loraFrequency,true) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    Serial.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }

  if (radio.setBandwidth(loraBandWith) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    Serial.println(F("Selected bandwidth is invalid for this module!"));
    while (true);
  }

  if (radio.setSpreadingFactor(spreadingFactor) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }

  if (radio.setCodingRate(codingRate) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }

  if (radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE) != RADIOLIB_ERR_NONE) {
    Serial.println(F("Unable to set sync word!"));
    while (true);
  }

  // set over current protection limit to 140 mA (accepted range is 45 - 240 mA)
  if (radio.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
    Serial.println(F("Selected current limit is invalid for this module!"));
    while (true);
  }   

  // set output power (accepted range is -17, 22 dBm)
  if (radio.setOutputPower(outputPower) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    Serial.println(F("Selected output power is invalid for this module!"));
    while (true);
  }

  // set LoRa preamble length (accepted range is 0 - 65535)
  if (radio.setPreambleLength(preambleLength) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
    Serial.println(F("Selected preamble length is invalid for this module!"));
    while (true);
  }

  // enable CRC
  if (radio.setCRC(myCRC) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
    Serial.println(F("Selected CRC is invalid for this module!"));
    while (true);
  }

  radio.setRxBoostedGainMode(true);
  Serial.println(F("All settings succesfully changed!"));  

 }

 String encodeHMSTimestamp(int hour, int minute, int second){
  char timestamp_buff[8];
  sprintf(timestamp_buff, "%02d", hour);
  sprintf(timestamp_buff + 2, "%02d", minute);
  sprintf(timestamp_buff + 4, "%02d", second);
  timestamp_buff[6] = 'h';

  String timestamp;
    for (uint8_t i = 0; i < 7; i++)
    {
      timestamp += String((char)timestamp_buff[i]);
    }  

  return timestamp;
}

String createLatForAPRS(double latitude) {
  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char lat_buff[10];
  int temp = 0;
  double dm_lat = 0.0;

  if (latitude < 0.0) {
    temp = -(int)latitude;
    dm_lat = temp * 100.0 - (latitude + temp) * 60.0;
  } else {
    temp = (int)latitude;
    dm_lat = temp * 100 + (latitude - temp) * 60.0;
  }

  dtostrf(dm_lat, 7, 2, lat_buff);

  if (dm_lat < 1000) {
    lat_buff[0] = '0';
  }

  if (latitude >= 0.0) {
    lat_buff[7] = 'N';
  } else {
    lat_buff[7] = 'S';
  }

  String latStr;
    for (uint8_t i = 0; i < 8; i++)
    {
      latStr += String((char)lat_buff[i]);
    }  

  return latStr;

}

String createLongForAPRS(double longitude) {
  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char long_buff[10];
  int temp = 0;
  double dm_lon = 0.0;

  if (longitude < 0.0) {
    temp = -(int)longitude;
    dm_lon = temp * 100.0 - (longitude + temp) * 60.0;
  } else {
    temp = (int)longitude;
    dm_lon = temp * 100 + (longitude - temp) * 60.0;
  }

  dtostrf(dm_lon, 8, 2, long_buff);

  if (dm_lon < 10000) {
    long_buff[0] = '0';
  }
  if (dm_lon < 1000) {
    long_buff[1] = '0';
  }

  if (longitude >= 0.0) {
    long_buff[8] = 'E';
  } else {
    long_buff[8] = 'W';
  }

  String longStr;
    for (uint8_t i = 0; i < 9; i++)
    {
      longStr += String((char)long_buff[i]);
    }  

  return longStr;
}

String getTrackerLocationAPRSMessage() {
  String message;
  //Latitude and Longitude
  String path = "";
  if (wide != "") {
    path = "," + wide;
  }
  message += callSign +">APLIGP"+path+":/";

  String timestamp = encodeHMSTimestamp(myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond()); //Zulu time DHM timestamp
  message += timestamp;

  String latitude = createLatForAPRS(myGPS.getLatitude() / 10000000.f);
  String longitude = createLongForAPRS(myGPS.getLongitude() / 10000000.f);
  message += latitude + symbolTable+longitude+symbolCode;

  //Course and Speed
  char course_speed_buff[10];

  uint16_t heading = (uint16_t)(myGPS.getHeading() / 100000);
  uint16_t speed = (uint16_t)(myGPS.getGroundSpeed() * 0.00194384f);
  //double altitude = (double)(myGPS.getAltitude() * 3.2808399d)  / 1000.d;
  double altitude = (double)(myGPS.getAltitude() * 3.2808399)  / 1000;
  uint16_t sattelite = (uint16_t)(myGPS.getSIV());
  sprintf(course_speed_buff, "%03d", heading);
  course_speed_buff[3] += '/';
  sprintf(course_speed_buff + 4, "%03d", speed);

  String course_speed_str;
  for (uint8_t i = 0; i <7; i++)
  {
    course_speed_str += String((char)course_speed_buff[i]);
  }
  message += course_speed_str;

  //Altitude
  String alt_str="";
  char alt_buff[10]; 
  alt_buff[0] = '/';
  alt_buff[1] = 'A';
  alt_buff[2] = '=';

  if (altitude > 0) {
    //for positive values
    sprintf(alt_buff + 3, "%06lu", (long)altitude);
  } else {
    //for negative values
    sprintf(alt_buff + 3, "%06ld", (long)altitude);
  }
  for (uint8_t i = 0; i <9; i++)
  {
    alt_str += String((char)alt_buff[i]);
  }

  message += alt_str;

  //TXCount, Temperature, Pressure and Voltage

  String txCountComment = " "+String(txCount)+"TXC";
  String digiTXCountComment = " DigiC:"+String(digipeated_packet_count);

  int16_t oversampling = 7;
  int32_t temperature;
  int32_t pressure;
  hp303b.measureTempOnce(temperature, oversampling);   
  hp303b.measurePressureOnce(pressure, oversampling);

  String temperatureComment = " "+String(temperature)+"C";
  
  String pressureComment; 
  String pressure_str = "";
  float pressureF = pressure / 100.0f; //Pa to hPa
  char pressure_buff[6];
  dtostrf(pressureF, 7, 2, pressure_buff);     
  for (uint8_t i = 0; i < 6; i++)          
  {
    pressure_str += String((char)pressure_buff[i]);
  }
  pressureComment = " "+pressure_str+"hPa";

  String voltageComment;
  char voltage_buff[3];
  dtostrf(voltage, 3, 2, voltage_buff);

  String voltage_str;
  for (uint8_t i = 0; i < 3; i++)
  {
    voltage_str += String((char)voltage_buff[i]);
  }    
    voltageComment =  " "+voltage_str+"V";

  String satComment = " "+String(sattelite)+"S ";
    
  if (digipeaterMode) {
      message += txCountComment + digiTXCountComment + temperatureComment + pressureComment + voltageComment + satComment + aprsComment;
    } else {
      message += txCountComment + temperatureComment + pressureComment + voltageComment + satComment + aprsComment;
    }
     
  return message;
}

void sendLocationMessage(){
    String aprsMessage = header + getTrackerLocationAPRSMessage();
    sendAPRSMessage(aprsMessage);
    txCount++;
    //Enable RX for digipeating if possible.
    enableRX();
}


int32_t pointInPolygonF(int32_t polyCorners, float * polygon, float latitude, float longitude){
  int32_t i;
  int32_t j = polyCorners * 2 - 2;
  int32_t oddNodes = 0;

  for(i = 0; i < polyCorners * 2; i += 2) {
    if((polygon[i + 1] < latitude && polygon[j + 1] >= latitude
    || polygon[j + 1] < latitude && polygon[i + 1] >= latitude)
    && (polygon[i] <= longitude || polygon[j] <= longitude))
    {
      oddNodes ^= (polygon[i] + (latitude - polygon[i + 1])
      / (polygon[j + 1] - polygon[i + 1]) * (polygon[j] - polygon[i]) < longitude);
    }

    j = i;
  }

  return oddNodes;
}

void GEOFENCE_position(float latitude, float longitude){

  float  UKF[] = {
    -0.65920,   60.97310,
    -7.58060,   58.07790,
    -8.21780,   54.23960,
    -4.76810,   53.80070,
    -5.86670,   49.76710,
    1.30740,    50.85450,
    1.86770,    52.78950,
    -2.04350,   55.97380,
    -0.65920,   60.97310
  }; 
  
  float  PolandF[] = {
    54.439234, 14.103456,
    51.022075, 14.897034,
    49.537732, 18.860178,
    49.113780, 22.481393,
    52.742317, 23.889542,
    54.156275, 23.456098,
    54.439234, 14.103456
  };      

  static float LatviaF[] = {
    26.64180,   55.68380,
    28.17990,   56.20670,
    27.78440,   57.33250,
    25.00490,   58.00230,
    24.14790,   57.17200,
    21.78590,   57.68650,
    20.81910,   56.07200,
    22.19240,   56.44430,
    25.68600,   56.18230,
    26.64180,   55.68380
  };      
  
  static float YemenF[] = {
    52.20302, 19.48287,
    41.92009, 17.42198,
    43.59620, 12.27820,
    53.68201, 15.80024,
    52.20302, 19.48287
  };
  
  static float North_KoreaF[] = {
    130.14189, 43.21627,
    124.03574, 39.8949,
    125.18292, 37.39202,
    128.47240, 38.66888,
    130.69478, 42.3274,
    130.14189, 43.21627
  };             

  if(pointInPolygonF(9, UKF, latitude, longitude) == 1){
    GEOFENCE_no_tx = 1; //Airborne TX is not allowed in UK
    GEOFENCE_APRS_frequency = 439912500;
    GEOFENCE_APRS_spreading_factor = 12;
    GEOFENCE_APRS_coding_rate = 5;
  } else if(pointInPolygonF(7, PolandF, latitude, longitude) == 1){
    GEOFENCE_no_tx = 0;
    GEOFENCE_APRS_frequency = 434855000;
    GEOFENCE_APRS_spreading_factor = 9;
    GEOFENCE_APRS_coding_rate = 7;          
  } else if(pointInPolygonF(10, LatviaF, latitude, longitude) == 1){
    GEOFENCE_no_tx = 1;
    GEOFENCE_APRS_frequency = 433775000;
    GEOFENCE_APRS_spreading_factor = 12;
    GEOFENCE_APRS_coding_rate = 5;                
  } else if(pointInPolygonF(5, YemenF, latitude, longitude) == 1){
    GEOFENCE_no_tx = 1;
    GEOFENCE_APRS_frequency = 433775000;
    GEOFENCE_APRS_spreading_factor = 12;
    GEOFENCE_APRS_coding_rate = 5; 
  } else if(pointInPolygonF(6, North_KoreaF, latitude, longitude) == 1){
    GEOFENCE_no_tx = 1;
    GEOFENCE_APRS_frequency = 433775000;
    GEOFENCE_APRS_spreading_factor = 12;
    GEOFENCE_APRS_coding_rate = 5;              
  } else {
    GEOFENCE_no_tx = 0;
    GEOFENCE_APRS_frequency = 433775000;
    GEOFENCE_APRS_spreading_factor = 12;
    GEOFENCE_APRS_coding_rate = 5;      
  }
}

void configureFreqbyLocation() {

  float tempLat = myGPS.getLatitude() / 10000000.f;
  float tempLong = myGPS.getLongitude() / 10000000.f; 

  GEOFENCE_position(tempLat,tempLong);  
  loraFrequency = GEOFENCE_APRS_frequency / 1000000.f;
  if (radio.setFrequency(loraFrequency,true) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    Serial.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }

  if (radio.setSpreadingFactor(GEOFENCE_APRS_spreading_factor) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }

  if (radio.setCodingRate(GEOFENCE_APRS_coding_rate) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }
}

void setup() {
  
  pinMode(A1,INPUT_PULLUP);
  if(digitalRead(A1)==LOW) while(1);

  delay(5000); //do not change this

  pinMode(GpsPwr, OUTPUT);    
  GpsOFF; 
  
  Serial.begin(115200);
    
  // Wait up to 5 seconds for serial to be opened, to allow catching
  // startup messages on native USB boards (that do not reset when
  // serial is opened).
  unsigned long start = millis();
  while (millis() - start < 5000 && !Serial);

  Serial.println();
  Serial.println(F("Starting"));
  Serial.println();

  Serial.print(F("APRS CallSign: "));
  Serial.println(callSign);
  Serial.println();

}

void loop() {

  voltage = readBatt();

  if (((voltage > battMin) && gpsFix) || ((voltage > gpsMinVolt) && !gpsFix)) {

    if(!gpsSetup){
      
      Serial.println(F("GPS setup"));   
      setupGPS();
      Serial.println(F("Searching for GPS fix...")); 
      gpsSetup = true;

      Serial.println(F("LoRa setup"));
      setupLoRa();
      
    }

    if (aliveStatus) {
      Serial.println(F("Sending status message..."));
      sendStatusMessage();       
      aliveStatus = false;

      while (readBatt() < battMin) {
        delay(battWait * 1000);
      }
    }  


    // If txInterval passed send the next packet.
    if ((millis() - lastPacket > txInterval) || !gpsFix){
    
      GpsON;
      delay(500);

      if(!ublox_high_alt_mode_enabled){
        setupUBloxDynamicModel();
      }

      // Calling myGPS.getPVT() returns true if there actually is a fresh navigation solution available.

      if (myGPS.getPVT() && (myGPS.getFixType() !=0) && (myGPS.getSIV() > 3)) {
        gpsFix=true;     

        if (txCount == 2 || txCount == 200) {
          configureFreqbyLocation();
        }

        //need to save power
        if (readBatt() < 4.5 || digipeaterMode) {
        GpsOFF;
        ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.
        delay(500);      
        }
        
        if (GEOFENCE_no_tx != 1) {
          sendLocationMessage();
          lastPacket = millis();          
        }                 
      }


      #if defined(DEVMODE)
        printGPSandSensorData();
      #endif    

    }

    if(receivedFlag) {
      // reset flag
      receivedFlag = false;
      //Serial.println(F("[SX1268] Received packet!"));
      digiTX();             
    }

    //this code block protecting serial connected (3V + 3V) super caps from overcharging by powering on GPS module.
    //GPS module uses too much power while on, so if voltage is too high for supercaps, GPS ON.
    if (!digipeaterMode && readBatt() > 6.5) {
      GpsON;
      delay(500);
    
    } else {

      GpsOFF;
      ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.     
      delay(battWait * 1000);
      
    }

    #if defined(DEVMODE)
      printGPSandSensorData();
    #endif    

  }

  if(receivedFlag) {
    // reset flag
    receivedFlag = false;
    //Serial.println(F("[SX1268] Received packet!"));
    digiTX();             
  }

  //this code block protecting serial connected (3V + 3V) super caps from overcharging by powering on GPS module.
  //GPS module uses too much power while on, so if voltage is too high for supercaps, GPS ON.
  if (!digipeaterMode && readBatt() > 6.5) {
    GpsON;
    delay(500);
   
  } else {
    GpsOFF;
    ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.     
    delay(battWait * 1000);
  }
}