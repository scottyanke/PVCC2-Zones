/* ========================================================================================
 *  This specific program is for Wemos D1 Mini's used to get temperature readings from DS18B20s,
 *  or from smaller (dumber) devices that have HTU21Ds or their own DS18B20s, but happen to be
 *  located in tunnels.
 *  The only purpose this Wemos (ESP8266) device serves is to communicate through the wifi guest
 *  network in the building to an mqtt server about what it knows.
 * ======================================================================================== */
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
// This is a small web server so that configuration changes can be made without attaching to any other network
#include <ESP8266WebServer.h>
#include <WiFiClient.h>
// AsyncMqttClient is used instead of PubSub, because it allows for QOS 2, and seems to work more reliably
#include <AsyncMqttClient.h>

#include <OneWire.h>
#include <DallasTemperature.h>

// The standard time functions, including one for timezones.
#include <TimeLib.h>
#include <Timezone.h>
tmElements_t tm, tm2;
int Year, Month, Day, Hour, Minute, Seconds, Wday ;
bool time_was_set = false;
uint16_t time_offset;

// The EEPROM functions are used so that the Wemos can store configuration data in the event
// of power failures.
#include <EEPROM.h>
#include <ModbusRtu.h>
// Note that this is the ESP version of the PCF8574 library.  There really is a difference and this works better
#include <pcf8574_esp.h>
//========================================================================================
// This device includes the ability to update over-the-air, using the standard Arduino
// library.  It is normally not connected to the guest wifi, but it is an AP all on its own,
// and OTA changes can be done through that method.  Just make sure that the PC side that
// has the update is connected to the 192.168.4.1 address, just like looking at the status
// or configuring the device.  With the ESPOTA.PY program, just use the 192.168.4.1 address
// while connected to the AP, and it will do the update just like serving the web pages.
// There is no need to even have Arduino on the portable PC side, as ESPOTA.PY is a command
// line python program that will connect to the appropriate address (always 192.168.4.1 in
// this case) and upload the binary, which has been assumed to be copied from the master PC
// anyway.  The command line looks like "python3 espota.py -r -i 192.168.4.1 -f mqtt4.ino.bin",
// which will also report the progress of the copy (using the -r) to the PC.  The bin file
// would have been copied from the /tmp directory on the master machine.
//========================================================================================
#include <ArduinoOTA.h>

// Common defines
// data array for modbus network sharing
uint16_t registers[10][60];   // This is where the data retrieved from modbus devices is stored
uint16_t last_registers[10][60];  // Used to compare values so only updates are send, not everything
uint8_t modbus_state;   // Current modbus state, from idle, to requesting, to receiving responses
uint8_t modbus_slave;   // Which slave are we talking to on modbus
unsigned long modbus_wait;  // Time between modbus requests
/**
 * This is an structure which contains a query to an slave device
 */
modbus_t telegram;


const char VERSION[] = __DATE__ " " __TIME__;
#define WIFI_SSID "my_ssid"
#define WIFI_PASSWORD "the_ssid_password"
#define WEBNAME "http://pvcc-hvac.6te.net/index.html"

#define MQTT_HOST IPAddress(192, 168, 0, 104)
#define MQTT_PORT 1883
#define MQTT_NAME "ZONES"
// Poll once every 600 seconds, or 10 minutes
#define TIME_BETWEEN_POLLING 600000
// 1800 seconds is half an hour
#define BLAST_INTERVAL 1800000

// The next few lines are used for the local timezone, since web servers supply world time
#define HTTP_TIME_SOURCE "http://mn.gov"
// US Central Time Zone (Chicago)
// This next line sets the correct time offset for the start of Central Daylight Time, on the second Sunday of March at 2am
TimeChangeRule myDST = {"CDT", Second, Sun, Mar, 2, -300};    // Daylight time = UTC - 5 hours
// and the mySTD variable is set for Central Standard Time, which starts the first Sunday in November
TimeChangeRule mySTD = {"CST", First, Sun, Nov, 2, -360};     // Standard time = UTC - 6 hours
Timezone myTZ(myDST, mySTD);

// Global class defines for some of the WiFi uses this device communicates through
HTTPClient http;
WiFiClient client;

ESP8266WebServer server(80);  // set the web server to port 80
AsyncMqttClient mqttClient;

Modbus modbusClient(0,0,0); // this is a client (meaning it commands other devices) and uses RS-232, not toggling RS485 which it uses anyway

OneWire oneWire(D5);  // The DS18B20s are connected to D5 on the Wemos
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature ds18b20s(&oneWire);
// Set i2c address
PCF857x pcf8574(0x20,&Wire);  // This is the default address of the PCF8574

// Global variables used throughout this program
bool ssidFound;     // Set if a scan finds the SSID we are expecting to connect to.
bool pcf8574Found = false;  // There is a PCF8574 that we can read the status of the pins from
char my_ssid[30];           // The SSID is generated from the last 3 bytes of this devices MAC address, so it can be used as a web server
long request_wait = 0;   // This is used to control when polling takes place, and is a millisecond value that can go negative
long blast_time = 0; // When the last forced mqtt update took place.
long PCF8574_wait = 0;  // The last time the PCF8574 was checked, since its on a different schedule than other devices
uint32_t read_time; // Only used to show on the web status page how long ago the readings were taken
String wifi_ssid;   // The SSID of the local guest network in the building
String wifi_password; // The password for the above SSID
String WebName;     // The WebName variable has the name of the external web server used as a fallback for finding the mqtt address.
byte mac_address[8];  // The unique address of this device, used to tell who is sending data and as the SSID for web access.
IPAddress mqtt_host;  // This is used for the IP address of the mqtt server to connect to.
IPAddress apIP(192, 168, 4, 1);  // The local address of this device when accessed through the web interface.

float temperature[10];  // An array of up to 10 local DS18B20s can be connected.
uint8_t lastPCF8574Value;

// QUEUE_WIDTH is the maximum size of an mqtt packet, and is a fixed length for this table
// QUEUE_MAX is the maximum number of entries that can be queued to be transmitted via mqtt
// QUEUE_ARRAY is the byte size of the queue table used for all of the queue entries.
// Note that the size of this table really impacts dynamic memory in the Wemos, and reduces space for local variables.
// Think carefully about the maximum number of queued entries.
#define QUEUE_WIDTH 128
#define QUEUE_MAX 240
#define QUEUE_ARRAY QUEUE_WIDTH * QUEUE_MAX
char queue[QUEUE_ARRAY];  // The global queue where all messages end up before being sent to the mqtt server
uint16_t queue_pos = 0;
uint8_t queue_len = 0;

// The eeprom information is used to store the connection information in the EEPROM on this device, so it can
// survive after a reboot or firmware update.
uint eeprom_addr = 0;
struct {
  char validate[6] = "";  // will have the word valid when it is valid
  char ssid[20] = "";
  char password[20] = "";
  IPAddress mqtt = IPAddress(0, 0, 0, 0);
  char webname[64] = "";
} eeprom_data;

// The next two variables are used for obtaining the time from a trusted external web page.
const char * headerKeys[] = {"date", "server"} ;
const size_t numberOfHeaders = 2;

//========================================================================================
// The setup() procedure takes care of reading the eeprom for any stored data, like the
// connection information and the address of the mqtt server, along with setting up the
// time, intializing the variables used during the loop() procedure, and getting the server
// side setup in case anything needs to be changed on the fly.
//========================================================================================
void setup()
{
  char ts[128];   // Primarily used for temporary strings of data, though not String data.
  uint8_t i,j;    // Small integers just used for counting
  float newTemp;
  uint8_t sensor_address[8];

  queue_pos = 0;
  queue_len = 0;
  pinMode(LED_BUILTIN, OUTPUT);    // This is really just an LED on the Wemos D1 Mini
  digitalWrite(LED_BUILTIN, HIGH); // Turn the light off
  // reset the holding registers to 0 to make sure they start clean
  for (j = 0; j < 10; j++)
    for (i = 0; i < 60; i++)
    {
      registers[j][i] = 0;
      last_registers[j][i] = 0;   // used for comparing current to previous to limit updates
    }
  Serial.begin(9600);     // All serial communications are at 9600,which is fine for what we are doing.
  pinMode(D6,INPUT_PULLUP); // used for 1-wire

  EEPROM.begin(512);  // try to get the connection info from EEPROM
  EEPROM.get(eeprom_addr,eeprom_data);
  if (!strcmp(eeprom_data.validate,"valid"))  // Is there a valid entry written in the EEPROM of this device
  {
    wifi_ssid = String(eeprom_data.ssid);
    wifi_password = String(eeprom_data.password);
    mqtt_host = eeprom_data.mqtt;
    WebName = String(eeprom_data.webname);
  }
  else  // Default to the defines is the EEPROM has not yet been programmed.
  {
    wifi_ssid = WIFI_SSID;
    wifi_password = WIFI_PASSWORD;
    mqtt_host = MQTT_HOST;
    WebName = WEBNAME;
  }
  mqttClient.setServer(mqtt_host, MQTT_PORT); // Set the name of the mqtt server.

  ds18b20s.begin();

  Wire.setClock(100000L);   // Only use a 100k clock frequency
  Wire.begin();
  Wire.beginTransmission (0x20);  // The address of the PCF8574
  if (Wire.endTransmission () == 0)
  {
    pcf8574Found = true;   // We did find a PCF8574 on the I2C bus, and will use it to read external binary events
    lastPCF8574Value = 0;  // this forces a transmit on the first pass
    pcf8574.begin(0xffff);  // make everything an input
  }

  ScanForWifi();  // look for the desired wifi ap, and get the time

  // This next chunk of code just gets the DS18B20s past the default 85C stage
  ds18b20s.requestTemperatures();
  oneWire.reset_search();
  j = 0;
  if (oneWire.search(sensor_address))
  {
    do
    {
      ds18b20s.getTempC(sensor_address);
      j++;
    } while (oneWire.search(sensor_address));
  }
  for (i = 0; i < 10; i++)    // temperature variable is just used for DS18B20s, allowing for 10 of them on this device
    temperature[i] = 0.0;

  modbusClient.begin( 9600 ); // begin the ModBus object.
  modbusClient.setTimeOut( 2000 ); // if there is no answer in 2 seconds, roll over to the next
  modbus_wait = millis() + 20000;   // wait 20 seconds from now before doing the first poll over modbus
  modbus_state = 0;
  modbus_slave = 0;
  request_wait = millis() + TIME_BETWEEN_POLLING + 20000;   // offset the normal requests by an additional 20 seconds
  blast_time = millis() + 60000;  // wait 60 seconds, then send everthing
  read_time = millis();

  WiFi.macAddress(mac_address);     // get the mac address of this chip to make it unique in the building
  sprintf(my_ssid,"%02X%02X%02X",mac_address[3],mac_address[4],mac_address[5]); // Use the hex version of the MAC address as our SSID
  WiFi.softAP(my_ssid, "87654321");             // Start the access point with password of 87654321

  delay(250);
  mqttClient.setClientId(my_ssid);    // Used to keep things runnng with QOS 2

  // Setup what to do with the various web pages used only for configuring/testing this device
  server.on("/config", handleConfig); // if we are told to change the configuration, jump to that web page
  server.on("/update", handleUpdate); // The handleUpdate() procedure is for program replacements done via OTA
  server.on("/", handleStatus);       // The most basic web request returns the status information
  server.on("/updateOTA", HTTP_POST, []()   // This chunk of code comes directly from the WebUpdate.ino example supplied under ESP8266WebServer.
  {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.setDebugOutput(true);
      WiFiUDP::stopAll();
      Serial.printf("Update: %s\n", upload.filename.c_str());
      uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
      if (!Update.begin(maxSketchSpace)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
      Serial.setDebugOutput(false);
    }
    yield();
  });
  // End of setting up the web server pages.

  server.begin();   // Start listening for connections to this devices AP.  Most of the time it's not needed, but helpful...
  delay((mac_address[5] + (mac_address[4] << 8)) & 0xBBF);  // do a random (based on mac address) delay just so everything doesn't try to talk at once

  // Another method of doing OTA updates.  It will probably be removed.
  ArduinoOTA.setHostname(my_ssid);
  ArduinoOTA.onError([](ota_error_t error) {  // the OTA process will call this next set of lines if needed.
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();   // Listen for OTA updates, which will probably come in over the local 192.168.4.1 address

  // This is the end of the setup routine.  Posting the VERSION helps identify that everything really is at the same version throughout the building.
  sprintf(ts,"R|000|Running v.%s",VERSION);
  add_to_queue(ts);   // tell the mqtt server that this device is up and running.  It isn't needed, but helpful.
  xmit_the_queue();   // xmit_the_queue() actually connects to the network, connects to the mqtt server, and sends the queued up data.
}

//========================================================================================
// Search for the desired access point, and if it is found, connect and get the time in
// the getTimeFromHttp() procedure.
//========================================================================================
void ScanForWifi()
{
  ssidFound = false;
  digitalWrite(LED_BUILTIN, LOW); // Turn the light on

  int n = WiFi.scanNetworks();  // Do a simple network check to see what's available.
  if (n == 0)
  {
    ; // No wifi networks were found, which can happen in the tunnels.
  }
  else
  {
    for (int i = 0; i < n; ++i)
    {
      if (wifi_ssid == WiFi.SSID(i))  // Check to see if the SSID we are supposed to connect to has been found by the scan
      {
        ssidFound = true;
        getTimeFromHttp();    // If the SSID was found then use it and get the time of day from the internet
        break;  // It was found, so stop looking.
      }
    }
  }
  digitalWrite(LED_BUILTIN, HIGH); // Turn the light off
}

//========================================================================================
// Pull the date and time using an HTTP request.  This is because the NTP port is blocked
// on the guest network that this system runs on.  It is annoying, but easier just to use
// the method of getting the time from a trusted web page.  This procedure will in turn
// call the timeElements() procedure with the timestring it got, which will compensate for
// the location and daylight savings.
//========================================================================================
void getTimeFromHttp() {
  int httpCode;
  String headerDate;

  connectToWifi();    // Connect to whatever guest network we are supposed to be on.
  if (WiFi.status() != WL_CONNECTED)  // If we couldn't connect through WiFi, then there's no point in continuing.
  {
    WiFi.disconnect();
    return;
  }
  http.begin(client,HTTP_TIME_SOURCE);   // A generally reliable place to get the date and time
  http.collectHeaders(headerKeys, numberOfHeaders);   // Just look at the headers
  httpCode = http.GET();
  if (httpCode > 0)
  {
    headerDate = http.header("date"); // We only want the date and time from the headers
    // headerDate looks like Sat, 19 Oct 2019 06:29:57 GMT
    timeElements(headerDate.c_str());   // set the date and time using what we got from the http request
  }
  http.end();   // We are done with our HTTP request for about 24 hours
  WiFi.disconnect();
}

//========================================================================================
// Parse the printable version of the date and time we got through HTTP into the appropriate
// format for setting the date and time within the Arduino program.  This takes a string
// value from an HTTP header.
//========================================================================================
void timeElements(const char *str)
{
  char t[80];
  int j;
  // Sat, 19 Oct 2019 06:29:57 GMT
  char delimiters[] = " :,";    // Used to separate the date and time fields
  char* valPosition;

  strcpy(t,str);
  valPosition = strtok(t, delimiters);
  j = 0;
  while(valPosition != NULL){
    switch(j) {
      case 0:   // Convert day of week into the proper Wday format.
        if (!strcmp(valPosition,"Sun")) Wday = 1;
        if (!strcmp(valPosition,"Mon")) Wday = 2;
        if (!strcmp(valPosition,"Tue")) Wday = 3;
        if (!strcmp(valPosition,"Wed")) Wday = 4;
        if (!strcmp(valPosition,"Thu")) Wday = 5;
        if (!strcmp(valPosition,"Fri")) Wday = 6;
        if (!strcmp(valPosition,"Sat")) Wday = 7;
        break;
      case 1: // Convert the day of the month into an integer.
        Day = atoi(valPosition);
        break;
      case 2: // Convert the month name into a simple integer.
        if (!strcmp(valPosition,"Jan")) Month = 1;
        if (!strcmp(valPosition,"Feb")) Month = 2;
        if (!strcmp(valPosition,"Mar")) Month = 3;
        if (!strcmp(valPosition,"Apr")) Month = 4;
        if (!strcmp(valPosition,"May")) Month = 5;
        if (!strcmp(valPosition,"Jun")) Month = 6;
        if (!strcmp(valPosition,"Jul")) Month = 7;
        if (!strcmp(valPosition,"Aug")) Month = 8;
        if (!strcmp(valPosition,"Sep")) Month = 9;
        if (!strcmp(valPosition,"Oct")) Month = 10;
        if (!strcmp(valPosition,"Nov")) Month = 11;
        if (!strcmp(valPosition,"Dec")) Month = 12;
        break;
      case 3: // Convert the year being passed into an integer.
        Year = atoi(valPosition);
        break;
      case 4: // Convert the printed hour into an integer.
        Hour = atoi(valPosition);
        break;
      case 5: // Convert the minute.
        Minute = atoi(valPosition);
        break;
      case 6: // An finally, convert the seconds passed into an integer.
        Seconds = atoi(valPosition);
        break;
    }
    j++;  // Look at the next position in the string
    //Here we pass in a NULL value, which tells strtok to continue working with the previous string
    valPosition = strtok(NULL, delimiters); // Use the strtok function to break the string into pieces
  }

  tm.Year = Year - 1970;  // Use the UNIX epoch since it is an offset
  tm.Month = Month;       // Setup the rest of the time values
  tm.Day = Day;
  tm.Hour = Hour;
  tm.Minute = Minute;
  tm.Second = Seconds;
  tm.Wday = Wday;

  setTime(myTZ.toLocal(makeTime(tm)));  // Call the myTX class to convert the date and time to the correct timezone, including daylight savings.

}

//========================================================================================
// Connect to the local guest network for long enough to either get the time, or to transmit
// a mqtt packet to the server.  Realistically, this program does a connection about every
// 30 minutes, maybe more often if it is monitoring something that changes frequently.
//========================================================================================
void connectToWifi()
{
  uint8_t i;
  if (!ssidFound)  // Check to see if the scan done at boot time actually found the SSID we want
    return;
  WiFi.disconnect();    // Sometimes we just need to reset everything.
  delay(50);
  WiFi.begin(wifi_ssid, wifi_password);
  i = 0;
  while (WiFi.status() != WL_CONNECTED && i < 120)   // Need to keep looking for about 15 seconds, because, yes, it can take that long to connect
  {
    i++;
    delay(125);
  }
}

//========================================================================================
// The getNewMqtt() procedure connects to the guest network, then gets the address of the
// mqtt server from a well known (to this program) website, just in case it changed.  Normally
// it wouldn't change, but because everything is running on the guest wifi network, anything
// could change at any time.
//========================================================================================
void getNewMqtt()   // gets a new mqtt server address if it changes by checking a web site
{
  char ts[128];
  IPAddress ip;

  connectToWifi();

  if (http.begin(client,WebName))
  {  // HTTP connection to the public server and page that has the mqtt address.
    // start connection and send HTTP header
    int httpCode = http.GET();
        // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been sent and Server response header has been handled
      // file found at server
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY || httpCode == HTTP_CODE_FOUND)
      {
        String payload = http.getString();
        payload.trim();   // get rid of the trailing newline
        ip.fromString(payload);
          sprintf(ts,"R|000|New mqtt address is going to be %d.%d.%d.%d",ip[0], ip[1], ip[2], ip[3]);
          add_to_queue(ts);
        if (mqtt_host != ip)  // check to see if the ip address changed
        {
          mqtt_host = ip;
          mqttClient.setServer(mqtt_host, MQTT_PORT);
          strcpy(eeprom_data.validate,"valid");
          wifi_ssid.toCharArray(eeprom_data.ssid,wifi_ssid.length() + 1);
          wifi_password.toCharArray(eeprom_data.password,wifi_password.length() + 1);
          eeprom_data.mqtt = IPAddress(ip[0], ip[1], ip[2], ip[3]);
          WebName.toCharArray(eeprom_data.webname,WebName.length() + 1);
          eeprom_addr = 0;
          EEPROM.put(eeprom_addr,eeprom_data);
          EEPROM.commit();
          add_to_queue((char *)"R|000|EEPROM has been changed");
          sprintf(ts,"R|000|New mqtt address is %d.%d.%d.%d",ip[0], ip[1], ip[2], ip[3]);
          add_to_queue(ts);
        }
      }
    } else {
      add_to_queue((char *)"E|[HTTP] GET... failed");
    }
  }
  http.end();
  delay(500);   // this is important.  Don't remove this delay
  WiFi.disconnect();
}

//========================================================================================
// The add_to_queue() procedure adds a character string to the mqtt queue.  As part of that
// process, it will also timestamp and format the added line.  It doesn't send anything,
// it just queues it internally to the Wemos.
//========================================================================================
void add_to_queue(char* str)
{
  char ts[128];

  // Everything added to the queue has the same prefix and suffix
  // The my_ssid part is simply diagnostics, except in the case of the boiler monitor.
  sprintf(ts, "%-6s|%04d-%02d-%02d %02d:%02d:%02d|%s|-|", my_ssid, year(), month(), day(), hour(), minute(), second(),str);
  ts[QUEUE_WIDTH - 1] = '\0';   // This is a safety thing, to make sure each queue entry isn't longer than it should be
  if (queue_len < (QUEUE_MAX))  // add to the queue if there is room
  {
    strcpy(queue+queue_pos,ts);
    queue_pos += QUEUE_WIDTH;
    queue_len++;
  }
}
//========================================================================================
// The xmit_the_queue() procedure takes all of the queued entries (added by add_to_queue())
// and sends them to the mqtt server.  It does this by connecting to the guest network, then
// sending the queued items in first-in-first-out order to the mqtt system.  After the queue
// has been cleared, it disconnects from the mqtt server, waits, then disconnects from the
// wifi guest network.
//========================================================================================
void xmit_the_queue()
{
  char ts[128];
  uint8_t j;
  uint16_t packetIdPub2;
  uint16_t queue_position;

  if (!queue_len)  // no sense connecting if there's nothing to send
    return;
  connectToWifi();
  if (WiFi.status() != WL_CONNECTED)
    return;
  mqttClient.connect();
  j = 0;
  while (!mqttClient.connected() && j < 200)  // give it up to 4 seconds to connect to mqtt
  {
    delay(20);
    j++;
  }
  if (mqttClient.connected() && queue_len)
  {
    queue_position = 0; // Used for getting info out of the queue FIFO, versus pulling from the end
    do
    {
      queue_len--;
      strcpy(ts,queue + queue_position);
      queue_position += QUEUE_WIDTH;
      if (strlen(ts) > 0)
      {
        // Message is being published with the 'Clean' session under QOS 2.
        packetIdPub2 = mqttClient.publish(MQTT_NAME, 2, true, ts);  // topic, qos, retain, payload, length=0, dup=false, message_id=0
        delay(250);   // We do have to wait for it to clear
      }
    } while (queue_len > 0);
    queue_pos = 0;    // reset the queue_pos for the next entries to be added to the queue in the future
    mqttClient.disconnect();
  };
  delay(500);   // this is important.  Don't remove this delay
  WiFi.disconnect();
}

//========================================================================================
// This simple function compares two values to see if they are different by more than maxDiff
//========================================================================================
bool checkDiff(float newValue, float prevValue, float maxDiff)
{
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

//========================================================================================
// This is the main processing loop for the system.
// The loop itself is really small.  Essentially it does stuff at certain
// intervals, and queues up the resulting responses.
//========================================================================================
void loop()
{
  int i,j;

  if (millis() < 1000)   // handle millis() rollovers that happen every 49.7 days
  {   // basically, just reset everything to startup values
    request_wait = millis() + TIME_BETWEEN_POLLING;
    blast_time = millis() + BLAST_INTERVAL;
    modbus_wait = millis() + 20000;
    PCF8574_wait = millis() + 15000;  // this gets read every 15 seconds
    delay(1000);  // just to get past millis being under 1000
  }
  ArduinoOTA.handle();      // One of the ways that over-the-air updates can be made.
  server.handleClient();    // used to handle configuration changes through the web interface

  switch( modbus_state )
  {
    case 0:
      if (millis() > modbus_wait)
      {
        modbus_state++; // wait state
        read_time = millis();
        digitalWrite(LED_BUILTIN, LOW); // Turn the light on
      }
      break;
    case 1:
      for (i = 0; i < 60; i++)
        registers[modbus_slave][i] = 0;
      telegram.u8id = 0x30 + modbus_slave; // slave address
      telegram.u8fct = 3; // function code (this one is registers read)
      telegram.u16RegAdd = 0; // start address in slave
      telegram.u16CoilsNo = 40; // number of elements (coils or registers) to read
      telegram.au16reg = &registers[modbus_slave][0]; // pointer to a memory array in the Arduino
      modbusClient.query( telegram ); // send query (only once)
      modbus_state++;   // now looking for responses
      modbus_slave++;   // set up for the next slave
      break;
    case 2:
      modbusClient.poll(); // check incoming messages
      if (modbusClient.getState() == COM_IDLE)  // COM_IDLE is after we've received our response
      {
        process_holding_registers(0);    // this queues up the data for mqtt
        modbus_state = 0;
        digitalWrite(LED_BUILTIN, HIGH); // Turn the light off
        if (modbus_slave > 9)
        {
          modbus_wait = millis() + TIME_BETWEEN_POLLING + time_offset;  // reset for the next loop
          modbus_slave = 0;
        }
      }
      break;
  }  // end of waking up on modbus_state

  // if we aren't looking from a response from a remote device, then it's ok to update the mqtt server if we need to
  if (modbus_state == 0)
  {
    // every 15 seconds check the pcf9574.
    if (millis() > PCF8574_wait && pcf8574Found)
    {
      PCF8574_wait = millis() + 15000;  // reset to check in another 15 seconds
      read_pcf8574_pins(0);   // Don't do a forced read
      xmit_the_queue();   // if anything is queued up, send it off
    }

    if (millis() > blast_time)
    {
      request_wait = millis() + TIME_BETWEEN_POLLING + time_offset;
      blast_time = millis() + BLAST_INTERVAL + time_offset;
      read_time = millis();

      get_local_ds18b20s(1);    // force a read of all the ds18b20s
      if (pcf8574Found)
        read_pcf8574_pins(1);    // this is a forced read, so we send the pins regardless of anything changing
      PCF8574_wait = millis() + 15000;
      process_holding_registers(1);    // this queues up the data for mqtt by forcing an update
      getNewMqtt();
      if (!ssidFound)   // check to see if we're on a network, or if we need to find one
        ScanForWifi();

      xmit_the_queue();   // send all of the queued data to the mqtt server
    }

    if (millis() > request_wait)
    {
      request_wait = millis() + TIME_BETWEEN_POLLING + time_offset;
      read_time = millis();
      get_local_ds18b20s(0); // get the local stuff
      // process the holding registers from the polled devices
      process_holding_registers(0);   // don't force an update

      xmit_the_queue();   // if anything is queued up, send it off
    }
    // Things get interesting at 2am.
    if (hour() == 2 && !time_was_set) // Get the time at 2am-ish, so we aren't too far off as time passes
	{                                 // The time_was_set variable is used so we only get the time once a day.
	  ScanForWifi();  // this gets the time as part of its function
	  time_was_set = true;  // say we got the 2am time so we don't loop around and try again immediately
	}
	if (hour() > 2)
	  time_was_set = false; // after 2am it is ok to reset this flag.
  }  // end of if (modbus_state == 0)
}

//========================================================================================
//========================================================================================
void process_holding_registers(uint8_t force)
{
  char temp_message[128];
  uint8_t i,j,k;    // It's amazing how the variables from fortran just hang around...
  float newTemp;

  // Note that modbus doesn't really handle negative values that well.  Everything is a positive integer.
  for (j = 0; j < 10; j++)  // for each unit that is polled we check the registers
  {
    k = 0;
    for (i = 0; i < 30; i++)
      if (registers[j][i] > 0)
        k = 1;    // all k does is indicate that there is a value other than 0 in one of the fields
    if (k)  // we got something from a device cause not everything is 0
    {
      for (i = 0; i < 6; i++)  // there are up to six ds18b20s on each remote device
      {
        if (registers[j][i*4] > 0)  // If this DS18B20 has a temperature that isn't 0, then queue it up
        {
          newTemp = float(registers[j][i * 4] / 100.0);
          if (checkDiff(newTemp, float(last_registers[j][i * 4] / 100.0), 0.55) || force)  // only update if something changed or we are forcing one
          {
            last_registers[j][i * 4] = registers[j][i * 4];
            sprintf(temp_message, "T|%c|%03d|%06.2f|%04x%04x%04x",0x30 + j, registers[j][25],newTemp,registers[j][(i*4)+1], registers[j][(i*4)+2], registers[j][(i*4)+3]);
            add_to_queue(temp_message);  // send to mqtt after queuing
          }
        }
      }

      // now pull the humidity readings from the HTU21D sensor on each remote device
      if (registers[j][28] > 0)
      {
        newTemp = float(registers[j][28] / 100.0);
        if (checkDiff(newTemp, float(last_registers[j][28] / 100.0), 0.55) ||
            checkDiff(float(registers[j][26] / 10.0), float(last_registers[j][26] / 10.0), 0.55) ||
            checkDiff(float(registers[j][27] / 10.0), float(last_registers[j][27] / 10.0), 0.55) || force) // only update if it changed from the last time
        {
          last_registers[j][26] = registers[j][26];
          last_registers[j][27] = registers[j][27];
          last_registers[j][28] = registers[j][28];
          sprintf(temp_message, "H|%c|%03d|%06.2f|%05.2f|%05.2f",0x30 + j, registers[j][25],newTemp,float(registers[j][26] / 10.0), float(registers[j][27] / 10.0));
          add_to_queue(temp_message);  // send to mqtt later
        }
      }
    }
  }
  // ------ End of processing the holding registers

}
//========================================================================================
//========================================================================================
void printBinary(byte inByte, char *ts)
{
  // if a pin is grounded (pulled low), then it is active, hence the '1'.  Pins left floating are low, the '0'
  for (int b = 7; b >= 0; b--)
  {
    if (bitRead(inByte, b))
    {
      *ts = '0';
      ts++;
    }
    else
    {
      *ts = '1';
      ts++;
    }
  }
}
//========================================================================================
//========================================================================================
void read_pcf8574_pins(uint8_t forced)
{
  char pcf8574_message[128];
  char ts[9];
  uint8_t val;

  pcf8574.write8(0xff);   // set all pins to input
  delay(25);
  val = pcf8574.read8();  // read all pins to see if any were pulled low
  if (val != lastPCF8574Value || forced)   // forced reads ignore whether the data has changed or not
  {
    printBinary((byte)val, ts);
    ts[8] = 0;
    sprintf(pcf8574_message, "P|000|%s", ts);
    add_to_queue(pcf8574_message);  // send to the queue to be sent to mqtt
    lastPCF8574Value = val;
  }
}
//========================================================================================
//========================================================================================
void get_local_ds18b20s(uint8_t forced)
{
  char temp_message[128];
  uint8_t sensor_address[8];
  uint8_t sensor_index;
  float newTemp;

  // We can do negative temperatures with the local DS18B20s, because we are not using modbus for them.
  // ------- Start of 1-wire section to read DS18B20s connected to this device -------
  // This is pretty much standard logic for searching for, and reading each DS18B20 on the wire.
  ds18b20s.requestTemperatures();
  oneWire.reset_search();
  sensor_index = 0;
  if (oneWire.search(sensor_address))
  {
    do
    {
      newTemp = ds18b20s.getTempC(sensor_address);
      if (checkDiff(newTemp, temperature[sensor_index], 0.55) || forced)
      {
        temperature[sensor_index] = newTemp;
        sprintf(temp_message, "T|000|%06.2f|%02x%02x%02x%02x%02x%02x", temperature[sensor_index],
          sensor_address[2],sensor_address[3],sensor_address[4],sensor_address[5],sensor_address[6],sensor_address[7]);
        add_to_queue(temp_message);  // send to the queue to be sent to mqtt
      }
      sensor_index++;
    } while (oneWire.search(sensor_address));
  }
  // ------ End of processing for 1-wire DS18B20 temperatures
}

//========================================================================================
// The web_modbus() routine is used by the web interface to force a read of the modbus
// devices.  That is its only usage.
//========================================================================================
void web_modbus()
{
  uint8_t state;
  uint8_t slave;
  uint8_t i;
  
  digitalWrite(LED_BUILTIN, LOW); // Turn the light on
  state = 1;
  slave = 0;
  do
  {
    switch( state )
    {
      case 0:
        digitalWrite(LED_BUILTIN, HIGH); // Turn the light off
        return;
      case 1:
        for (i = 0; i < 60; i++)
          registers[slave][i] = 0;
        telegram.u8id = 0x30 + slave; // slave address
        telegram.u8fct = 3; // function code (this one is registers read)
        telegram.u16RegAdd = 0; // start address in slave
        telegram.u16CoilsNo = 40; // number of elements (coils or registers) to read
        telegram.au16reg = &registers[slave][0]; // pointer to a memory array in the Arduino
        modbusClient.query( telegram ); // send query (only once)
        state++;   // now looking for responses
        slave++;   // set up for the next slave
        break;
    case 2:
      modbusClient.poll(); // check incoming messages
      if (modbusClient.getState() == COM_IDLE)  // COM_IDLE is after we've received our response
      {
        if (slave > 9)
        {
          state = 0;
          web_process_holding_registers();    // this prints the holding registers to the web page
        }
        else
          state = 1;
      }
      break;
    }  // end of processing the slaves through modbus
  } while (1);
}
//========================================================================================
// The web_process_holding_registers() routine is just going to print the results of the
// registers to the web page, regardless of anything changing.  It is mainly a trimmed down
// version of process_holding_registers.
//========================================================================================
void web_process_holding_registers()
{
  char temp_message[128];
  uint8_t i,j,k;    // It's amazing how the variables from fortran just hang around...
  float newTemp;

  // Note that modbus doesn't really handle negative values that well.  Everything is a positive integer.
  for (j = 0; j < 10; j++)  // for each unit that is polled we check the registers
  {
    k = 0;
    for (i = 0; i < 30; i++)
      if (registers[j][i] > 0)
        k = 1;    // all k does is indicate that there is a value other than 0 in one of the fields
    if (k)  // we got something from a device cause not everything is 0
    {
      for (i = 0; i < 6; i++)  // there are up to six ds18b20s on each remote device
      {
        if (registers[j][i*4] > 0)  // If this DS18B20 has a temperature that isn't 0, then print it
        {
          newTemp = float(registers[j][i * 4] / 100.0);
          sprintf(temp_message, "T|%c|%03d|%06.2f|%04x%04x%04x<br>",0x30 + j, registers[j][25],newTemp,registers[j][(i*4)+1], registers[j][(i*4)+2], registers[j][(i*4)+3]);
          server.sendContent(temp_message);  // send to mqtt after queuing
        }
      }

      // now pull the humidity readings from the HTU21D sensor on each remote device
      if (registers[j][28] > 0)
      {
        newTemp = float(registers[j][28] / 100.0);
        sprintf(temp_message, "H|%c|%03d|%06.2f|%05.2f|%05.2f<br>",0x30 + j, registers[j][25],newTemp,float(registers[j][26] / 10.0), float(registers[j][27] / 10.0));
        server.sendContent(temp_message);  // print the message to the web page
      }
    }
    else
    {
      sprintf(temp_message, "%c has nothing to print<br>",0x30 + j);
      server.sendContent(temp_message);
    }
  }
}

//========================================================================================
// handleConfig() is a procedure called by the web server.  It allows for changes to the
// network, the address of the mqtt server, and the public name of the web site that also
// has the mqtt server address.  Without this procedure the device will never connect to
// the local wifi network, unless everything was setup properly in eeprom.
//========================================================================================
void handleConfig() {
  char ts[1000];
  // note that embedded in this form are lots of variable fields pre-filled with the sprintf
  if (server.hasArg("ssid")&& server.hasArg("Password")&& server.hasArg("MQTT_IP")) //If all form fields contain data call handleSubmit()
    handleSubmit();
  else // Display the form
  {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    // here begin chunked transfer
    server.send(200, "text/html");
    server.sendContent("<!DOCTYPE HTML><html><head><meta content=\"text/html; charset=ISO-8859-1\" http-equiv=\"content-type\">" \
        "<meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0\">");
    server.sendContent("<title>Configuration</title><style>\"body { background-color: #808080; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }\"</style></head>");
    server.sendContent("<body><h1>Configuration</h1><FORM action=\"/config\" method=\"post\">");
    sprintf(ts,"<P><label>SSID:&nbsp;</label><input maxlength=\"30\" value=\"%s\" name=\"ssid\"><br>",wifi_ssid.c_str());
    server.sendContent(ts);
    sprintf(ts,"<label>Password:&nbsp;</label><input maxlength=\"30\" value=\"%s\" name=\"Password\"><br>",wifi_password.c_str());
    server.sendContent(ts);
    sprintf(ts,"<label>MQTT IP:&nbsp;</label><input maxlength=\"15\" value=\"%d.%d.%d.%d\" name=\"MQTT_IP\"><br> ",mqtt_host[0],mqtt_host[1],mqtt_host[2],mqtt_host[3]);
    server.sendContent(ts);
    sprintf(ts,"<label>HTTP Web Name:&nbsp;</label><input maxlength=\"63\" value=\"%s\" name=\"WebName\"><br> ",WebName.c_str());
    server.sendContent(ts);
    server.sendContent("<INPUT type=\"submit\" value=\"Send\"> <INPUT type=\"reset\"></P>");
    sprintf(ts,"<P><br>I think the time is %04d-%02d-%02d %02d:%02d:%02d</P></FORM></body></html>",year(), month(), day(), hour(), minute(), second());
    server.sendContent(ts);
  }
}

//========================================================================================
// Report on the status of everthing through the web interface, mainly from 192.168.4.1.
// This routine is very helpfull for ensuring everthing is setup properly, from the wifi
// network to the connection to the PSOC.
//========================================================================================
void handleStatus()
{
  char ts[100];
  int16_t last_read = (millis() - read_time) / 1000;
  char pcf8574_vals[9];
  uint8_t val;
  uint8_t sensor_address[8];

  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  // here begin chunked transfer
  server.send(200, "text/html");
  server.sendContent("<!DOCTYPE HTML><html><head><meta content=\"text/html; charset=ISO-8859-1\" http-equiv=\"content-type\">" \
      "<meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0\">");
  sprintf(ts,"<html><head><title>%s Status</title>",my_ssid);
  server.sendContent(ts);
  server.sendContent("</head><body>");
  sprintf(ts,"<P><h1>%s Status</h1><br>Version %s<br>",my_ssid, VERSION);
  server.sendContent(ts);
  sprintf(ts,"WiFi SSID is %s<br>",wifi_ssid.c_str());
  server.sendContent(ts);
  sprintf(ts,"WiFi password is %s<br>",wifi_password.c_str());
  server.sendContent(ts);
  sprintf(ts,"MQTT host is %d.%d.%d.%d<br>",mqtt_host[0],mqtt_host[1],mqtt_host[2],mqtt_host[3]);
  server.sendContent(ts);
  sprintf(ts,"Fallback name is %s<br>",WebName.c_str());
  server.sendContent(ts);
  sprintf(ts,"Last reading was %d seconds ago.<br>",last_read);
  server.sendContent(ts);
  sprintf(ts,"There are %d queued entries.<br>",queue_len);
  server.sendContent(ts);
  sprintf(ts,"I think the time is %04d-%02d-%02d %02d:%02d:%02d</P>", year(), month(), day(), hour(), minute(), second());
  server.sendContent(ts);
  if (pcf8574Found)
  {
    pcf8574.write8(0xff);
    delay(25);
    val = pcf8574.read8();
    printBinary((byte)val, pcf8574_vals);
    pcf8574_vals[8] = 0;
    sprintf(ts,"Pins are :%s<br>",pcf8574_vals);
    server.sendContent(ts);
  }
  else
    server.sendContent("No pins found.<br>");

  // ------- Start of 1-wire section to read DS18B20s connected to this device -------
  ds18b20s.requestTemperatures();
  oneWire.reset_search();
  if (oneWire.search(sensor_address))
  {
    do
    {
      sprintf(ts,"%6.2f on %02x%02x%02x%02x%02x%02x<br>",ds18b20s.getTempF(sensor_address),sensor_address[2],sensor_address[3],sensor_address[4],sensor_address[5],sensor_address[6],sensor_address[7]);
      server.sendContent(ts);
    } while (oneWire.search(sensor_address));
  }
  // ------ End of processing for 1-wire DS18B20 temperatures
  server.sendContent("Reading modbus devices...<br>");
  web_modbus();
  server.sendContent("Finished with modbus devices.<br>");
  server.sendContent("All sensors have been read");
  server.sendContent("</body></html>");
  server.sendContent(""); // this closes out the send
  server.client().stop();
}
//========================================================================================
// The handleSubmit() procedure takes whatever values were submitted by the /config page and
// updates the eeprom and global variables to the new ssid, password, mqtt server, etc.
// A restart is NOT needed after this procedure, as all of the communications is done on an
// as-needed basis using the global values that this updated.
//========================================================================================
void handleSubmit()
{//display values and write to memmory
  int i;
  IPAddress ip;
  String response="<!DOCTYPE HTML><html><head><meta content=\"text/html; charset=ISO-8859-1\" http-equiv=\"content-type\">" \
      "<meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0\">";
  response += "<p>The ssid is ";
  response += server.arg("ssid");
  response +="<br>";
  response +="And the password is ";
  response +=server.arg("Password");
  response +="<br>";
  response +="And the MQTT IP Address is ";
  response +=server.arg("MQTT_IP");
  response +="<br>";
  response +="And the Web Name is ";
  response +=server.arg("WebName");
  response +="</P><BR>";
  response +="<H2><a href=\"/\">go home</a></H2><br>";

  server.send(200, "text/html", response);
  // write data to EEPROM memory
  strcpy(eeprom_data.validate,"valid");
  i = server.arg("ssid").length() + 1;
  server.arg("ssid").toCharArray(eeprom_data.ssid,i);
  i = server.arg("Password").length() + 1;
  server.arg("Password").toCharArray(eeprom_data.password,i);
  ip.fromString(server.arg("MQTT_IP"));
  eeprom_data.mqtt = IPAddress(ip[0], ip[1], ip[2], ip[3]);
  i = server.arg("WebName").length() + 1;
  server.arg("WebName").toCharArray(eeprom_data.webname,i);

  eeprom_addr = 0;
  EEPROM.put(eeprom_addr,eeprom_data);
  EEPROM.commit();
  wifi_ssid = String(eeprom_data.ssid);
  wifi_password = String(eeprom_data.password);
  WebName = String(eeprom_data.webname);
  mqtt_host = eeprom_data.mqtt;
  mqttClient.setServer(mqtt_host, MQTT_PORT);
  ScanForWifi();
  ESP.restart();
}
//========================================================================================
// The handleUpdate() procedure sets up a simple web page with a couple of really, really,
// small buttons.  One button is for the selection of a binary file to replace the firmware
// of this device.  The other does the actual upload, update, and reboots the device.
//========================================================================================
void handleUpdate()
{
  String response="<html><body><form method='POST' action='/updateOTA' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";
  server.send(200, "text/html", response);
  server.sendContent("</body></html>");
  server.sendContent(""); // this closes out the send
  server.client().stop();

}

//========================================================================================
// A very simple web page is produced by handleNotFound() when an invalid web page is
// requested of this device.
//========================================================================================
void handleNotFound()
{
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  message +="<H2><a href=\"/\">go home</a></H2><br>";
  server.send(404, "text/plain", message);
}

//========================================================================================
// End of the program.
//========================================================================================
