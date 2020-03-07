/* ========================================================================================
 *  This specific program is for Wemos D1 Mini's used to get temperature readings from DS18B20s,
 *  or from smaller (dumber) devices that have HTU21Ds or their own DS18B20s, but happen to be
 *  located in tunnels.
 *  The only purpose this Wemos (ESP8266) device serves is to communicate through the wifi guest
 *  network in the building to an mqtt server about what it knows.
 *  
 *  The IDE options for this device are:
 *  Board: LOLIN(WENOS) D1 R2 & mini
 *  Upload Speed: "921600"
 *  CPU Frequency: "80 Mhz"
 *  Flash Size: "4MB (FS:1MB OTA:~1019KB)"
 *  Debug port: "Disabled"
 *  Debug Level: "None"
 *  LwIP Variant: "V2 Lower Memory"
 *  VTables: "Flash"
 *  Exceptions: "Legacy (new can return nullptr)"
 *  Erase Flash: "Only Sketch"
 *  SSL Support: "All SSL ciphers (most compatible)"
 *  
 *  The library versions used by this program are:
 *  Wire at version 1.0
 *  ESP8266WiFi at version 1.0
 *  ESP8266HTTPClient at version 1.2
 *  ESP8266WebServer at version 1.0
 *  async-mqtt-client at version 0.8.2
 *  ESPAsyncTCP at version 1.2.0
 *  OneWire at version 2.3.5
 *  DallasTemperature at version 3.8.0
 *  Modbus-Master-Slave-for-Arduino
 *  Time at version 1.6 in folder
 *  Timezone at version 1.2.4
 *  EEPROM at version 1.0
 *  
 *  Executable segment sizes:
 *  IROM   : 302452          - code in flash         (default or ICACHE_FLASH_ATTR) 
 *  IRAM   : 28412   / 32768 - code in IRAM          (ICACHE_RAM_ATTR, ISRs...) 
 *  DATA   : 1276  )         - initialized variables (global, static) in RAM/HEAP 
 *  RODATA : 3064  ) / 81920 - constants             (global, static) in RAM/HEAP 
 *  BSS    : 27200 )         - zeroed variables      (global, static) in RAM/HEAP
 * ======================================================================================== */
ADC_MODE(ADC_VCC)
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>
// This is a small web server so that configuration changes can be made without attaching to any other local network
#include <ESP8266WebServer.h>
// AsyncMqttClient is used instead of PubSub, because it allows for QOS 2, and seems to work more reliably
#include <AsyncMqttClient.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include "ModbusRtuX.h"   // Used for reading other devices with the modbus protocol over RS485
// ModbusRtuX.h is a stripped down version of ModbusRtu.h.  It only has the procedures for the
// master, and the functions for reading registers.  The original will work, but things like
// the software serial are not needed, nor are the slave functions.  Just trying to save memory.

// The standard time functions, including one for timezones.
#include <TimeLib.h>
#include <Timezone.h>
bool time_was_set = false;
uint16_t time_offset;
/**
 * This system keeps the time in UTC, and converts to local time on demand.  That way it can handle
 * the Daylight Savings issues that pop up twice a year.  This works out well as it gets the time
 * in UTC anyway...
 */
// US Central Time Zone (Chicago)
// This next line sets the correct time offset for the start of Central Daylight Time, on the second Sunday of March at 2am
TimeChangeRule myDST = {"CDT", Second, Sun, Mar, 2, -300};    // Daylight time = UTC - 5 hours
// and the mySTD variable is set for Central Standard Time, which starts the first Sunday in November
TimeChangeRule mySTD = {"CST", First, Sun, Nov, 2, -360};     // Standard time = UTC - 6 hours
Timezone myTZ(myDST, mySTD);  // These are used so much that they might as well be global
  
// The EEPROM functions are used so that the Wemos can store configuration data in the event
// of power failures.  Yes, there is one specifically for the ESP, but it parks the data all
// over the place, and makes it difficult to read.
#include <EEPROM.h>

// Common defines
/**
 * This is an structure which contains a query to an slave device
 */
modbus_t telegram;

const char VERSION[] = __DATE__ " " __TIME__;
#define WIFI_SSID "my_ssid"
#define WIFI_PASSWORD "the_ssid_password"
#define ALT_WIFI_SSID "Poiuytrewq"
#define ALT_WIFI_PASSWORD "sparky050504"
#define WEBNAME "X"

#define MQTT_HOST IPAddress(192, 168, 0, 104)
#define ALT_MQTT_HOST IPAddress(192, 168, 0, 104)
#define MQTT_PORT 1883
// This program (device) publishes to the "Zones" subject on the mqtt server, versus the use of "AHU" by the air handlers.
#define MQTT_NAME "Zones"
// Poll once every 600 seconds, or 10 minutes
#define TIME_BETWEEN_POLLING 600000
// 1800 seconds is half an hour
#define BLAST_INTERVAL 1800000
// The I2C address of the connected PCF8574 (if it is there)
#define PCF8574_I2C 0x20
// Check the PCF8574 every 15 seconds
#define PCF8574_INTERVAL 15000
// The I2C address of the PCF8591
#define PCF8591_I2C 0x48
// Check the PCF8591 once every minute
#define PCF8591_INTERVAL 60000
// There are at most 10 devices, and each device has 30 registers we are interested in.  Up to 10 DS19B20s can be read.
#define MODBUS_DEVICES 10
#define MODBUS_REGISTERS 30
#define DS18B20_DEVICES 10
// MILLIS_MAX is the trip point for rebooting this device, due to weird problems connecting in the production environment after a few days.
// It will boot itself if the current millis() value is higher than this number, and it's 00:30 in the morning.
#define MILLIS_MAX 172800000

// The next few lines are used for the local timezone, since web servers supply world time.  The state's web server is a reliable choice.
#define HTTP_TIME_SOURCE "http://mn.gov"
int last_hour;    //  Simply used to limit how often the address is checked for, and diagnostics are reported

// Global class defines for some of the WiFi uses this device communicates through
HTTPClient http;
WiFiClient client;

ESP8266WebServer server(80);  // set the web server to port 80
AsyncMqttClient mqttClient;

Modbus modbusClient(0,0,0); // this is a client (meaning it commands other devices) and uses RS-232, not toggling RS485 which it uses anyway
// data array for modbus network sharing
uint16_t **registers;   // This is where the data retrieved from modbus devices is stored
uint16_t **last_registers;  // Used to compare values so only updates are send, not everything
uint8_t modbus_state;   // Current modbus state, from idle, to requesting, to receiving responses
uint8_t modbus_slave;   // Which slave are we talking to on modbus
unsigned long modbus_wait;  // Time between modbus requests

OneWire oneWire(D5);  // The DS18B20s are connected to D5 on the Wemos
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature ds18b20s(&oneWire);

// Global variables used throughout this program
bool pcf8574Found = false;  // There is a PCF8574 that we can read the status of the pins from
bool pcf8591Found = false;
char my_ssid[12];           // The SSID is generated from the last 3 bytes of this devices MAC address, so it can be used as a web server
uint32_t request_wait;      // This is used to control when polling takes place, and is a millisecond value.
uint32_t blast_time;        // When the last forced mqtt update took place.
uint32_t PCF8574_wait;      // The next time the PCF8574 was checked, since its on a different schedule than other devices
uint32_t PCF8591_wait;      // The next time the PCF8591 was checked, since its on a different schedule than other devices
uint32_t read_time;         // Only used to show on the web status page how long ago the readings were taken
String wifi_ssid;           // The SSID of the local guest network in the building
String wifi_password;       // The password for the above SSID
String WebName;             // The WebName variable has the name of the external web server used as a fallback for finding the mqtt address.
byte mac_address[8];        // The unique address of this device, used to tell who is sending data and as the SSID for web access.
IPAddress mqtt_host;        // This is used for the IP address of the mqtt server to connect to.
IPAddress apIP(192, 168, 4, 1);  // The local address of this device when accessed through the web interface.

float temperature[DS18B20_DEVICES];      // An array of up to DS18B20_DEVICES local DS18B20s can be connected.
uint8_t lastPCF8574Value;   // Used for comparing if any of the lines changed on the PCF8574
float lastPCF8591Value[4];   // Used only for comparing if any of the lines changed on the PCF8591

// QUEUE_WIDTH is the maximum size of an mqtt packet, and is a fixed length for this table
// QUEUE_MAX is the maximum number of entries that can be queued to be transmitted via mqtt
// QUEUE_ARRAY is the byte size of the queue table used for all of the queue entries.
// Note that the size of this table really impacts dynamic memory in the Wemos, and reduces space for local variables.
// Think carefully about the maximum number of queued entries.
#define QUEUE_WIDTH 128
// Warning - If QUEUE_MAX is too big, then the heap isn't big enough for OTA updates.  For example, 250 fails, but 200 works.
#define QUEUE_MAX 100
#define QUEUE_ARRAY QUEUE_WIDTH * QUEUE_MAX
// For the queue, malloc is used instead of reserving space here, or the heap just gets too big
char *queue;  // The global queue where all messages end up before being sent to the mqtt server
uint16_t queue_pos;
uint8_t queue_len;

// The eeprom information is used to store the connection information in the EEPROM on this device, so it can
// survive after a reboot or firmware update.
struct EEPROMStruct {
  char validate[6] = "";  // will have the word 'valid' when it is valid
  char ssid[20] = "";
  char password[20] = "";
  IPAddress mqtt = IPAddress(0, 0, 0, 0);
  char webname[64] = "";
  char debugging;
} eeprom_data;

uint8_t debugging_enabled;
// The next two variables are used for obtaining the time from a trusted external web page.
const char * headerKeys[] = {"date", "server"} ;
const size_t numberOfHeaders = 2;
// And this string is used for OTA updates of this program.
const char* serverIndex = "<form method='POST' action='/updateOTA' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";
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
// anyway.  The command line looks like "python3 espota.py -r -i 192.168.4.1 -f Zones.ino.bin",
// which will also report the progress of the copy (using the -r) to the PC.  The bin file
// would have been copied from the /tmp directory on the master machine.  Using a phone works great.
//========================================================================================
//========================================================================================
// A small procedure to try to clean up memory leaks
//========================================================================================
struct tcp_pcb;
extern struct tcp_pcb* tcp_tw_pcbs;
extern "C" void tcp_abort (struct tcp_pcb* pcb);
void tcpCleanup ()
{
  while (tcp_tw_pcbs != NULL)
  {
    tcp_abort(tcp_tw_pcbs);
  }
}
// Trying to clean up issues with the hardware watchdog kicking in
void hw_wdt_disable(){
  *((volatile uint32_t*) 0x60000900) &= ~(1); // Hardware WDT OFF
}

void hw_wdt_enable(){
  *((volatile uint32_t*) 0x60000900) |= 1; // Hardware WDT ON
}
void hw_wdt_reset(){
 *((volatile uint32_t*) 0x60000914) = 0x73; 
}
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
  uint8_t sensor_address[8];

  WiFi.macAddress(mac_address);     // get the mac address of this chip to make it unique in the building
  sprintf(my_ssid,"%02X%02X%02X",mac_address[3],mac_address[4],mac_address[5]); // Use the hex version of the MAC address as our SSID
  WiFi.softAP(my_ssid, "87654321");             // Start the access point with password of 87654321
  WiFi.persistent(false); // Only change the current in-memory Wi-Fi settings, and does not affect the Wi-Fi settings stored in flash memory.

  Serial.begin(9600);     // All serial communications are at 9600,which is fine for what we are doing.
  while (!Serial) {}      // Wait for OK
  Serial.setRxBufferSize(256);    // crank up the size of the RX buffer
  pinMode(D6,INPUT_PULLUP); // used for 1-wire

  // Allocate memory for the queue at this time.  The queue is a global variable, access by multiple procedures.
  queue_pos = 0;
  queue_len = 0;
  queue = (char *)malloc(QUEUE_ARRAY * sizeof(char));
  if (queue == NULL)
  {
    Serial.print("Failed to allocate the queue.\r\n");
  }
  add_to_queue((char *)"R|000|Booting........");  // Let the other end know we had to boot.

  // Allocate memory for the modbus registers.  This is the same as uint16_t registers[MODBUS_DEVICES][MODBUS_REGISTERS];
  registers = (uint16_t **)malloc(sizeof(uint16_t *) * MODBUS_DEVICES); 
  last_registers = (uint16_t **)malloc(sizeof(uint16_t *) * MODBUS_DEVICES); 
  for (i=0; i < MODBUS_DEVICES; i++)
  {
    registers[i] = (uint16_t *)malloc(MODBUS_REGISTERS * sizeof(uint16_t)); 
    last_registers[i] = (uint16_t *)malloc(MODBUS_REGISTERS * sizeof(uint16_t)); 
  }
  // reset the holding registers to 0 to make sure they start clean
  for (j = 0; j < MODBUS_DEVICES; j++)
    for (i = 0; i < MODBUS_REGISTERS; i++)
    {
      registers[j][i] = 0;
      last_registers[j][i] = 0;   // used for comparing current to previous to limit updates
    }

  ESP.wdtFeed();
  EEPROM.begin(512);  // try to get the connection info from EEPROM
  EEPROM.get(0,eeprom_data);
  if (!strcmp(eeprom_data.validate,"valid"))  // Is there a valid entry written in the EEPROM of this device
  {
    wifi_ssid = String(eeprom_data.ssid);
    wifi_password = String(eeprom_data.password);
    mqtt_host = eeprom_data.mqtt;
    WebName = String(eeprom_data.webname);
    debugging_enabled = eeprom_data.debugging;
  }
  else  // Default to the defines if the EEPROM has not yet been programmed.
  {
    wifi_ssid = WIFI_SSID;
    wifi_password = WIFI_PASSWORD;
    mqtt_host = MQTT_HOST;
    WebName = WEBNAME;
    debugging_enabled = 0;
  }
  EEPROM.end();
  if (!ScanForWifi()) // This is a fallback because constantly remembering to change the EEPROM is a pain.
  {
    wifi_ssid = ALT_WIFI_SSID;
    wifi_password = ALT_WIFI_PASSWORD;
    mqtt_host = ALT_MQTT_HOST;
  }
  mqttClient.setServer(mqtt_host, MQTT_PORT); // Set the name of the mqtt server.

  // Setup what to do with the various web pages used only for configuring/testing this device
  server.on(F("/config"), handleConfig); // if we are told to change the configuration, jump to that web page
  server.on(F("/submit"), handleSubmit);
  server.on(F("/"), handleStatus);       // The most basic web request returns the status information

  // The update code is for doing OTA updates of this program.  Only /update is called by users.  The /updateOTA
  // code is called in the background, never directly by the users.
  server.on(F("/update"), HTTP_GET, []() 
  {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  server.on(F("/updateOTA"), HTTP_POST, []() 
  {
    server.sendHeader(F("Connection"), "close");
    server.send(200, F("text/plain"), (Update.hasError()) ? "FAIL" : "OK");
    delay(1000);    // Show it for 1 second, then restart the ESP8266 using a software command
    ESP.restart();
  }, []() 
  {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) 
    {
      uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
      Update.begin(maxSketchSpace);
    } 
    else 
      if (upload.status == UPLOAD_FILE_WRITE) 
        Update.write(upload.buf, upload.currentSize);
      else 
        if (upload.status == UPLOAD_FILE_END)
          Update.end(true);
    delay(1);
  }); 
  server.begin();   // Start listening for connections to this devices AP.  Most of the time it's not needed, but helpful...
  ESP.wdtFeed();
  ds18b20s.begin();  // Start the sensors

  Wire.setClock(100000L);   // Only use a 100k clock frequency, the PCF8591 can't handle a higher speed
  Wire.begin();
  Wire.beginTransmission (PCF8574_I2C);  // The address of the PCF8574
  if (Wire.endTransmission () == 0)
  {
    pcf8574Found = true;   // We did find a PCF8574 on the I2C bus, and will use it to read external binary events
    lastPCF8574Value = 0;  // this forces a transmit on the first pass
    Wire.beginTransmission(PCF8574_I2C);
    Wire.write(0xff);
    Wire.endTransmission();
  }
  ESP.wdtFeed();  
  delay(50);
  Wire.beginTransmission(PCF8591_I2C); // The address of the PCF8591
  if (Wire.endTransmission () == 0)
  {
    pcf8591Found = true;   // We did find a PCF8591 on the I2C bus, and will use it to read external analog voltages
    lastPCF8591Value[0] = 6.1;  // this forces a transmit on the first pass
    lastPCF8591Value[1] = 6.1;
    lastPCF8591Value[2] = 6.1;
    lastPCF8591Value[3] = 6.1;
  }
  PCF8574_wait = millis() + PCF8574_INTERVAL;  // Check in 15 seconds
  PCF8591_wait = millis() + PCF8591_INTERVAL + (PCF8574_INTERVAL / 2);  // Offset this interval from the PCF8574
  
  getTimeFromHttp();  // get the time
  last_hour = 25;     // Use an invalid number just to initialize it.
  
  // This next chunk of code just gets the DS18B20s past the default 85C stage
  ds18b20s.requestTemperatures();
  oneWire.reset_search();
  j = 0;
  if (oneWire.search(sensor_address))
  {
    do
    {
      ds18b20s.getTempC(sensor_address);  // The temperature isn't needed, it just has to be read during setup
      j++;
      delay(1);
    } while (oneWire.search(sensor_address));
  }
  for (i = 0; i < DS18B20_DEVICES; i++)    // temperature variable is just used for DS18B20s, allowing for DS18B20_DEVICES of them on this device
    temperature[i] = 0.0;

  modbusClient.begin( 9600 ); // begin the ModBus object.
  modbusClient.setTimeOut( 2000 ); // if there is no answer in 2 seconds, roll over to the next
  modbus_wait = millis() + 60000;   // wait 60 seconds from now before doing the first poll over modbus
  modbus_state = 0;
  modbus_slave = 0;
  request_wait = millis() + TIME_BETWEEN_POLLING + 30000;   // offset the normal requests by an additional 30 seconds
  blast_time = millis() + 120000;  // wait 2 minutes, then send everthing
  read_time = millis();   // Realistically, the time since the first reboot (initially)
  time_offset = mac_address[5] * 128;   // Create a random interval for this device, based on its MAC address

  // This is the end of the setup routine.  Posting the VERSION helps identify that everything really is at the same version throughout the building.
  sprintf(ts,"R|%03ld|Booted V.%s",(millis() + 499) / 1000,VERSION);
  add_to_queue(ts);   // tell the mqtt server that this device is up and running.  It isn't needed, but helpful.

  delay(100 + time_offset);     // Do a random delay at this time to ensure that not everything is read at the same time by multiple devices.
                                // Note that this can add up to 32 seconds to the boot time, which is generally 6 seconds without this delay.
  ESP.wdtFeed();
  mqttClient.setClientId(my_ssid);    // Used to keep things runnng with QOS 2

  if (debugging_enabled)
  {
    char ts1[45];
    uint32_t heap_free;
    uint16_t heap_max;
    uint8_t heap_frag;
    sprintf(ts,"R|000|Boot reason was:%s",ESP.getResetReason().c_str());
    add_to_queue(ts);
    sprintf(ts1,"R|000|Core Version %s",ESP.getCoreVersion().c_str());
    add_to_queue(ts1);
    sprintf(ts1,"R|000|SDK Version %s",ESP.getSdkVersion());
    add_to_queue(ts1);
    sprintf(ts1,"R|000|Boot Version %d",ESP.getBootVersion());
    add_to_queue(ts1);
    sprintf(ts1,"R|000|Boot mode %d",ESP.getBootMode());
    add_to_queue(ts1);
    sprintf(ts1,"R|000|CPU Freq %dMhz",ESP.getCpuFreqMHz());
    add_to_queue(ts1);
    sprintf(ts1,"R|000|VCC is %4.2f",((double)ESP.getVcc()/1000) * 1.1);
    add_to_queue(ts1);
    ESP.getHeapStats(&heap_free, &heap_max, &heap_frag);
    sprintf(ts1,"R|000|Free heap is %zu",heap_free);
    add_to_queue(ts1);
    sprintf(ts1,"R|000|Heap fragmentation is %d%%",heap_frag);
    add_to_queue(ts1);
    sprintf(ts1,"R|000|MaxFreeBlockSize is %d",heap_max);
    add_to_queue(ts1);
    sprintf(ts1,"R|000|getFreeContStack is %zu",ESP.getFreeContStack());
    add_to_queue(ts1);
  }
  xmit_the_queue();   // xmit_the_queue() actually connects to the network, connects to the mqtt server, and sends the queued up data.
}

//========================================================================================
// Pull the date and time using an HTTP request.  This is because the NTP port is blocked
// on the guest network that this system runs on.  It is annoying, but easier just to use
// the method of getting the time from a trusted web page.  This procedure will in turn
// call the timeElements() procedure with the timestring it got, which will compensate for
// the location and daylight savings.
// The returned value should be the time in seconds from 1970.  Or 0 if something went wrong.
//========================================================================================
time_t getTimeFromHttp() 
{
  int httpCode;
  String headerDate;
  time_t tt;

  tt = 0;
  if (debugging_enabled)
    add_to_queue((char *)"R|000|Getting the time");
  connectToWifi();    // Connect to whatever guest network we are supposed to be on.
  if (WiFi.status() != WL_CONNECTED)  // If we couldn't connect through WiFi, then there's no point in continuing.
  {
    WiFi.disconnect();
    return 0;
  }
  ESP.wdtFeed();
  http.begin(client,HTTP_TIME_SOURCE);   // A generally reliable place to get the date and time
  http.setReuse(false);                  // Don't try to hang onto old web pages
  http.collectHeaders(headerKeys, numberOfHeaders);   // Just look at the headers
  httpCode = http.GET();
  ESP.wdtFeed();
  if (httpCode > 0)
  {
    headerDate.reserve(35);
    headerDate = http.header("date"); // We only want the date and time from the headers
    // headerDate looks like Sat, 19 Oct 2019 06:29:57 GMT
    tt = timeElements((char *)headerDate.c_str());   // set the date and time using what we got from the http request
    if (debugging_enabled)
      add_to_queue((char *)"R|000|Got the time");
  }
  http.end();   // We are done with our HTTP request for about 24 hours
  delay(100);   // The delays are necessary
  WiFi.disconnect();
  delay(100);
  tcpCleanup(); // Close any open connections
  return tt;
}

//========================================================================================
// Parse the printable version of the date and time we got through HTTP into the appropriate
// format for setting the date and time within the Arduino program.  This takes a string
// value from an HTTP header.
//========================================================================================
time_t timeElements(char *str)
{
  tmElements_t tm;
  int Year, Month, Day, Hour, Minute, Seconds, Wday ;
  int j;
  // An example of a time string to convert is - Sat, 19 Oct 2019 06:29:57 GMT
  char delimiters[] = " :,";    // Used to separate the date and time fields
  char* valPosition;
  Year = 1970;    // Set some base default values
  Month = 1;
  Day = 1;
  Hour = 0;
  Minute = 0;
  Seconds = 0;
  Wday = 1;

//  Timezone myTZ(myDST, mySTD);
  valPosition = strtok(str, delimiters);
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

  ESP.wdtFeed();
  setTime(makeTime(tm));    // Set the system time to UTC
  //setTime(myTZ.toLocal(makeTime(tm)));  // Call the myTZ class to convert the date and time to the correct timezone, including daylight savings.
  return makeTime(tm);
}

//========================================================================================
// Search for the desired access point, and if it is found, return 1
//========================================================================================
uint8_t ScanForWifi()
{
  int n = WiFi.scanNetworks();  // Do a simple network check to see what's available.
  if (!n)   // n is the number of APs found
  {
    ; // No wifi networks were found, which can happen in the tunnels.
    return(0);
  }
  else
  {
    for (int i = 0; i < n; ++i)
      if (wifi_ssid == WiFi.SSID(i))  // Check to see if the SSID we are supposed to connect to has been found by the scan
        return(1);
  }
  return(0);
}

//========================================================================================
// Connect to the local guest network for long enough to either get the time, or to transmit
// a mqtt packet to the server.  Realistically, this program does a connection about every
// 30 minutes, maybe more often if it is monitoring something that changes frequently.
//========================================================================================
void connectToWifi()
{
  uint8_t i;
  if (WiFi.status() == WL_CONNECTED)
    return;    // No point in connecting if we already are...
  delay(50);

  WiFi.begin(wifi_ssid, wifi_password);
  i = 0;
  while (WiFi.status() != WL_CONNECTED && i < 240)   // Need to keep looking for about 15 seconds, because, yes, it can take that long to connect
  {
    i++;
    delay(62);
    ESP.wdtFeed();
  }

  delay(100);
}

//========================================================================================
// The getNewMqtt() procedure connects to the guest network, then gets the address of the
// mqtt server from a well known (to this program) website, just in case it changed.  Normally
// it wouldn't change, but because everything is running on the guest wifi network, anything
// could change at any time.
// While this section of the code works, at the time of the writing the hosting website
// dropped supporting the page, so there is nothing currently in place actually using this.
//========================================================================================
void getNewMqtt()   // gets a new mqtt server address if it changes by checking a web site
{

  if (WebName.equalsIgnoreCase("X"))
    return;   // Nothing to get an address from, so return
  connectToWifi();
  if (WiFi.status() != WL_CONNECTED)  // If we couldn't connect through WiFi, then there's no point in continuing.
  {
    WiFi.disconnect();
    return;
  }

  delay(1);
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
        IPAddress ip;
        ip.fromString(payload);
        if (ip[0] == 0 || ip[3] == 0) // is it a valid ip address?  Starting and ending with 0 generally aren't valid.
        {
          http.end();
          delay(500);   // this is important.  Don't remove this delay.  Things go bad otherwise.
          WiFi.disconnect();
          return;
        }
        if (mqtt_host != ip)  // check to see if the ip address changed
        {
          char ts[64];
          sprintf(ts,"R|000|Changing the mqtt address to be %d.%d.%d.%d",ip[0], ip[1], ip[2], ip[3]);
          add_to_queue(ts);
          xmit_the_queue(); // Actually tell the old address
          mqtt_host = ip;   // From this point forward the address is changed.
          mqttClient.setServer(mqtt_host, MQTT_PORT);
          strcpy(eeprom_data.validate,"valid");
          wifi_ssid.toCharArray(eeprom_data.ssid,wifi_ssid.length() + 1);
          wifi_password.toCharArray(eeprom_data.password,wifi_password.length() + 1);
          eeprom_data.mqtt = IPAddress(ip[0], ip[1], ip[2], ip[3]);
          WebName.toCharArray(eeprom_data.webname,WebName.length() + 1);
          EEPROM.begin(512);
          EEPROM.put(0,eeprom_data);
          EEPROM.commit();
          EEPROM.end();
          sprintf(ts,"R|000|EEPROM mqtt address is now %d.%d.%d.%d",ip[0], ip[1], ip[2], ip[3]);
          add_to_queue(ts);
        }
      }
    } 
    else
      add_to_queue((char *)"E|[HTTP] GET... failed");
  }
  http.end();
  delay(500);   // this is important.  Don't remove this delay
  WiFi.disconnect();
  delay(250);
  tcpCleanup();
}

//========================================================================================
// The add_to_queue() procedure adds a character string to the mqtt queue.  As part of that
// process, it will also timestamp and format the added line.  It doesn't send anything,
// it just queues it internally to the Wemos.
//========================================================================================
void add_to_queue(char* str)
{
  // Everything added to the queue has the same prefix and suffix
  // The my_ssid part is simply diagnostics, except in the case of the boiler monitor.
  if (strlen(str) > 90) // Just to keep the string within the required length
    return;
  if (queue_len < (QUEUE_MAX - 1))  // add to the queue if there is room
  {
    Timezone myTZ(myDST, mySTD);
    time_t utc = now();
    time_t t = myTZ.toLocal(utc);
    sprintf(queue+queue_pos, "%-6s|%04d-%02d-%02d %02d:%02d:%02d|%s|-|", my_ssid, year(t), month(t), day(t), hour(t), minute(t), second(t),str);
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
  uint8_t j;
  uint16_t queue_position;

  if (!queue_len)  // no sense connecting if there's nothing to send
    return;
  connectToWifi();
  if (WiFi.status() != WL_CONNECTED)
    return;
  ESP.wdtFeed();
  mqttClient.connect();   // Get connected to the mqtt server
  j = 0;
  while (!mqttClient.connected() && j < 200)  // give it up to 4 seconds to connect to mqtt
  {
    delay(25);
    j++;
  }
  if (mqttClient.connected() && queue_len)
  {
    ESP.wdtFeed();
    queue_position = 0; // Used for getting info out of the queue FIFO, versus pulling from the end
    do
    {
      queue_len--;    // queue_len is the number of entries in the queue
      if (strlen(queue + queue_position) > 0)
      {
        // Message is being published with the 'Clean' session under QOS 2.
        mqttClient.publish(MQTT_NAME, 2, true, queue + queue_position);  // topic, qos, retain, payload, length=0, dup=false, message_id=0
        delay(250);   // We do have to wait for it to clear
      }
      queue_position += QUEUE_WIDTH;
    } while (queue_len > 0);
    queue_pos = 0;    // reset the queue_pos for the next entries to be added to the queue in the future
    mqttClient.disconnect();
  };
  ESP.wdtFeed();
  delay(500);   // this is important.  Don't remove this delay
  WiFi.disconnect();
  delay(250);
  tcpCleanup();
}

//========================================================================================
// This simple function compares two values to see if they are different by more than maxDiff
// Technically it could probably be done with a macro, but this is probably easier.
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
  int i;

  server.handleClient();    // used to handle configuration changes through the web interface
  if (hour() != last_hour && minute() > 56)   // At the end of each hour, do nothing for 3 minutes
  {
    ESP.wdtFeed();
    return;   // Just loop around for the server side of things for three minutes
  }           // Don't even check the PCF9574, as much as it hurts not to.

  switch( modbus_state )
  {
    case 0:
      if (millis() > modbus_wait)
      {
        modbus_state++; // wait state
        read_time = millis();
      }
      break;
    case 1:
      ESP.wdtFeed();
      for (i = 0; i < MODBUS_REGISTERS; i++)
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
      ESP.wdtFeed();
      modbusClient.poll(); // check incoming messages
      if (modbusClient.getState() == COM_IDLE)  // COM_IDLE is after we've received our response
      {
        process_holding_registers(0);    // this queues up the data for mqtt
        modbus_state = 0;
        if (modbus_slave > (MODBUS_DEVICES - 1))
        {
          modbus_wait = millis() + TIME_BETWEEN_POLLING + time_offset;  // reset for the next loop
          modbus_slave = 0;
        }
      }
      break;
  }  // end of waking up on modbus_state

  // if we aren't looking from a response from a remote device, then it's ok to do other stuff
  if (modbus_state == 0)
  {
    // every 15 seconds check the pcf9574.
    if (millis() > PCF8574_wait && pcf8574Found)
    {
      PCF8574_wait = millis() + PCF8574_INTERVAL;  // reset to check in another 15 seconds
      read_pcf8574_pins(0);   // Don't do a forced read
      xmit_the_queue();   // if anything is queued up, send it off
    }
    if (millis() > PCF8591_wait && pcf8591Found)
    {
      PCF8591_wait = millis() + PCF8591_INTERVAL;  // Check in another minute
      read_pcf8591(0);   // Don't do a forced read
      xmit_the_queue();   // if anything is queued up, send it off
    }

    if (millis() > blast_time)    // Send everthing we've got
    {
      request_wait = millis() + TIME_BETWEEN_POLLING + time_offset;
      blast_time = millis() + BLAST_INTERVAL + time_offset;
      read_time = millis();
      // modbus_wait is not affected by this loop.  That is set separately and occurs at TIME_BETWEEN_POLLING intervals regardless of the reporting intervals.

      get_local_ds18b20s(1);    // force a read of all the ds18b20s
      if (pcf8574Found)
        read_pcf8574_pins(1);    // this is a forced read, so we send the pins regardless of anything changing
      PCF8574_wait = millis() + PCF8574_INTERVAL;
      if (pcf8591Found)
        read_pcf8591(1);    // this is a forced read, so we send the analog voltages regardless of anything changing
      PCF8591_wait = millis() + PCF8591_INTERVAL;
      process_holding_registers(1);    // this queues up the data for mqtt by forcing an update

      if (debugging_enabled)
      {
        char ts1[45];
        uint32_t heap_free;
        uint16_t heap_max;
        uint8_t heap_frag;
        ESP.wdtFeed();
        ESP.getHeapStats(&heap_free, &heap_max, &heap_frag);
        add_to_queue((char *)"R|000|______________________");
        sprintf(ts1,"R|000|Free heap is %zu",heap_free);
        add_to_queue(ts1);
        sprintf(ts1,"R|000|Heap fragmentation is %d%%",heap_frag);
        add_to_queue(ts1);
        sprintf(ts1,"R|000|MaxFreeBlockSize is %d",heap_max);
        add_to_queue(ts1);
        sprintf(ts1,"R|000|getFreeContStack is %zu",ESP.getFreeContStack());
        add_to_queue(ts1);
      }

      xmit_the_queue();   // send all of the queued data to the mqtt server
    }

    if (millis() > request_wait)    // Just send what has changed since the last blast
    {
      request_wait = millis() + TIME_BETWEEN_POLLING + time_offset;
      read_time = millis();
      get_local_ds18b20s(0); // get the local stuff
      // process the holding registers from the polled devices
      process_holding_registers(0);   // don't force an update
      xmit_the_queue();   // if anything is queued up, send it off
    }
    
    // Things get interesting at 4am-ish.  It would be 2am, but then I'd have to deal with daylight savings at the same time
    time_t utc = now();
    time_t t = myTZ.toLocal(utc);
    if (hour(t) == 4 && minute(t) > 20 && !time_was_set) // Get the time at 4am-ish, so we aren't too far off as time passes
  	{  // The time_was_set variable is used so we only get the time once a day.
      delay(time_offset * 2);
  	  if (!getTimeFromHttp())  // this gets the time
      {
        add_to_queue((char *)"E|000|Failed to get the time from HTTP");
        xmit_the_queue();
        delay(5000);
        ESP.restart();    // reboot because something is seriously wrong
      }
  	  time_was_set = true;  // say we got the 4am time so we don't loop around and try again immediately
  	}
  	if (queue_len > (QUEUE_MAX * .75) || (hour(t) < 1 && minute(t) > 30 && millis() > MILLIS_MAX)) // Reboot every two or three days, due to a weird connection problem with the local network
    { // Two or three days means "only boot between 12:30am and 1am", which can stretch things out a bit.
      add_to_queue((char *)"Booting due to connection issues");
      xmit_the_queue(); // flush out any hanging queued entries
      delay(5000);     // make sure everything clears, then reboot
      ESP.restart();    // As part of the restart, millis is reset to 0.
    }
    if (hour(t) > 4)
	    time_was_set = false; // after 4am it is ok to reset this flag.
     
  }  // end of if (modbus_state == 0)
}

//========================================================================================
//========================================================================================
void process_holding_registers(uint8_t force)
{
  char temp_message[32];
  uint8_t i,j,k;    
  float newTemp;

  // Note that modbus doesn't really handle negative values that well.  Everything is a positive integer.
  for (j = 0; j < MODBUS_DEVICES; j++)  // for each unit that is polled we check the registers
  {
    k = 0;
    for (i = 0; i < MODBUS_REGISTERS; i++)
      if (registers[j][i] > 0)
        k = 1;    // all k does is indicate that there is a value other than 0 in one of the fields
    if (k)  // we got something from a device cause not everything is 0
    {
      for (i = 0; i < 6; i++)  // there are up to six ds18b20s on each remote device, that is a fixed value on the remote
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
// All printBinary does is convert the bits in inByte to a string of 1's and 0's in ts.
// It may seem a bit backward, because if a pin is LOW, then it prints a 1, and if the pin
// is left high, then it prints a 0.  Pins float HIGH, so if they are pulled to ground, then
// that means whatever switch, relay, or contact on the far end is active, thus the LOW.
//========================================================================================
void printBinary(byte inByte, char *ts)
{
  // if a pin is grounded (pulled low), then it is active, hence the '1'.  Pins left floating are low, the '0'
  for (int b = 7; b >= 0; b--)  // Print the bits in MSB order.
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
  char ts[9];
  uint8_t val;

  if (!pcf8574Found)
    return;
  Wire.requestFrom(PCF8574_I2C, 1);
  val = Wire.read();  // read all pins to see if any were pulled low
  if (val != lastPCF8574Value || forced)   // forced reads ignore whether the data has changed or not
  {
    char pcf8574_message[16];
    printBinary((byte)val, ts);
    ts[8] = 0;
    sprintf(pcf8574_message, "P|000|%s", ts);
    add_to_queue(pcf8574_message);  // send to the queue to be sent to mqtt
    lastPCF8574Value = val;
  }
}
//========================================================================================
// This procedure reads the ADC values from the PCF8591.  Up to 4 ADCs are supported, using
// an upper limit of 5V.  If the PCF8591 is found during setup, this will be read about once
// a minute by the main loop.  Only if the value changes by at least 0.2 volts will anything
// be reported.
// Note that the PCF8591 only has an 8-bit result, so only about 0.02 volts of gradiation
// are possible.  We don't need to go down that low.
//========================================================================================
void read_pcf8591(uint8_t forced)
{
  if (!pcf8591Found)
    return;

  char pcf8591_message[24];
  float adcvalue;

  Wire.beginTransmission(PCF8591_I2C);
  Wire.write(0x04); // Request a sequencial read of one pin after the latter
  Wire.endTransmission();
  Wire.requestFrom(PCF8591_I2C, 5);   // Read all of the ADC pins sequencially
  Wire.read();  // The first byte transmitted in a read cycle contains the conversion result code of the previous read cycle.
  delay(5);   //  Give it time to do all the ADC conversions, which mostly happen during the last read

  for (uint8_t i = 0; i < 4; i++)   // Loop through each of the ADCs
  {
    adcvalue=(Wire.read() * 5.0) / 256.0;   // Get the ADC value
    if (checkDiff(adcvalue,lastPCF8591Value[i],0.2) || forced)   // forced reads ignore whether the data has changed or not
    {
      sprintf(pcf8591_message, "A|000|%06.2f|%d", adcvalue,i);
      add_to_queue(pcf8591_message);  // send to the queue to be sent to mqtt
      lastPCF8591Value[i] = adcvalue;
    }
    delay(5); // Pause briefly between each read to give it time to convert the next value
  }
}
//========================================================================================
// Read the temperatures from any DS18B20s found connected to this device.  If it is there,
// then it is there, otherwise just keep going.  Any DS18B20 found is identified by it's 
// unique address (hardcoded within the sensor), and that information is passed along to 
// the main processing program (elsewhere).  Temperatures are always read and transmitted
// in Celsius, for consistency with other sensor types.
//========================================================================================
void get_local_ds18b20s(uint8_t forced)
{
  char temp_message[32];
  uint8_t sensor_address[9];
  uint8_t sensor_index;
  float newTemp;

  // We can do negative temperatures with the local DS18B20s, because we are not using modbus for them.
  // ------- Start of 1-wire section to read DS18B20s connected to this device -------
  // This is pretty much standard logic for searching for, and reading each DS18B20 on the wire.
  ds18b20s.requestTemperatures();
  oneWire.reset_search();
  delay(1);
  sensor_index = 0;
  if (oneWire.search(sensor_address))
  {
    do
    {
      newTemp = ds18b20s.getTempC(sensor_address);  // Get the temperature from the sensor in Celsius, as it's more standard 
      if (checkDiff(newTemp, temperature[sensor_index], 0.55) || forced)  // Only report if it has changed or if this is a forced read
      {
        temperature[sensor_index] = newTemp;
        sprintf(temp_message, "T|000|%06.2f|%02x%02x%02x%02x%02x%02x", temperature[sensor_index],
          sensor_address[2],sensor_address[3],sensor_address[4],sensor_address[5],sensor_address[6],sensor_address[7]);
        add_to_queue(temp_message);  // send to the queue to be sent to mqtt
      }
      sensor_index++;
      delay(1);
    } while (oneWire.search(sensor_address) && sensor_index < DS18B20_DEVICES);
  }
  // ------ End of processing for 1-wire DS18B20 temperatures
}

//========================================================================================
// The web_modbus() routine is used by the web interface to force a read of the modbus
// devices.  That is its only usage.
//========================================================================================
void web_modbus()
{
  char temp_message[32];
  uint8_t state;
  uint8_t slave;
  uint8_t i;
  
  state = 1;
  slave = 0;
  do
  {
    switch( state )
    {
      case 0:
        server.sendContent("Finished polling modbus<br>");  // print the message to the web page
        return;
      case 1:
        delay(1);
        for (i = 0; i < MODBUS_REGISTERS; i++)  // Reset the registers to 0
          registers[slave][i] = 0;
        telegram.u8id = 0x30 + slave;           // The slave address is printable in this case
        telegram.u8fct = 3;                     // function code (this one is registers read)
        telegram.u16RegAdd = 0;                 // starting register address in slave
        telegram.u16CoilsNo = MODBUS_REGISTERS; // number of elements (coils or registers) to read
        telegram.au16reg = &registers[slave][0]; // pointer to a memory array in the Arduino
        sprintf(temp_message,"Polling device %c<br>",slave + 0x30);
        server.sendContent(temp_message);
        modbusClient.query( telegram );         // send query to the slave (only once)
        state++;                                // Change the status to show that we're now looking for responses
        slave++;                                // set up for the next slave
        break;
    case 2:
      delay(1);
      modbusClient.poll();                      // check incoming messages from the slave
      if (modbusClient.getState() == COM_IDLE)  // COM_IDLE is after we've received our response
      {
        if (slave > (MODBUS_DEVICES - 1))
        {
          state = 0;
          server.sendContent(F("Processing holding registers...<br>"));
          web_process_holding_registers();      // this prints the holding registers to the web page
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
  char temp_message[32];
  uint8_t i,j,k;    
  float newTemp;

  for (j = 0; j < MODBUS_DEVICES; j++)  // for each unit that is polled we check the registers
  {
    k = 0;
    for (i = 0; i < MODBUS_REGISTERS; i++)
      if (registers[j][i] > 0)
        k = 1;    // all k does is indicate that there is a value other than 0 in one of the fields
    if (k)  // we got something from a device cause not everything is 0
    {
      delay(1);
      for (i = 0; i < 6; i++)  // there are up to six ds18b20s on each remote device
      {
        if (registers[j][i*4] > 0)  // If this DS18B20 has a temperature that isn't 0, then print it
        {
          newTemp = float(registers[j][i * 4] / 100.0);
          sprintf(temp_message, "T|%c|%03d|%06.2f|%04x%04x%04x<br>",0x30 + j, registers[j][25],newTemp,registers[j][(i*4)+1], registers[j][(i*4)+2], registers[j][(i*4)+3]);
          server.sendContent(temp_message);  // send to the web page
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
  }
}

//========================================================================================
// handleConfig() is a procedure called by the web server.  It allows for changes to the
// network, the address of the mqtt server, and the public name of the web site that also
// has the mqtt server address.  Without this procedure the device will never connect to
// the local wifi network, unless everything was setup properly in eeprom.
//========================================================================================
void handleConfig() 
{
  char ts[80];
  time_t utc = now();
  time_t t = myTZ.toLocal(utc);
  // note that embedded in this form are lots of variable fields pre-filled with the sprintf
  ESP.wdtFeed();
  server.sendHeader(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
  server.sendHeader(F("Pragma"), F("no-cache"));
  server.sendHeader(F("Expires"), "-1");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  // here begin chunked transfer
  server.send(200, "text/html");
  server.sendContent(F("<!DOCTYPE HTML><html><head><meta content=\"text/html; charset=ISO-8859-1\" http-equiv=\"content-type\">" \
      "<meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0\">"));
  server.sendContent(F("<title>Configuration</title><style>\"body { background-color: #808080; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }\"</style></head>"));
  server.sendContent(F("<body><h1>Configuration</h1><FORM action=\"/submit\" method=\"post\">"));
  sprintf(ts,"<P><label>SSID:&nbsp;</label><input maxlength=\"30\" value=\"%s\" name=\"ssid\"><br>",wifi_ssid.c_str());
  server.sendContent(ts);
  sprintf(ts,"<label>Password:&nbsp;</label><input maxlength=\"30\" value=\"%s\" name=\"Password\"><br>",wifi_password.c_str());
  server.sendContent(ts);
  sprintf(ts,"<label>MQTT IP:&nbsp;</label><input maxlength=\"15\" value=\"%d.%d.%d.%d\" name=\"MQTT_IP\"><br> ",mqtt_host[0],mqtt_host[1],mqtt_host[2],mqtt_host[3]);
  server.sendContent(ts);
  sprintf(ts,"<label>HTTP Web Name:&nbsp;</label><input maxlength=\"63\" value=\"%s\" name=\"WebName\"><br> ",WebName.c_str());
  server.sendContent(ts);
  if (debugging_enabled)
    server.sendContent(F("<label>Enable debugging&nbsp;</label><input type=\"checkbox\" id=\"debugging\" name=\"enable_debug\" value=\"Debug\" checked=\"checked\"><br>"));
  else
    server.sendContent(F("<label>Enable debugging&nbsp;</label><input type=\"checkbox\" id=\"debugging\" name=\"enable_debug\" value=\"Debug\"><br>"));
  server.sendContent(F("<INPUT type=\"submit\" value=\"Send\"> <INPUT type=\"reset\"></P>"));
  sprintf(ts,"<P><br>The time is %04d-%02d-%02d %02d:%02d:%02d</P></FORM></body></html>",year(t), month(t), day(t), hour(t), minute(t), second(t));
  server.sendContent(ts);
  server.sendContent(""); // this closes out the send
  ESP.wdtFeed();
  server.client().stop();
}

//========================================================================================
// Report on the status of everthing through the web interface, mainly from 192.168.4.1.
// This routine is very helpfull for ensuring everthing is setup properly.
//========================================================================================
void handleStatus()
{
  char ts[72];
  uint16_t queue_length;
  uint16_t queue_position;
  int16_t last_read = (millis() - read_time) / 1000;
  char pcf8574_vals[9];
  uint8_t val;
  float adcvalue;
  uint8_t sensor_address[8];
  time_t utc = now();
  time_t t = myTZ.toLocal(utc);

  ESP.wdtFeed();
  modbus_state = 4;   // This is so we are exclusive while the web page is being processed
  server.sendHeader(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
  server.sendHeader(F("Pragma"), F("no-cache"));
  server.sendHeader(F("Expires"), F("-1"));
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  // here begin chunked transfer a line at a time
  server.send(200, "text/html");
  ESP.wdtFeed();
  server.sendContent(F("<!DOCTYPE HTML><html><head><meta content=\"text/html; charset=ISO-8859-1\" http-equiv=\"content-type\">" \
      "<meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0\">"));
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
  sprintf(ts,"Boot reason was:%s<br>",ESP.getResetReason().c_str());
  server.sendContent(ts);
  sprintf(ts,"Core Version %s<br>",ESP.getCoreVersion().c_str());
  server.sendContent(ts);
  sprintf(ts,"SDK Version %s<br>",ESP.getSdkVersion());
  server.sendContent(ts);
  sprintf(ts,"Boot Version %d<br>",ESP.getBootVersion());
  server.sendContent(ts);
  sprintf(ts,"Boot mode %d<br>",ESP.getBootMode());
  server.sendContent(ts);
  sprintf(ts,"CPU Freq %dMhz<br>",ESP.getCpuFreqMHz()); // Yes, I really don't need this, but since I'm here anyway...
  server.sendContent(ts);
  sprintf(ts,"VCC is %4.2f<br>",((double)ESP.getVcc()/1000) * 1.1);
  server.sendContent(ts);
  sprintf(ts,"Free heap is currently %zu<br>",ESP.getFreeHeap());
  server.sendContent(ts);
  sprintf(ts,"Heap fragmentation is %d<br>",ESP.getHeapFragmentation());
  server.sendContent(ts);
  sprintf(ts,"MaxFreeBlockSize is %zu<br>",ESP.getMaxFreeBlockSize());
  server.sendContent(ts);
  sprintf(ts,"I think the time is %04d-%02d-%02d %02d:%02d:%02d</P>", year(t), month(t), day(t), hour(t), minute(t), second(t));
  server.sendContent(ts);
  ESP.wdtFeed();
  if (pcf8574Found)
  {
    Wire.requestFrom(PCF8574_I2C, 1);
    val = Wire.read();  // read all pins to see if any were pulled low
    printBinary((byte)val, pcf8574_vals); // Convert to a string
    pcf8574_vals[8] = 0;
    sprintf(ts,"Pins are :%s<br>",pcf8574_vals);
    server.sendContent(ts);
  }
  else
    server.sendContent(F("No PCF8574 pins found.<br>"));
  if (pcf8591Found)
  {
    Wire.beginTransmission(PCF8591_I2C);
    Wire.write(0x04); // Request a sequencial read of one pin after the latter
    Wire.endTransmission();
    Wire.requestFrom(PCF8591_I2C, 5);   // Read all of the ADC pins at once
    Wire.read();  // This is really just a dummy byte, technically left over from the last transmission
    for (uint8_t i = 0; i < 4; i++)   // Loop through each of the ADCs
    {
      adcvalue=(Wire.read() * 5) / 256;  // Get the value of each ADC, and adjust for a 5V reference
      sprintf(ts, "Analog %d: %6.2f<br>", i,adcvalue);
      server.sendContent(ts);
      delay(5);   // Give some time for conversions
    }
  }
  else
    server.sendContent(F("No PCF8591 found.<br>"));

  // ------- Start of 1-wire section to read DS18B20s connected to this device -------
  delay(1);
  ds18b20s.requestTemperatures();
  oneWire.reset_search();
  if (oneWire.search(sensor_address))
  {
    do
    {
      sprintf(ts,"%6.2f on %02x%02x%02x%02x%02x%02x<br>",ds18b20s.getTempF(sensor_address),sensor_address[2],sensor_address[3],sensor_address[4],sensor_address[5],sensor_address[6],sensor_address[7]);
      server.sendContent(ts);
      delay(1);
    } while (oneWire.search(sensor_address));
  }
  // ------ End of processing for 1-wire DS18B20 temperatures
  server.sendContent(F("Reading modbus devices...<br>This could take around 20 seconds<br>"));
  web_modbus();
  server.sendContent(F("Finished with modbus devices.<br>"));
  server.sendContent(F("All sensors have been read<br>"));
  server.sendContent(F("__________________________<br>"));
  server.sendContent(F("Reading the mqtt queue<br>"));
  
  if (queue_len)
  {
    queue_length = queue_len;   // queue_length is just the copy within this procedure, vs queue_len, which is the global value
    queue_position = 0;         // Used for getting info out of the queue FIFO, versus pulling from the end.  This is the local copy.
    do
    {
      queue_length--;
      sprintf(ts,"%s<br>",queue + queue_position);
      queue_position += QUEUE_WIDTH;
      server.sendContent(ts);
      delay(1);
    } while (queue_length > 0);
  }
  server.sendContent(F("Finished reading the mqtt queue<br>"));
  
  server.sendContent(F("</body></html>"));
  server.sendContent(""); // this closes out the send
  ESP.wdtFeed();
  server.client().stop();
  modbus_state = 0; // return things to normal
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
  
  server.sendHeader(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
  server.sendHeader(F("Pragma"), F("no-cache"));
  server.sendHeader(F("Expires"), F("-1"));
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  // here begin chunked transfer a line at a time
  server.send(200, "text/html");
  delay(1);
  server.sendContent(F("<!DOCTYPE HTML><html><head><meta content=\"text/html; charset=ISO-8859-1\" http-equiv=\"content-type\">" \
      "<meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0\"><html><head><title>"));
  server.sendContent(my_ssid);
  server.sendContent(F(" Status</title></head><body><p>The ssid is "));
  server.sendContent(server.arg("ssid"));
  server.sendContent(F("<br>And the password is "));
  server.sendContent(server.arg("Password"));
  server.sendContent(F("<br>And the MQTT IP Address is "));
  server.sendContent(server.arg("MQTT_IP"));
  server.sendContent(F("<br>And the Web Name is "));
  server.sendContent(server.arg("WebName"));
  if (server.arg("enable_debug") == "Debug")
    server.sendContent(F("<br>Debugging enabled"));
  server.sendContent(F("</P><BR><H2><a href=\"/\">go home</a></H2><br>"));
  server.sendContent(F("</body></html>"));
  server.sendContent(""); // this closes out the send
  server.client().stop();

  // write data to EEPROM memory
  strcpy(eeprom_data.validate,"valid");
  i = server.arg("ssid").length() + 1;  // needed to make sure the correct length is used for the values
  server.arg("ssid").toCharArray(eeprom_data.ssid,i);
  i = server.arg("Password").length() + 1;
  server.arg("Password").toCharArray(eeprom_data.password,i);
  ip.fromString(server.arg("MQTT_IP"));
  eeprom_data.mqtt = IPAddress(ip[0], ip[1], ip[2], ip[3]);
  i = server.arg("WebName").length() + 1;
  server.arg("WebName").toCharArray(eeprom_data.webname,i);
  if (server.arg("enable_debug") == "Debug")
    eeprom_data.debugging = 1;
  else
    eeprom_data.debugging = 0;
  ESP.wdtFeed();

  EEPROM.begin(512);
  EEPROM.put(0,eeprom_data);
  ESP.wdtFeed();
  EEPROM.commit();    // This is what actually forces the data to be written to EEPROM.
  EEPROM.end();
  delay(500);   // Wait for the eeprom to acually be written
  // It is simpler to just restart the Wemos at this time, than try to reset all values.
  ESP.restart();
}

//========================================================================================
// A very simple web page is produced by handleNotFound() when an invalid web page is
// requested of this device.
//========================================================================================
void handleNotFound()
{
  server.sendHeader(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
  server.sendHeader(F("Pragma"), F("no-cache"));
  server.sendHeader(F("Expires"), F("-1"));
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  // here begin chunked transfer a line at a time
  server.send(404, "text/html");
  delay(1);
  server.sendContent(F("File Not Found<br><br>URI:"));
  server.sendContent(server.uri());
  server.sendContent(F("</P><BR><H2><a href=\"/\">go home</a></H2><br>"));
  server.sendContent(F("</body></html>"));
  server.sendContent(""); // this closes out the send
  server.client().stop();
}

//========================================================================================
// End of the program.
//========================================================================================
