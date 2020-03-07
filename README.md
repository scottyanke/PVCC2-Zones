# Hallway temperatures and Tunnels

This project is part of a larger monitoring system for air handlers, temperature
and humidity sensors, air conditioners, and a steam boiler.

This specific project is for monitoring the temperatures in various parts of
the building by using **DS18B20s** scattered through the hallways, and HTU21Ds
in the tunnels.  WiFi is used to communicate the data to a Raspberry Pi server 
running Mosuitto (mqtt), although these devices also use the modbus protocal
over an RS485 connection (the tunnel part).

This projects device is an **ESP8266** on a Wemos D1 mini board.  It talks
through the guest WiFi network to a master machine running Mosquitto.  The
ESP8266 is both a station and an AP, with the AP being used for configuration,
status checks, and updates to the program firmware via OTA (Over The Air).  It
is only connected to the network as a station when it has information to send.
That's typically at least twice an hour, usually 3 times an hour.  The total time
spent on the WiFi network is generally less than 30 seconds an hour.

Each ESP8266 is identified by its MAC address.  That is what is used in the
transmission of all data to the main server.  The MAC is also used as the AP
name when updates need to be done or the status needs to be manually pulled.  In
some physical locations, up to 3 of these devices can be connected to, thus the
unique naming.  There are no DIP switches or jumpers associated with these
devices.  The MAC address is unique enough, and eliminate the need to create any
additional addressing.

Time is retrieved from a web page.  NTP is not possible in this environment, as
that port was blocked.  All servers send the date and time in their headers
anyway, so a simple get is all that is needed.  Time is set as UTC, with the
local time being checked during reporting or when something needs to happen at a
specific time of day.  It was easier for Daylight Savings issues to be handled
this way.

This program scans for any DS18B20s it can find.  If it finds any one, it will
report all that it found.  The serial number is unique for each DS18B20, and is
used as the id for the location of the temperature of every DS18B20 it can find.
All temperatures are sent as Celcius numbers, instead of Farhenheit, so as to be
consistent across sensors.  Up to 10 DS18B20s can be read by each device, which 
is more than they realistically can support due to wiring issues.

Because of the lengths of cable that have to be used, only 4 of the DS18B20s
seem to work on one line.  Adding the fifth tends to shut them all down, at
least with this setup.  Altogether, over multiple devices, about 3,000 feet of
CAT-3 cable is used to connect to the DS18B20s, with 3 volts supplied to each
device (not using parasite power, since we've got enough wire pairs).

The modbus part of this program allows it to connect to devices in the tunnels
where WiFi won't reach.  In effect, it acts as a forwarder of the date to
the main system.  RS485 is used, and up to 10 devices are scanned for responses.
If a device responds, it's there.  If not, then the assumpumption is that there
is nothing with that address connected.  A subset of the standard modbus protocol
is used, limited to reading the registers from each tunnel device.

Arduino was used to create the program, compile it, and distribute it to about
ten of the ESP8266 chips.  The full details about the Arduino environment are
described in the program header.  It was just easier this way.

Anyone can use these programs however they wish.  These are designs that are
being used every day in a nursing home.  Much of the code came from other
examples that people have on GitHub or in Arduino.
