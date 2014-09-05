/**************************************************************************
This receiver collects sensor data from the transmitter nodes and forwards
the latest results to ThinkSpeak once a minute.
Copyright (C) 2014 Brian Bodiya

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

For questions, comments, or suggestions about this software,
you can contact Brian Bodiya at: brian@bodiya.com
*************************************************************************/

#include <VirtualWire.h>
#include <Process.h>

int receiverPin = 11;
int led = 12;

unsigned int numChannels = 3;
//0 = Home Temperature
//1 = Home Sensors
//2 = Home Test Channel
String THINKSPEAK_API_KEY[3] = {"***REMOVED***","***REMOVED***","***REMOVED***"};

unsigned int numSensors = 24;
//0 = ***REMOVED***'s Room (temp)
//1 = Living Room (temp)
//2 = Master Bedroom (temp)
//3 = Office (temp)
//4 = Basement (finished) (temp)
//5 = Basement (unfinished) (temp)
//6 = Kitchen (temp)
//8 = Basement (finished) (humidity)
//9 = Basement (unfinished) (humidity)
//10 = Kitchen (pressure)
float lastReading[24];

unsigned long lastWriteTime = 0;

void setup()
{
  pinMode(led, OUTPUT);
  blinkLED();
  //Serial.begin(9600);	// Debugging only
  //Serial.println("setup");
  Bridge.begin();
  Console.begin();
  //while (!Console);
  Console.println("setup");
  clearReadings();

  // Initialise the IO and ISR
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_set_rx_pin(receiverPin);
  vw_setup(1000);	 // Bits per sec

  vw_rx_start();       // Start the receiver PLL running
}

void loop()
{
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  
  //blink_LED();
  if (vw_get_message(buf, &buflen) && (buflen == 6)) // Non-blocking
  {
    /*Serial.print("Received ");Serial.print(buflen);Serial.println(" bytes");
    Serial.print("Received0: ");Serial.println(buf[0]);
    Serial.print("Received1: ");Serial.println(buf[1]);
    Serial.print("Received2: ");Serial.println(buf[2]);
    Serial.print("Received3: ");Serial.println(buf[3]);
    Serial.print("Received4: ");Serial.println(buf[4]);*/
    unsigned int* sensorId  = (unsigned int*)&buf[0];
    float* reading  = (float*)&buf[2];
    lastReading[*sensorId] = *reading;
    Console.print(*sensorId);Console.print(": ");Console.println(*reading);
    //Serial.println(*reading);
    blinkLED();
  }
  unsigned long currentTime = millis();
  if ((currentTime-lastWriteTime >= 60000UL) || (currentTime < lastWriteTime))
  {
    digitalWrite(led, HIGH);
    postToThingSpeak(THINKSPEAK_API_KEY,lastReading);
    clearReadings();
    digitalWrite(led, LOW);
    lastWriteTime = millis();
  }
}

void clearReadings()
{
  for(int i = 0;i < numSensors;i++)
  {
    lastReading[i] = 250000000;
  }
}

void blinkLED()
{
  digitalWrite(led, HIGH);
  delay(10);
  digitalWrite(led, LOW);
}

void postToThingSpeak(String key[], float value[]) {
  Process p;
  for (int j=0;j<numChannels;j++) {
    String cmd = "curl --data \"key="+key[j];
    bool update_required = false;
    for (int i=0;i<8;i++) {
      if (value[i+(j*8)] < 240000000)
      {
        cmd = cmd + "&field"+ (i+1) + "=" + value[i+(j*8)];
        update_required = true;
      }
    }
    cmd = cmd + "\" http://api.thingspeak.com/update";
    if (update_required)
      p.runShellCommand(cmd);
    Console.println(cmd);
  }
  p.close();
}
