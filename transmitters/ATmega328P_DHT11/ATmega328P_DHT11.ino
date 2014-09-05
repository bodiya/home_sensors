/**************************************************************************
This transmitter sleeps most of the time, retreives data from a sensor once
a minute, then wakes up every so often to transmit data.
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

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <VirtualWire.h>
#include "DHT.h"

#define DHTPIN 9     // what pin we're connected to
#define DHTTYPE DHT11   // DHT 11 
DHT dht(DHTPIN, DHTTYPE);

volatile int f_wdt=1;

float tmp_reading;
float hum_reading;
float lastTmpReading = 999;
float lastHumReading = 999;
bool newTmpReading = false;
bool newHumReading = false;
int transfer_pin = 2; //digital
unsigned int sensorIdTmp = 4;
unsigned int sensorIdHum = 5;
uint8_t send_buffer_tmp[6];
uint8_t send_buffer_hum[6];

unsigned int loop_count = 0;
bool goodReading = false;

void setup(){
  Serial.begin(115200);  
  
  wdtSetup(0);

  // Initialise the IO and ISR
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_set_tx_pin(transfer_pin);
  vw_setup(1000);	 // Bits per sec
  
  dht.begin();
  
  //don't sleep too fast, otherwise we can't burn new sketches
  delay(5000);
}

void loop(){
  if(f_wdt == 1)
  {
    //only read on 1/7 of wake ups (send every 56 seconds)
    if (loop_count % 7 == 0)
    {
      hum_reading = dht.readHumidity();
      newHumReading = (hum_reading != lastHumReading);
      lastHumReading = hum_reading;
      // Read temperature as Fahrenheit
      tmp_reading = dht.readTemperature(true);
      newTmpReading = (tmp_reading != lastTmpReading);
      lastTmpReading = tmp_reading;
      goodReading = !(isnan(hum_reading) || isnan(tmp_reading));
      goodReading = goodReading && (newTmpReading || newHumReading);
      Serial.print("DHT TEMP: ");Serial.println(tmp_reading);
      Serial.print("DHT HUM : ");Serial.print(hum_reading);Serial.println("%");
      memcpy(&send_buffer_tmp[0],&sensorIdTmp,2);
      memcpy(&send_buffer_tmp[2],&tmp_reading,4);
      memcpy(&send_buffer_hum[0],&sensorIdHum,2);
      memcpy(&send_buffer_hum[2],&hum_reading,4);
      
      loop_count = 1;
    }
    else if ((loop_count % 2) && (goodReading))
    {
      if (newTmpReading)
      {
        vw_send(send_buffer_tmp,6);
        vw_wait_tx(); // Wait until the whole message is gone
      }
      if (newHumReading)
      {
        vw_send(send_buffer_hum,6);
        vw_wait_tx(); // Wait until the whole message is gone
      }
      loop_count++;
    }
    else
    {
      loop_count++;
    }
    
    /* Don't forget to clear the flag. */
    f_wdt = 0;

    /* Re-enter sleep mode. */
    enterSleep();
  }
}

/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
  //disable ADC
  ADCSRA = 0;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  //set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
  //enable ADC
  //ADCSRA = 1;

  /* Re-enable the peripherals. */
  power_all_enable();
}

void wdtSetup(int setting)
{
  byte timer = 1<<WDP0 | 1<<WDP2; /* 0.5 seconds */
  if (setting == 1)
    timer = 1<<WDP2; /* 0.25 seconds */
  else if (setting == 2)
    timer = 1<<WDP0 | 1<<WDP1; /* 0.125 seconds */
  else if (setting = 3)
    timer = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */

  /*** Setup the WDT ***/
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  //WDTCSR = timer;
  //WDTCSR = 1<<WDP1 | 1<<WDP2; /* 1.0 seconds */
  //WDTCSR = 1<<WDP0 | 1<<WDP1 | 1<<WDP2; /* 2.0 seconds */
  
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE); 
}

/***************************************************
 *  Name:        ISR(WDT_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 *
 ***************************************************/
ISR(WDT_vect)
{
  if(f_wdt == 0)
  {
    f_wdt=1;
  }
  else
  {
    Serial.println("WDT Overrun!!!");
    delay(100);
  }
}
