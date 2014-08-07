/**************************************************************************
This transmitter sleeps most of the time, retreives data from a sensor once
a minute, then wakes up every few seconds to transmit data.
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
you can contact Brian Bodiya at: ***REMOVED***
*************************************************************************/

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <VirtualWire.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

volatile int f_wdt=1;

float tmp_reading;
float hPa_reading;
int transfer_pin = 2; //digital
unsigned int sensorIdTmp = 16;
unsigned int sensorIdPre = 17;
uint8_t send_buffer_tmp[6];
uint8_t send_buffer_hPa[6];

unsigned int loop_count = 0;
bool goodReading = false;

void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  delay(500);
}

void setup(){
  Serial.begin(115200);  
  
  wdtSetup(0);

  // Initialise the IO and ISR
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_set_tx_pin(transfer_pin);
  vw_setup(1000);	 // Bits per sec
  
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  //don't sleep too fast, otherwise we can't burn new sketches
  delay(5000);
}

void loop(){
  if(f_wdt == 1)
  {
    //only read on 1/7 of wake ups (send every 56 seconds)
    if (loop_count % 7 == 0)
    {
      /* Get a new sensor event */ 
      sensors_event_t event;
      bmp.getEvent(&event);
     
      /* Display the results (barometric pressure is measure in hPa) */
      if (event.pressure)
      {
        hPa_reading = event.pressure;
        
        bmp.getTemperature(&tmp_reading);
        tmp_reading = (tmp_reading * 1.8) + 32;
    
        /* Then convert the atmospheric pressure, SLP and temp to altitude    */
        /* Update this next line with the current SLP for better results      */
        /*float seaLevelPressure = 1005;//SENSORS_PRESSURE_SEALEVELHPA;
        Serial.print("Altitude:    "); 
        Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                            event.pressure,
                                            temperature)); 
        Serial.println(" m");
        Serial.println("");*/
        goodReading = true;
      }
      else
      {
        goodReading = false;
      }
      
      Serial.print("BMP TEMP: ");Serial.println(tmp_reading);
      Serial.print("BMP PRES: ");Serial.print(hPa_reading);Serial.println("hPa");
      memcpy(&send_buffer_tmp[0],&sensorIdTmp,2);
      memcpy(&send_buffer_tmp[2],&tmp_reading,4);
      memcpy(&send_buffer_hPa[0],&sensorIdPre,2);
      memcpy(&send_buffer_hPa[2],&hPa_reading,4);
      
      loop_count = 1;
    }
    else
    {
      //resend the message, just for good measure
      loop_count++;
    }
    if(goodReading)
    {
      vw_send(send_buffer_tmp,6);
      vw_wait_tx(); // Wait until the whole message is gone
      vw_send(send_buffer_hPa,6);
      vw_wait_tx(); // Wait until the whole message is gone
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
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  //set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
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
