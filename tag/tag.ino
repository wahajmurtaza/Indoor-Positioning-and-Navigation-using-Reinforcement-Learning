
#include <SPI.h>
#include "DW1000Ranging.h"

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

float b_distance=2.0, c_distance=2.0, d_distance=2.0;
String b_string = "2.0",c_string = "2.0", d_string = "2.0", final_string = "2.0,2.0,2.0,";
unsigned int current_time =0, last_time = 0;
float b_arr[10] = {2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0};
float c_arr[10] = {2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0};
float d_arr[10] = {2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0};
float b_avg = 2.0 , c_avg = 2.0 , d_avg = 3.0 ;


void setup() {
  Serial.begin(9600);
  delay(1000);
  //init the configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);
  //Serial.println("started");
  
  //we start the module as a tag
  DW1000Ranging.startAsTag("0A:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, DW1000.CHANNEL_7,false);
}

void loop() {
  DW1000Ranging.loop();
  //Serial.println("loop");
  current_time = millis();
  if(current_time - last_time >500)
  {
    b_avg = 0; c_avg = 0; d_avg = 0;
    int denom = 0;
    for(int ii = 3; ii <= 9 ; ii++)
    {
      b_avg+=b_arr[ii];
      c_avg += c_arr[ii];
      d_avg += d_arr[ii];
    }
    
    b_avg = b_avg / 7.0;
    c_avg = c_avg / 7.0;
    d_avg = d_avg / 7.0;
    
    b_string = String(b_avg);
    c_string = String(c_avg);
    d_string = String(d_avg);
    
	  final_string = b_string + "," + c_string + "," + d_string + ",";
	  Serial.println(final_string);
	  last_time = current_time;
  }
}

void newRange() {
  uint16_t namezzz = DW1000Ranging.getDistantDevice()->getShortAddress();
  float current_distance = DW1000Ranging.getDistantDevice()->getRange();
  if (namezzz == 11){update_b();   b_arr[9] = current_distance ; }
  if (namezzz == 12){update_c();   c_arr[9] = current_distance ; }
  if (namezzz == 13){update_d();   d_arr[9] = current_distance ; }
}

void update_b()
{
  for (int i=1;i<=10;i++)
  {
    b_arr[i-1] = b_arr[i];
  }  
}

void update_c()
{
  for (int i=1;i<=10;i++)
  {
    c_arr[i-1] = c_arr[i];
  }  
}

void update_d()
{
  for (int i=1;i<=10;i++)
  {
    d_arr[i-1] = d_arr[i];
  }  
}




void newDevice(DW1000Device* device) {
}

void inactiveDevice(DW1000Device* device) {
}
