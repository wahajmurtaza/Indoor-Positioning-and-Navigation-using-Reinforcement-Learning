
/* This example is written for Nodemcu Modules */

#include "ESP_Wahaj.h" // importing our library
#include <SoftwareSerial.h>
SoftwareSerial mySerial(D7,D8);



# define led D4
int pwm = 255;
String path = "nothing";
String angle = "255";
String distances = "3.0,3.0,3.0,\r\n";


void setup(){
  Serial.begin(9600);
  mySerial.begin(9600); 
  start("xxx","password");  // Wifi details connect to
  Serial.setTimeout(50) ;
  mySerial.setTimeout(50);
  

}

void loop(){

  if(Serial.available()>0)
  {
    angle = Serial.readString();
   // Serial.println(angle);
   
  }

  if(mySerial.available() >0)
  {
     distances=mySerial.readString();
  }

  if(CheckNewReq() == 1)
  {
    
    if (getPath()=="/distances"){
    returnThisStr(distances);
    }
    else if (getPath()=="/angle"){
    //returnThisStr(angle);
    returnThisInt(angle.toInt());
    //returnThisInt(angle); 
    }
    else if(getPath()=="/favicon.ico"){   
      returnThisStr("garbage");
    }

    else       
    {
      path = getPath();
      path.remove(0,1);   
      //Serial.println(path);
      pwm = path.toInt();
         //mySerial.println(String(pwm));
         Serial.println(pwm);
      returnThisInt(pwm); 
    }
    
  }
  
delay(60);
  
}
