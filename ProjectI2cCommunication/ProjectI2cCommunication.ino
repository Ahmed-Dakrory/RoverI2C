// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

//Mode 0 angleRover
//Mode 1 gyroRover
//Mode 2 actionDistance
//Mode 3 angleAction
//Mode 4 actionRate


int mode = 0;
void setup() {
  Wire.begin(8);                // join i2c bus with address #8
   Wire.onReceive(receiveEvent); // register event
  
  Wire.onRequest(sendNumberOfLocations); // sendData event
}

void loop() {
  
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()

  String angleRover="";
  String gyroRover="";
  String actionDistance="";
  String angleAction="";
  String actionRate="";

  long timeBefore =0;
  long totalRecieveTime = 0;
  
void receiveEvent(int howMany) {
  String valueRead = "";
  boolean neg = false;
  while ( Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    if(c == '!'){
      mode = 5;
      neg = true;
      totalRecieveTime = (millis()-timeBefore);
      Serial.print(angleRover);
      Serial.print(",");      
      Serial.print(gyroRover);
      Serial.print(": ");
      Serial.print(actionDistance);
      Serial.print(",");      
      Serial.print(angleAction);
      Serial.print(",");      
      Serial.print(actionRate);
      Serial.print("  , time: ");
      Serial.println(totalRecieveTime);
      
      timeBefore=millis();
      angleRover="";
      gyroRover="";
      actionDistance="";
      angleAction="";
      actionRate="";
    }else if(c == ';'){
      neg = true;
      mode = 4;
    }else if(c == '$'){
      neg = true;
      mode = 3;
    }else if(c == ':'){
      neg = true;
      mode = 2;
    }else if(c == '&'){
      neg = true;
      mode = 1;
    }else if(c == '#'){
      neg = true;
      mode = 0;
    }else{
      neg = false;
    }
    
    if(mode == 0 && !neg){
      angleRover = angleRover+c;
    }else if(mode == 1 && !neg){
      gyroRover = gyroRover+c;
    }else if(mode == 2 && !neg){
      actionDistance = actionDistance+c;
    }else if(mode == 3 && !neg){
      angleAction = angleAction+c;
    }else if(mode == 4 && !neg){
      actionRate = actionRate+c;
    }
  }
 
}


void sendNumberOfLocations() {
                              // device address is specified in datasheet
  Wire.write("51.9228757,4.4800224,51.9228757,4.4800224,51.9228757,4.4800224"); // respond with message of 6 bytes
  
  // as expected by master
}
