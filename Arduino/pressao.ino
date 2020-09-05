/* FSR testing sketch. 
 
Connect one end of FSR to 5V, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground
Connect LED from pin 11 through a resistor to ground 
 
For more information see www.ladyada.net/learn/sensors/fsr.html */
 
int fsrAnalogPin = 13; // FSR is connected to analog 0
int fsrReading;      // the analog reading from the FSR resistor divider

#define LED_PIN_3            3

//unsigned long millisRead = 0;
//unsigned int delayRead = 100;

void setupPressao() 
{
  pinMode(LED_PIN_3  , OUTPUT);  
}

void loopPressao()
{
  fsrReading = analogRead(fsrAnalogPin);
  Serial.print("Analog reading = ");
  Serial.println(fsrReading);  
  
  if(fsrReading > 705)
  {
    digitalWrite(LED_PIN_3, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN_3, LOW);
  }      
  delay(100);
}


 
