
// Funcao uteis, gerais

void setupUteis() 
{
  Serial.begin(9600); //INICIALIZA A PORTA SERIAL
}

boolean delayMillis(unsigned long *millisCount, unsigned int delayCount)
{
  if(millis() >= (*millisCount + delayCount))
  {
    *millisCount = millis();
    return true;
  }
  else
  {
    return false;
  }
}

boolean delayMicros(unsigned long *microsCount, unsigned long delayCount)
{
  if(micros() >= (*microsCount + delayCount))
  {
    *microsCount = micros();
    return true;
  }
  else
  {
    return false;
  }
}

boolean delayMillisKeep(unsigned long *millisCount, unsigned int delayCount)
{
  if(millis() >= (*millisCount + delayCount))
  {
//    *millisCount = millis();
    return true;
  }
  else
  {
    return false;
  }
}

//void loopUteis()
//{
//  
//}


 
