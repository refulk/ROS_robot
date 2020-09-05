//#define LED_PIN            13
#define HEATER_0_PIN       10

void setupEletroima() {
  pinMode(HEATER_0_PIN , OUTPUT);
  //pinMode(LED_PIN  , OUTPUT);  
}

void ligarEletroima()
{
  digitalWrite(HEATER_0_PIN, HIGH);
  //digitalWrite(LED_PIN, HIGH);
}

void desligarEletroima()
{
  digitalWrite(HEATER_0_PIN, LOW);
  //digitalWrite(LED_PIN, LOW);
}
