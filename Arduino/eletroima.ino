#define HEATER_0_PIN       10

void setupEletroima() {
  pinMode(HEATER_0_PIN , OUTPUT);
}

void ligarEletroima()
{
  digitalWrite(HEATER_0_PIN, HIGH);
}

void desligarEletroima()
{
  digitalWrite(HEATER_0_PIN, LOW);
}
