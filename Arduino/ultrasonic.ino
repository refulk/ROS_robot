#include <Ultrasonic.h>

//frente, direita, tras, esquerda
const int echoPin[] = {17,29,21,32}; //PINO DIGITAL UTILIZADO PELO HC-SR04 ECHO(RECEBE)
const int trigPin[] = {16,27,20,47}; //PINO DIGITAL UTILIZADO PELO HC-SR04 TRIG(ENVIA)

//const int echoPin1 = 17; //PINO DIGITAL UTILIZADO PELO HC-SR04 ECHO(RECEBE)#frente
//const int trigPin1 = 16; //PINO DIGITAL UTILIZADO PELO HC-SR04 TRIG(ENVIA)
//const int echoPin2 = 29; //PINO DIGITAL UTILIZADO PELO HC-SR04 ECHO(RECEBE)#direita
//const int trigPin2 = 27; //PINO DIGITAL UTILIZADO PELO HC-SR04 TRIG(ENVIA)
//const int echoPin3 = 21; //PINO DIGITAL UTILIZADO PELO HC-SR04 ECHO(RECEBE)#tras
//const int trigPin3 = 20; //PINO DIGITAL UTILIZADO PELO HC-SR04 TRIG(ENVIA)
//const int echoPin4 = 32; //PINO DIGITAL UTILIZADO PELO HC-SR04 ECHO(RECEBE)#esquerda
//const int trigPin4 = 47; //PINO DIGITAL UTILIZADO PELO HC-SR04 TRIG(ENVIA)

Ultrasonic ultrasonic0(trigPin[0],echoPin[0]); //INICIALIZANDO OS PINOS DO ARDUINO
Ultrasonic ultrasonic1(trigPin[1],echoPin[1]); //INICIALIZANDO OS PINOS DO ARDUINO
Ultrasonic ultrasonic2(trigPin[2],echoPin[2]); //INICIALIZANDO OS PINOS DO ARDUINO
Ultrasonic ultrasonic3(trigPin[3],echoPin[3]); //INICIALIZANDO OS PINOS DO ARDUINO

unsigned long duration;
int distance; //VARIÁVEL DO TIPO INTEIRO
String result2; //VARIÁVEL DO TIPO STRING

int distancia; //VARIÁVEL DO TIPO INTEIRO
String result; //VARIÁVEL DO TIPO STRING

unsigned long mUltra = 0;
unsigned int dUltra = 500;

int distanciaMinima = 6; //cm

int index = 0;
int valores[4][5] = {
  {1,1,1,1,1},
  {1,1,1,1,1},
  {1,1,1,1,1},
  {1,1,1,1,1}
};
int soma[4] = {5,5,5,5};

void setupUltrasonic() 
{
  pinMode(echoPin[0], INPUT); //DEFINE O PINO COMO ENTRADA (RECEBE)
  pinMode(trigPin[0], OUTPUT); //DEFINE O PINO COMO SAIDA (ENVIA)
  pinMode(echoPin[1], INPUT); //DEFINE O PINO COMO ENTRADA (RECEBE)
  pinMode(trigPin[1], OUTPUT); //DEFINE O PINO COMO SAIDA (ENVIA)
  pinMode(echoPin[2], INPUT); //DEFINE O PINO COMO ENTRADA (RECEBE)
  pinMode(trigPin[2], OUTPUT); //DEFINE O PINO COMO SAIDA (ENVIA)
  pinMode(echoPin[3], INPUT); //DEFINE O PINO COMO ENTRADA (RECEBE)
  pinMode(trigPin[3], OUTPUT); //DEFINE O PINO COMO SAIDA (ENVIA)
}

//MÉTODO RESPONSÁVEL POR CALCULAR A DISTÂNCIA
//void hcsr04_OLD()
//{
//    digitalWrite(trigPin, LOW); //SETA O PINO 6 COM UM PULSO BAIXO "LOW"
//    delayMicroseconds(2); //INTERVALO DE 2 MICROSSEGUNDOS
//    digitalWrite(trigPin, HIGH); //SETA O PINO 6 COM PULSO ALTO "HIGH"
//    delayMicroseconds(10); //INTERVALO DE 10 MICROSSEGUNDOS
//    digitalWrite(trigPin, LOW); //SETA O PINO 6 COM PULSO BAIXO "LOW" NOVAMENTE
//    //FUNÇÃO RANGING, FAZ A CONVERSÃO DO TEMPO DE
//    //RESPOSTA DO ECHO EM CENTIMETROS, E ARMAZENA
//    //NA VARIAVEL "distancia"
//    distancia = (ultrasonic.Ranging(CM)); //VARIÁVEL GLOBAL RECEBE O VALOR DA DISTÂNCIA MEDIDA
//    result = String(distancia); //VARIÁVEL GLOBAL DO TIPO STRING RECEBE A DISTÂNCIA(CONVERTIDO DE INTEIRO PARA STRING)
//    
////    delay(500); //INTERVALO DE 500 MILISSEGUNDOS
//}

//MÉTODO RESPONSÁVEL POR CALCULAR A DISTÂNCIA
void hcsr04(int direcao)
{
    digitalWrite(trigPin[direcao], LOW); //SETA O PINO 6 COM UM PULSO BAIXO "LOW"
    delayMicroseconds(2); //INTERVALO DE 2 MICROSSEGUNDOS
    digitalWrite(trigPin[direcao], HIGH); //SETA O PINO 6 COM PULSO ALTO "HIGH"
    delayMicroseconds(10); //INTERVALO DE 10 MICROSSEGUNDOS
    digitalWrite(trigPin[direcao], LOW); //SETA O PINO 6 COM PULSO BAIXO "LOW" NOVAMENTE
    
    duration = pulseIn(echoPin[direcao], HIGH);
    distance = duration*0.034/2;
    result2 = String(distance); //VARIÁVEL GLOBAL DO TIPO STRING RECEBE A DISTÂNCIA(CONVERTIDO DE INTEIRO PARA STRING)
    
}

boolean freeWay(int direcao)
{
  hcsr04(direcao);
  if(distance > distanciaMinima)
    return true; //caminho livre, pode prosseguir
  else
    return false; //obstaculo detectado
}

//só deve verificar na direcao do movimento
void detectarObstaculo(char *obstaculo, int direcao)
{
  char aux[10];
  if(freeWay(direcao)) 
  { //livre = 1
    //return "0"; //livre
    strcpy(obstaculo,"0");
  }
  else
  { //obstaculo detectado = 0
    //return char(direcao);
    direcao++;
    itoa(direcao,obstaculo,10); 
    //strcpy(obstaculo,"1");  
  }
}
/*
char detectarObstaculo(int direcao)
{
  if(freeWay(direcao)) 
  { //livre = 1
    if(valores[direcao][index] == 0)
    {
      soma[direcao]++;
      valores[direcao][index] = 1;      
    }
  }
  else
  { //obstaculo detectado = 0
    if(valores[direcao][index] == 1)z
    {
      soma[direcao]--;   
      valores[direcao][index] = 0;       
    }  
  }
  index++;
  if(index == 5)
    index = 0;
  if(soma[direcao] <= 2)
    return char(direcao);
    //return '1'; //sensor 1 com obstaculo
  else
    return '0'; //livre
}
*/
void loopUltrasonic(int direcao)
{
  
  if(delayMillis(&mUltra, dUltra))
  {
    hcsr04(direcao);
    Serial.println(result2);
  }
  
}
 
