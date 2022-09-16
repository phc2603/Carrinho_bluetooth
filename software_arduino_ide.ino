//inicializando o bluetooth
#include  "BluetoothSerial.h"
#include "string.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#include <Arduino.h>
//portas da esp32 que abrigam os motores
int IN1 = 13;
int IN2 = 12;
int IN3 = 14;
int IN4 = 27;
int Trigger = 33;//entrada do pino que dispara o pulso do sensor
int Echo = 32;//Entrada do pino usado para ler a saida do sensor
float tempo;
float distancia;
BluetoothSerial SerialBT;


void setup() {
  //inicializando o pino que dispara o pulso do sensor, o pino que recebe a saida do sensor e os motores
  Serial.begin(9600);
  pinMode(Trigger,OUTPUT);
  digitalWrite(Trigger,LOW);
  pinMode(Echo,INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  delay(100);
  SerialBT.begin("ESP32-DR.ROB");
  Serial.println("PRONTO!!!");

}


//procedimento criado para enviar o pulso ultrassonico
void Dispara_pulso(){
  digitalWrite(Trigger,HIGH);//ao enviar um nivel alto para o pino trigger, ele dispara o pulso
  delayMicroseconds(10);
  digitalWrite(Trigger,LOW);
}


//funcao para calcular a distancia total atraves do tempo medido, convertido para segundos, multiplicado pela velocidade do som
float distancia_total(float tempo_total){
    float distancia_local;
    tempo_total = tempo_total/1000000;
    distancia_local = (340*tempo_total)/2.0;
    distancia_local = distancia_local * 100;//convertendo para centimetro
    return distancia_local;
}

//procedimento para girar os 4 motores no sentido horário
void frente(){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

//procedimento para girar os 4 motores no sentido anti-horário
void tras(){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);  
}

//procedimento para desligar os 2 motores da esquerda e ligar os 2 da direita
void esquerda(){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);  
}

//procedimento para desligar os 2 motores da direita e ligar os 2 da esquerda
void direita(){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
}

//procedimento para parar os 4 motores
void parar(){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
}

void loop() {
    if (SerialBT.available()){
        SerialBT.write(Serial.read());
    }  
        int numero_recebido = SerialBT.read();//numero que foi enviado ao microcontrolador, com base na acao do usuario
        //distancia de colisao atraves do sensor ultrassonico
        Dispara_pulso();
        tempo = pulseIn(Echo,HIGH);//metodo que mede o tamanho do pulso em microsegundos
        distancia = distancia_total(tempo); 
        Serial.println(distancia);
        //caso a distancia seja menor que 7, enviar esse valor ao serial para emitir o alerta de possivel colisao
        if (distancia<7){
          String distancia_str = "";
          distancia_str.concat(distancia); 
          SerialBT.println(distancia_str);  
        }
        
         if(numero_recebido == 1){
            //configurando o motor para mover para frente
            Serial.println("FRENTE");
            frente();
         }
         else if (numero_recebido == 2){
              //configurando motor para mover para tras
              Serial.println("TRAS");  
              tras();
         }
         else if(numero_recebido == 3){
          //configurando motor para mover a esquerda
            Serial.println("ESQUERDA");
                   esquerda(); 
 
         }
         else if(numero_recebido == 4){
         //configurando motor para mover a direita
             Serial.println("DIREITA");
                    direita();
         }
         else if (numero_recebido == 5){
         //configurando motor para parar
             Serial.println("PARA");
                    parar();
         }
    delay(30);
       
}
