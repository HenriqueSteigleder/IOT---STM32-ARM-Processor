/*Projeto da Internet das Coisas
 * 
 * Autor: Henrique Dalcin Steigleder
 * Placa: NodeMCU 8266
 * 
 */

//Bibliotecas necessárias
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <string.h>

//Configurações da Internet
const char* ssid = "Wifi_ID";
const char* pass = "Wifi_Password";

//Configurações do Broker
const char* broker_address = "mqtt.tago.io";
const int broker_port = 1883;
const char* clientID = "UserName";
const char* username = "token";
const char* password = "fdbb435d-3c31-4d92-befb-090f15547d5b";
const char* publish_topic = "tago/data/post";

//Cria as instancias dos objetos
WiFiClient espClient;
PubSubClient client(espClient);

//Variaveis globais
float temp=0;
String Texto = "";
float umid = 0;
float som = 0;
float numero = 0;
float luz = 0;
int Conta_Envia = 0;
int Conta_Recebe = 0;


void setup() {
  // put your setup code here, to run once:
  setUp_Inicial();
}

void convert_strtofloat(String conteudo){
 
    //Aqui vamos Converter Os dados da Serial (char) para valores Float
   char str[50];
   const char s[2] = "-"; // linha 45

   conteudo.toCharArray(str, 50);
   
   char *token;
   char *rest = str;   
   
   /* strtok separa os pedacos de uma string em outras menores com base em um ponto específico que delimitamos 
   * "-" vai ser o caracter que vai separar as diversas strings que teremos dentro de uma só 
   *Aqui vamos separar a primeira string dentro do ponteiro Token
   */
      token = strtok_r(rest, s, &rest); //linha 28
   
   /* walk through other tokens */
      temp = atof(token);
      //------------------------------------
      
      token = strtok_r(rest, s, &rest);
      som = atof(token);
      //------------------------------------
      token = strtok_r(rest, s, &rest);
      umid = atof(token);
      //------------------------------------
      token = strtok_r(rest, s, &rest);
      luz = atof(token);

      
    while ((token = strtok_r(rest, " ", &rest))){
    }
    
    //printf( " %s\n", str );
    Serial.println(temp);
    Serial.println(som);
    Serial.println(umid);
    Serial.println(luz);
    return;
}

void loop() {
  // put your main code here, to run repeatedly:
  conectaMQTT();
  
  if(Conta_Envia>60){
  publicaMQTT("temperature", temp,"C");
  publicaMQTT("umidade", umid,"%");
  publicaMQTT("som", som,"dB");
  publicaMQTT("luz", luz,"Lx");
  Conta_Envia = 0;
  }
  Conta_Envia++;
  
  delay(1000);
    if (Serial2.available() > 0){
    // Lê toda string recebida
    String recebido = leStringSerial();
  }
  Conta_Recebe++;
  if(Conta_Recebe>30){
    //Recebe aqui dados 
    //que eu for ler da 
    //tago.io
    Conta_Recebe = 0;
  }
}


void brokerNewMessageCallback(char* topic, byte* payload, unsigned int length){
  //Interrupcao chamada quando uma nova mensagem é recebida do broker

}
