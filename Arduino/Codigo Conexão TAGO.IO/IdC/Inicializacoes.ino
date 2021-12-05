void setUp_Inicial(void){

  Serial2.begin(115200);
  Serial.begin(115200);
  while (!Serial) {
     // wait for serial port to connect. Needed for native USB port only
  }
  Serial.begin(115200); //inicial serial em 115200
  delay(10);
  Serial.print("\nConectando a ");
  Serial.println(ssid);

  
  //Inicia conexão; Enquanto não conecta imprime '.'
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConexão WiFi estabelecida");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());

  //Conecta ao broker
  client.setServer(broker_address, broker_port);

  //Define funcao de callback ao receber mensagem do broker
  client.setCallback(brokerNewMessageCallback);

}
