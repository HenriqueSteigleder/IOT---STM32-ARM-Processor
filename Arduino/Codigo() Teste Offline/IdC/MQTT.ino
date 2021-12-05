void conectaMQTT(void) {
  //Verifica conexão
  if(!client.connected()){
    //Enquanto não estivermos conectados
    while (!client.connected()){
    Serial.print("Realizando uma conexão MQTT...");
    //Tenta realizar uma conexão
    if(client.connect(clientID, username, password)) Serial.println("conectado");
    else{
      Serial.print("Falha ao conectar, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5s");
      delay(5000);
      }
    }
  }
  client.loop();
}

void publicaMQTT(const char* nomeVariavel, float valorVariavel,const char* unidVariavel){
  //String msg = String("{\"variable\":\"")+String(nomeVariavel)+String("\",\"value\":")+String(valorVariavel)+String("}");
  char payload[128]="";
  sprintf(payload, "{\"variable\":\"%s\",\"value\":%f,\"unit\":\"%s\"}", nomeVariavel, valorVariavel,unidVariavel);
  Serial.print("Mensagem publicada: ");
  Serial.println(payload);
  client.publish(publish_topic, payload);
}
