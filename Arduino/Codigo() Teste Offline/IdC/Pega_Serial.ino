  String leStringSerial(){
  String conteudo = "";
  char caractere;
  
  // Enquanto receber algo pela serial
  while(Serial2.available() > 0) {
    // Lê byte da serial
    caractere = Serial2.read();
    // Ignora caractere de quebra de linha
    if (caractere != '\n'){
      // Concatena valores
      conteudo.concat(caractere);
    }
    // Aguarda buffer serial ler próximo caractere
    delay(10);
  }
  Serial.print("Recebi: ");
  Serial.println(conteudo);

     
  convert_strtofloat(conteudo);  
  return conteudo;
}
