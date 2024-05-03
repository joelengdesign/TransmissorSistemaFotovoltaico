#include <Arduino.h>

#include "MODBUS.h"

MODBUS::MODBUS(int RS485Comunicacao){
    pinMode(RS485Comunicacao, OUTPUT);
    pinoRS485Comunicacao = RS485Comunicacao;
}

void MODBUS::EnviarPacote(byte EnderecoDoDispositivo,
                        byte CodigoDaFuncao, uint16_t EnderecoInicial,
                        uint16_t QuantidadeDeRegistradores){
    // Serial.println("Construindo Pacote de Requisição...");
    byte CodigoDeEnvioParcial[6];
    byte CodigoDeEnvio[8];
    uint16_t ErrorCheckEnvio;

    CodigoDeEnvioParcial[0] = EnderecoDoDispositivo;
    CodigoDeEnvioParcial[1] = CodigoDaFuncao;
    CodigoDeEnvioParcial[2] = ((EnderecoInicial >> 8*1) & 0xff);
    CodigoDeEnvioParcial[3] = ((EnderecoInicial >> 8*0) & 0xff);
    CodigoDeEnvioParcial[4] = ((QuantidadeDeRegistradores >> 8*1) & 0xff);
    CodigoDeEnvioParcial[5] = ((QuantidadeDeRegistradores >> 8*0) & 0xff);

    ErrorCheckEnvio = ErrorCheck(CodigoDeEnvioParcial,sizeof(CodigoDeEnvioParcial));

    for (byte i=0; i<sizeof(CodigoDeEnvioParcial);i++){
        CodigoDeEnvio[i]=CodigoDeEnvioParcial[i];
    }
    
    CodigoDeEnvio[6] = ((ErrorCheckEnvio >> 8*1) & 0xff);
    CodigoDeEnvio[7] = ((ErrorCheckEnvio >> 8*0) & 0xff);

    // Serial.println("");
    // Serial.print("Pacote de Requisição: ");
    // printar(CodigoDeEnvio,sizeof(CodigoDeEnvio));

    digitalWrite(pinoRS485Comunicacao,HIGH);
    Serial1.write(CodigoDeEnvio,sizeof(CodigoDeEnvio));
    Serial1.flush();
    digitalWrite(pinoRS485Comunicacao,LOW);
    stillWaitNextBit = true;
    // Serial.print(stillWaitNextBit);
}

float MODBUS::IEEE754_HexToFloat(uint8_t dado[], int n){
  union{
    float valor_float;
    uint32_t valor_inteiro;
  }conversor;

  conversor.valor_inteiro = ((uint32_t)dado[n] << 24) | 
                              ((uint32_t)dado[n+1] << 16)|
                              ((uint32_t)dado[n+2] << 8) | 
                              (uint32_t)dado[n+3];
  return(conversor.valor_float);
}

uint16_t MODBUS::ErrorCheck(byte mensagem[], uint8_t tamanho)
{  
  uint16_t crc = 0xFFFF;
  for (byte pos = 0; pos < tamanho; pos++)  {
    crc ^= mensagem[pos];    // Faz uma XOR entre o LSByte do CRC com o byte de dados atual
 
    for (byte i = 8; i != 0; i--) {    // Itera sobre cada bit
      if ((crc & 0b1) != 0) {      // Se o LSB for 1:
        crc >>= 1;                  // Desloca para a direita
        crc ^= 0xA001;              // E faz XOR com o polinômio 0xA001 (1010 0000 0000 0001 ): x16 + x15 + x2 + 1
      }else{                      // Senão:
        crc >>= 1;                  // Desloca para a direita
      }
    }
  }

  // O formato retornado já sai invertido (LSByte primeiro que o MSByte)
  crc = ((crc & 0xff) << 8*1) | (crc>>8*1);
  return crc;
}

// função booleana que verifica a integridade do pacote, de forma que retorna falso caso a integridade do pacote esteja comprometida
bool MODBUS::validacaoPacote(byte pacote[]){
  bool valid;
  valid = false;
  byte quantBytes = pacote[2]; /* 
  captura a informação relacionada ao tamanho da carga útil localizada em:
  pacote[0] - Identificador do dispositivo
  pacote[1] - Código da função
  pacote[2] - quantidade de bytes da carga útil

  Carga útil
  pacote[3] - primeira posição da carga útil
  .
  .
  .
  pacote[UltimoIndice-2] - última posição da carga útil

  Os dois últimos bytes se referem ao CRC do pacote recebido
  pacote[UltimoIndice-1]
  pacote[UltimoIndice]
  */ 

  // O tamanho do pacote parcial é o tamanho da mensagem de requisição menos os dois últimos bytes do ErrorCheck recebido
  byte pacoteParcial[quantBytes+3]; 

  // Acrescenta o ErrorCheck recebido em uma variável de 2 bytes utilizando deslocamento de bits
  uint16_t ErrorCheckRebecido = (((uint16_t)(pacote[quantBytes+3] & 0xff)) << 8*1) | (((uint16_t)(pacote[quantBytes+4] & 0xff)) << 8*0);
  // Serial.print("ErrorCheck Recebido: ");
  // Serial.println(ErrorCheckRebecido,HEX);


  // Extrai apenas o pacote sem os dois últimos bytes para calcular o ErrorCheck e comparar com o ErrorCheck recebido
  for(byte i=0;i<sizeof(pacoteParcial);i++){
    pacoteParcial[i] = pacote[i];
  }

  // calcula o ErrorCheck 
  uint16_t ErrorCheckCalculado = ErrorCheck(pacoteParcial,sizeof(pacoteParcial));
  // Serial.print("ErrorCheck Calculado: ");
  // Serial.println(ErrorCheckCalculado,HEX);

  // compara o ErrorCheck recebido e o ErrorCheck calculado
  if(ErrorCheckRebecido == ErrorCheckCalculado) valid = true; // se forem iguais retorna verdadeiro
  else valid = false; // se não, retorna falso

  return valid;
}

// função para printar vetores em hexadecimal
void MODBUS::printar(byte pacote[], byte tamanho){
  for(uint8_t i = 0; i<tamanho; i++)
    Serial.print((pacote[i] < 0x10) ? "0"+String(pacote[i],HEX) : String(pacote[i],HEX)+" ");
  Serial.println();
}