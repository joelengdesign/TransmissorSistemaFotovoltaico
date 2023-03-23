#include <Arduino.h>

#include "MODBUS.h"

#define RegistradoresInput
// #define RegistradoresHolding

MODBUS::MODBUS(int RS485Comunicacao, int led){
    pinMode(RS485Comunicacao, OUTPUT);
    pinMode(led, OUTPUT);
    pinoRS485Comunicacao = RS485Comunicacao;
    Led = led;
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

    Serial.println("");
    Serial.print("Pacote de Requisição: ");
    printar(CodigoDeEnvio,sizeof(CodigoDeEnvio));

    digitalWrite(Led,HIGH);
    digitalWrite(pinoRS485Comunicacao,HIGH);
    
    for (byte i=0; i<sizeof(CodigoDeEnvio);i++){
        Serial1.write(CodigoDeEnvio[i]);
    }
    
    Serial1.flush();
    digitalWrite(pinoRS485Comunicacao,LOW);
    digitalWrite(Led,LOW);
    stillWaitNextBit = true;
    // Serial.print(stillWaitNextBit);
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
  crc = invert(crc);
  return crc;
}

uint16_t MODBUS::invert(uint16_t data){
  data = ((data & 0xff) << 8*1) | (data>>8*1);
  return data;
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
  
  // Extrai apenas o pacote sem os dois últimos bytes para calcular o ErrorCheck e comparar com o ErrorCheck recebido
  for(byte i=0;i<sizeof(pacoteParcial);i++){
    pacoteParcial[i] = pacote[i];
  }

  // calcula o ErrorCheck 
  uint16_t ErrorCheckCalculado = ErrorCheck(pacoteParcial,sizeof(pacoteParcial));

  // compara o ErrorCheck recebido e o ErrorCheck calculado
  if(ErrorCheckRebecido == ErrorCheckCalculado) valid = true; // se forem iguais retorna verdadeiro
  else valid = false; // se não, retorna falso

  return valid;
}

// função para printar vetores em hexadecimal
void MODBUS::printar(byte pacote[], byte tamanho){
  for(byte i=0; i<tamanho;i++){
      if(pacote[i]<0x10){
        Serial.print("0");Serial.print(pacote[i],HEX); Serial.print(" ");
      }
      else{
        Serial.print(pacote[i],HEX); Serial.print(" ");
      }
    }
    Serial.println(""); Serial.println("------------------");
}