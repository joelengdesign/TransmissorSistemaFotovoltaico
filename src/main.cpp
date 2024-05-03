/*
  Autor: Joel Carvalho - Mestre em Engenharia Elétrica pela Universidade Federal do Pará

  Descrição: Este código tem por objetivo receber as informações do UACT 3F por MODBUS e transmitir por LoRa
  
  Data de edição: 03/05/2024
  
  Versão: 2
  
  Notas adicionais: Protocolos envolvidos: MODBUS RTU/LoRa
*/

#include <Arduino.h>
#include <lmic.h> // biblioteca lmic para transmissão LoRa
#include <hal/hal.h>
#include <MODBUS.h> // biblioteca MODBUS criada para extração de dados da UACT 3F
#include <TimeLib.h> // biblioteca para converter timestamp no formato unix para hora e data
#include <softwareReset.hpp> // biblioteca para resetar o microcontrolador

MODBUS modbus(23);

// chaves de acesso AES128 da criptografia LoRaWAN
static const PROGMEM u1_t NWKSKEY[16] = {/* Chave Restrita */}; //projeto
static const u1_t PROGMEM APPSKEY[16] = {/* Chave Restrita */}; //projeto

// static const u1_t PROGMEM APPSKEY[16] = {0x88, 0x2a, 0x33, 0x04, 0xe1, 0xa9, 0x7d, 0xee, 0xc0, 0x1c, 0xd4, 0x1d, 0x59, 0xb0, 0x4c, 0x08 }; //projeto

static const u4_t DEVADDR = 0x26013803 ; // projeto <-- Change this address for every node!

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

const unsigned TX_INTERVAL = 4;
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

const byte aplicacao = 0x05; // código do sistema de baterias

// parâmetros do protocolo MODBUS
const byte DeviceAdress = 0x01; // endereço do dispositivo - UACT 3F
const byte TypeRegisters = 0x04; // código de acesso dos registradores do tipo input
const uint16_t InitAdress = 0x0004; // endereço do registrador inicial
const uint16_t QuantRegisters = 0x003C; // quantidade de registradores

// Definir um vetor para armazenar os valores de deslocamento
const int offsets[] = {5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 57, 61, 65, 69, 73, 77, 81, 85, 89, 93, 97, 101, 105};

// Definir os nomes das variáveis e a quantidade delas
const char* nomesVariaveis[] = {
    "Tensão RMS Fase A",
    "Tensão RMS Fase B",
    "Tensão RMS Fase C",
    "Corrente RMS Fase A",
    "Corrente RMS Fase B",
    "Corrente RMS Fase C",
    "Corrente RMS Neutro",
    "Frequencia Fase A",
    "Frequencia Fase B",
    "Frequencia Fase C",
    "Potencia Ativa Fase A",
    "Potencia Ativa Fase B",
    "Potencia Ativa Fase C",
    "Potencia Ativa Total",
    "Potencia Reativa FaseA",
    "Potencia ReativaFase B",
    "Potencia Reativa Fase C",
    "Potencia Reativa Total",
    "Potencia Aparente Fase A",
    "Potencia Aparente Fase B",
    "Potencia AparenteFase C",
    "Potencia Aparente Total",
    "Fator De Potencia Fase A",
    "Fator De Potencia Fase B",
    "Fator De Potencia Fase C",
    "Fator De Potencia Total"
};

union {
   uint8_t timesVector8[4] ;      //  4 bytes
   uint32_t times32 ;   //  mapped onto the same storage as myFloat
} timestamp;

// Quantidade de variáveis
const int numVariaveis = sizeof(offsets) / sizeof(offsets[0]);

// parâmetros dos dados recebidos da UAC CC
struct UACTStruct
{
  byte packetReceived[125];
  // byte receivedByteIndex = 0;
  byte PkgTotal[numVariaveis*4+5];
};
UACTStruct uactComponent;

// função que é chamada sempre que existem dados na serial do arduino para serem lidos
void serialEvent1(){
  if (modbus.stillWaitNextBit) while (Serial1.available()) Serial1.readBytes(uactComponent.packetReceived,125);
}

// função para controlar o tempo entre requisição e resposta à UACT CC
void espera(unsigned long tempo) {
  unsigned long inicio = millis();
  while (millis() - inicio < tempo) serialEvent1();
}

void esperaMinutos(int minutos) {
  unsigned long tempo_em_millis = minutos * 60000; // converte minutos em milissegundos
  unsigned long tempo_inicial = millis(); // obtém o tempo inicial em milissegundos
  while (millis() - tempo_inicial < tempo_em_millis) { // espera até que o tempo tenha passado
    // não faz nada
  }
}

// função que limpa o payload de resposta da UACT CC
void clearReceivedPackage(){
  for (byte i = 0; i < sizeof(uactComponent.packetReceived); i++) uactComponent.packetReceived[i] = 0x0;
}

// função que limpa o payload de transmissão com todos os dados tratados
void clearReceivedPackageTotal(){
  for (byte i = 0; i < sizeof(uactComponent.PkgTotal); i++) uactComponent.PkgTotal[i] = 0x0;
  // Serial.println("limpei o pacote Total");
}

// função que recebe o pacote da UACT CC
void OrganizePkg(){
  modbus.stillWaitNextBit = false;
  bool valid = modbus.validacaoPacote(uactComponent.packetReceived);
  uactComponent.PkgTotal[0] = aplicacao;

  if(valid){ // só entra neste bloco se o CRC do pacote de resposta da UACT CC estiver correto
    uactComponent.PkgTotal[0] = aplicacao;
    uactComponent.PkgTotal[1] = uactComponent.packetReceived[5];
    uactComponent.PkgTotal[2] = uactComponent.packetReceived[6];
    uactComponent.PkgTotal[3] = uactComponent.packetReceived[3];
    uactComponent.PkgTotal[4] = uactComponent.packetReceived[4];
    // variaveis de interesse
    for (byte i = 5; i<109; i=i+4){
      // i=40
      uactComponent.PkgTotal[i] = uactComponent.packetReceived[i+16]; //   21-5
      uactComponent.PkgTotal[i+1] = uactComponent.packetReceived[i+17]; // 22-5
      uactComponent.PkgTotal[i+2] = uactComponent.packetReceived[i+14]; // 19-5
      uactComponent.PkgTotal[i+3] = uactComponent.packetReceived[i+15]; // 20-5
    }
  }
  else{ // se o pacote não estiver correto reseta o arduino
    Serial.println("Pacote inválido");
    softwareReset::standard();
  }
  clearReceivedPackage();
}

void printTimes(){
  // timestamp
  for(uint8_t i=1; i<5;i++){
    timestamp.timesVector8[i-1] = uactComponent.PkgTotal[1];
  }

  setTime(timestamp.times32);
  
  int dia = day();
  int mes = month();
  int ano = year();
  int hora = hour();
  int minuto = minute();
  int segundo = second();

  // Imprime a data e hora obtidas
    if(dia<10){
      Serial.print("0");
    }
    Serial.print(dia);
    Serial.print("/");

    if(mes<10){
      Serial.print("0");
    }
    Serial.print(mes);
    Serial.print("/");
    Serial.println(ano);

    if(hora<10){
      Serial.print("0");
    }
    Serial.print(hora);
    Serial.print("h");

    if(minuto<10){
      Serial.print("0");
    }
    Serial.print(minuto);
    Serial.println("min");

    if(segundo<10){
      Serial.print("0");
    }
    Serial.print(segundo);
    Serial.println("min");
}

void printVarsFloat(){
  for (int i = 0; i < numVariaveis; i++) {
    Serial.print(nomesVariaveis[i]);
    Serial.print(": ");
    Serial.println(modbus.IEEE754_HexToFloat(uactComponent.PkgTotal, offsets[i])); 
  }
}

// Função de envio do pacote por LoRa.
void do_send(osjob_t* j) {

  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println("OP_TXRXPEND, not sending");
  }
  else {
    // acessando o timestamp nos registradores holding
    clearReceivedPackage();
    modbus.EnviarPacote(DeviceAdress,TypeRegisters,InitAdress,QuantRegisters); // envia a requisição
    espera(300); // aguarda 300 milissegundos
    // Serial.print("Pacote Recebido: ");
    // modbus.printar(uactComponent.packetReceived,sizeof(uactComponent.packetReceived)); // printa o pacote recebido da UACT CC
    OrganizePkg(); // organiza o pacote para transmissão LoRa
    Serial.println("");
    Serial.print("Pacote para envio: ");
    modbus.printar(uactComponent.PkgTotal,sizeof(uactComponent.PkgTotal)); // printa o pacote organizado
    
    printTimes();
    Serial.println();
    printVarsFloat();

    LMIC_setTxData2(1, uactComponent.PkgTotal, sizeof(uactComponent.PkgTotal), 0); // envia o pacote organizado por LoRa
    Serial.println("Pacote enviado");
    clearReceivedPackageTotal(); // limpa o pacote de envio para não transmitir a informação anterior, quando houver erros no pacote
    esperaMinutos(4);
  }
}
void onEvent (ev_t ev) {

  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT")); break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND")); break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED")); break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED")); break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING")); break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED")); break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1")); break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED")); break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED")); break;
    case EV_TXCOMPLETE:

      Serial.print(getSf(LMIC.rps));
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print("Data Received: ");
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}


void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);

  while (!Serial);

  delay(100);
  Serial.println("Sistema de Baterias. Enviando Dados...");
  Serial.println("---------------------------------------------");

#ifdef VCC_ENABLE
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif
  
  os_init();

  LMIC_reset();

#ifdef PROGMEM

  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else

  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif
  LMIC_selectSubBand(1);

  LMIC_setLinkCheckMode(0);

  LMIC_setAdrMode(0);

  /* esta função define a configuração LoRa de transmissão com o Spreading Factor (SF) e a potência de transmissão
  É importante destacar que esta função procura a melhor configuração de envio, de acordo com o tamanho do pacote
  Ela irá enviar no maior SF possível que suporte o  tamanho do pacote enviado, já que quanto maior o SF, maior alcance
  e qualidade na transmissão terá */
  LMIC_setDrTxpow(DR_SF10, 20);

  Serial.println(LMIC.datarate);
  do_send(&sendjob); // chama a função que executará o envio do pacote por LoRa
}

void loop() {
  os_runloop_once();
}