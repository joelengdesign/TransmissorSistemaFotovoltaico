// Código de transmissão de dados da aplicação SISTEMA DE BATERIAS relacionada ao Smart Campus da UFPA
// código editado em 03 de abril de 2023

#include <Arduino.h>
#include <lmic.h> // biblioteca lmic para transmissão LoRa
#include <hal/hal.h>
#include <MODBUS.h> // biblioteca MODBUS criada para extração de dados da UACT 3F
#include <TimeLib.h> // biblioteca para converter timestamp no formato unix para hora e data
#include <softwareReset.hpp> // biblioteca para resetar o microcontrolador

MODBUS modbus(23,LED_BUILTIN);

// chaves de acesso AES128 da criptografia LoRaWAN
static const PROGMEM u1_t NWKSKEY[16] = {/* Informação confidencial */}; //projeto

static const u1_t PROGMEM APPSKEY[16] = {/* Informação confidencial */}; //projeto

static const u4_t DEVADDR = 0x260138A0 ; // projeto <-- Change this address for every node!

union {
  float valor_float;
  uint32_t valor_inteiro;
} conversor;

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

const byte aplicacao = 0x03; // código do sistema de baterias

// parâmetros do protocolo MODBUS
const byte DeviceAdress = 0x01; // endereço do dispositivo - UACT 3F
const byte TypeRegisters = 0x04; // código de acesso dos registradores do tipo input
const uint16_t InitAdress = 0x0004; // endereço do registrador inicial
const uint16_t QuantRegisters = 0x003C; // quantidade de registradores

// variaveis da aplicação
float TensaoRMSFaseA;
float TensaoRMSFaseB;
float TensaoRMSFaseC;

float CorrenteRMSFaseA;
float CorrenteRMSFaseB;
float CorrenteRMSFaseC;
float CorrenteRMSNeutro;

float FrequenciaFaseA;
float FrequenciaFaseB;
float FrequenciaFaseC;

float PotenciaAtivaFaseA;
float PotenciaAtivaFaseB;
float PotenciaAtivaFaseC;
float PotenciaAtivaTotal;

float PotenciaReativaFaseA;
float PotenciaReativaFaseB;
float PotenciaReativaFaseC;
float PotenciaReativaTotal;

float PotenciaAparenteFaseA;
float PotenciaAparenteFaseB;
float PotenciaAparenteFaseC;
float PotenciaAparenteTotal;

float FatorDePotenciaFaseA;
float FatorDePotenciaFaseB;
float FatorDePotenciaFaseC;
float FatorDePotenciaTotal;

float ConsumoFaseA;
float ConsumoFaseB;
float ConsumoFaseC;
float ConsumoTotal;

// parâmetros dos dados recebidos da UAC CC
struct UACTStruct
{
  byte packetReceived[125];
  byte receivedByteIndex = 0;
  byte PkgTotal[125];
};
UACTStruct uactComponent;

// função que é chamada sempre que existem dados na serial do arduino para serem lidos
void serialEvent1(){
  byte incomingByte;
  // Serial.println(stillWaitNextBit);
  if (modbus.stillWaitNextBit)
  {
    while (Serial1.available()) {
      incomingByte = Serial1.read();
      // Serial.print(incomingByte, HEX);
      // Serial.print(' ');
      uactComponent.packetReceived[uactComponent.receivedByteIndex] = incomingByte;
      uactComponent.receivedByteIndex += 1;
    }
  }
}

// função para controlar o tempo entre requisição e resposta à UACT CC
void espera(unsigned long tempo) {
  unsigned long inicio = millis();
  while (millis() - inicio < tempo) {
    serialEvent1(); // Execute a função serialEvent1 durante o tempo de espera
  }
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
  uactComponent.receivedByteIndex = 0;
  // Serial.println("limpei o pacote");
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

    // preenchendo com zero as variáveis com zeros
    for (byte i=109; i<125; i++){
      uactComponent.PkgTotal[i] = 0x00;
    }
  }
  else{ // se o pacote não estiver correto reseta o arduino
    Serial.println("Pacote inválido");
    softwareReset::standard();
  }
  clearReceivedPackage();
}

void printarVars(){
    // timestamp
    unsigned long timestamp = ((unsigned long)uactComponent.PkgTotal[1] << 24) |  // Desloca o primeiro byte 24 bits para a esquerda
                          ((unsigned long)uactComponent.PkgTotal[2] << 16) |  // Desloca o segundo byte 16 bits para a esquerda
                          ((unsigned long)uactComponent.PkgTotal[3] << 8)  |  // Desloca o terceiro byte 8 bits para a esquerda
                          ((unsigned long)uactComponent.PkgTotal[4]);        // Não precisa deslocar

    setTime(timestamp);
    
    int dia = day();
    int mes = month();
    int ano = year();
    int hora = hour();
    int minuto = minute();

    // variaveis

    TensaoRMSFaseA = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,5,6,7,8);
    TensaoRMSFaseB = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,9,10,11,12);
    TensaoRMSFaseC = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,13,14,15,16);

    CorrenteRMSFaseA = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,17,18,19,20);
    CorrenteRMSFaseB = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,21,22,23,24);
    CorrenteRMSFaseC = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,25,26,27,28);
    CorrenteRMSNeutro = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,29,30,31,32);

    FrequenciaFaseA = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,33,34,35,36);
    FrequenciaFaseB = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,37,38,39,40);
    FrequenciaFaseC = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,41,42,43,44);

    PotenciaAtivaFaseA = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,45,46,47,48);
    PotenciaAtivaFaseB = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,49,50,51,52);
    PotenciaAtivaFaseC = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,53,54,55,56);
    PotenciaAtivaTotal = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,57,58,59,60);

    PotenciaReativaFaseA = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,61,62,63,64);
    PotenciaReativaFaseB = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,65,66,67,68);
    PotenciaReativaFaseC = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,69,70,71,72);
    PotenciaReativaFaseC = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,73,74,75,76);

    PotenciaAparenteFaseA = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,77,78,79,80);
    PotenciaAparenteFaseB = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,81,82,83,84);
    PotenciaAparenteFaseC = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,85,86,87,88);
    PotenciaAparenteTotal = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,89,90,91,92);

    FatorDePotenciaFaseA = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,93,94,95,96);
    FatorDePotenciaFaseB = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,97,98,99,100);
    FatorDePotenciaFaseC = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,101,102,103,104);
    FatorDePotenciaTotal = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,105,106,107,108);

    ConsumoFaseA = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,109,110,111,112);
    ConsumoFaseB = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,113,114,115,116);
    ConsumoFaseC = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,117,118,119,120);
    ConsumoTotal = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,121,122,123,12);

    // Imprime a data e hora obtidas
    if(dia<10){
      Serial.print("0");
      Serial.print(dia);
    }
    else{
      Serial.print(dia);
    }
    Serial.print("/");
    if(mes<10){
      Serial.print("0");
      Serial.print(mes);
    }
    else{
      Serial.print(mes);
    }
    Serial.print("/");
    Serial.println(ano);
    if(hora<10){
      Serial.print("0");
      Serial.print(hora);
    }
    else{
      Serial.print(hora);
    }
    Serial.print("h");
    if(hora<10){
      Serial.print("0");
      Serial.print(minuto);
    }
    else{
      Serial.print(minuto);
    }
    Serial.println("min");

    Serial.print("Tensão RMS Fase A: ");
    Serial.println(TensaoRMSFaseA);

    Serial.print("Tensão RMS FaseB: ");
    Serial.println(TensaoRMSFaseB);

    Serial.print("Tensão RMS Fase C: ");
    Serial.println(TensaoRMSFaseC);

    Serial.print("Corrente RMS Fase A: ");
    Serial.println(CorrenteRMSFaseA);

    Serial.print("Corrente RMS Fase B: ");
    Serial.println(CorrenteRMSFaseB);

    Serial.print("Corrente RMS Fase C: ");
    Serial.println(CorrenteRMSFaseC);

    Serial.print("Corrente RMS Neutro: ");
    Serial.println(CorrenteRMSNeutro);

    Serial.print("Frequência Fase A: ");
    Serial.println(FrequenciaFaseA);

    Serial.print("FrequenciaFaseB: ");
    Serial.println(FrequenciaFaseB);

    Serial.print("Frequência Fase C: ");
    Serial.println(FrequenciaFaseC);

    Serial.print("Potência Ativa Fase A: ");
    Serial.println(PotenciaAtivaFaseA);

    Serial.print("Potência Ativa Fase B: ");
    Serial.println(PotenciaAtivaFaseB);

    Serial.print("Potência Ativa Fase C: ");
    Serial.println(PotenciaAtivaFaseC);

    Serial.print("Potência Ativa Total: ");
    Serial.println(PotenciaAtivaTotal);

    Serial.print("Potência Reativa Fase A: ");
    Serial.println(PotenciaReativaFaseA);

    Serial.print("Potência Reativa Fase B: ");
    Serial.println(PotenciaReativaFaseB);

    Serial.print("Potência Reativa Fase C: ");
    Serial.println(PotenciaReativaFaseC);

    Serial.print("Potência Reativa Total: ");
    Serial.println(PotenciaReativaTotal);

    Serial.print("Potência Aparente Fase A: ");
    Serial.println(PotenciaAparenteFaseA);

    Serial.print("Potência Aparente Fase B: ");
    Serial.println(PotenciaAparenteFaseB);

    Serial.print("Potência Aparente Fase C: ");
    Serial.println(PotenciaAparenteFaseC);

    Serial.print("Potência Aparente Total: ");
    Serial.println(PotenciaAparenteTotal);

    Serial.print("Fator de Potência Fase A: ");
    Serial.println(FatorDePotenciaFaseA);

    Serial.print("Fator de Potência Fase B: ");
    Serial.println(FatorDePotenciaFaseB);

    Serial.print("Fator de Potência Fase C: ");
    Serial.println(FatorDePotenciaFaseC);

    Serial.print("Fator de Potência Total: ");
    Serial.println(FatorDePotenciaTotal);

    Serial.print("Consumo Fase A: ");
    Serial.println(ConsumoFaseA);

    Serial.print("Consumo Fase B: ");
    Serial.println(ConsumoFaseB);

    Serial.print("ConsumoFaseC: ");
    Serial.println(ConsumoFaseC);

    Serial.print("Consumo Total: ");
    Serial.println(ConsumoTotal);
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
    
    printarVars();

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