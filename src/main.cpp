// Código de transmissão de dados da aplicação SISTEMA FOTOVOLTÁICO relacionada ao Smart Campus da UFPA
// última atualização em 29 de março de 2023

#include <Arduino.h>
#include <lmic.h> // biblioteca lmic para transmissão LoRa
#include <hal/hal.h>
#include <MODBUS.h> // biblioteca MODBUS criada para extração de dados da UACT 3F
#include <softwareReset.hpp> // biblioteca para resetar o microcontrolador

MODBUS modbus(23,LED_BUILTIN);

// chaves de acesso AES128 da criptografia LoRaWAN
static const PROGMEM u1_t NWKSKEY[16] = {/* Informação confidencial */}; //projeto

static const u1_t PROGMEM APPSKEY[16] = {/* Informação confidencial */}; //projeto

static const u4_t DEVADDR = 0x260138A8 ; // projeto <-- Change this address for every node!

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
const uint16_t InitAdress = 0x0000; // endereço do registrador inicial
const uint16_t QuantRegisters = 0x0040; // quantidade de registradores

// parâmetros dos dados recebidos da UAC CC
struct UACTStruct
{
  byte packetReceived[129];
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
  static unsigned long lastRunTime = 0; // variável estática para armazenar o tempo da última execução da função
  unsigned long currentTime = millis(); // obter o tempo atual

  if (currentTime - lastRunTime >= tempo) { // verificar se já passou o tempo especificado
    lastRunTime = currentTime; // atualizar o tempo da última execução
    return; // sair da função e continuar a execução do programa
  }

  // se o tempo ainda não passou, aguardar
  delay(tempo - (currentTime - lastRunTime));
  lastRunTime = millis(); // atualizar o tempo da última execução
}

// função que limpa o payload de resposta da UACT CC
void clearReceivedPackage(){
  for (byte i = 0; i < sizeof(uactComponent.packetReceived); i++) uactComponent.packetReceived[i] = 0x0;
  uactComponent.receivedByteIndex = 0;
  Serial.println("limpei o pacote");
}

// função que limpa o payload de transmissão com todos os dados tratados
void clearReceivedPackageTotal(){
  for (byte i = 0; i < sizeof(uactComponent.PkgTotal); i++) uactComponent.PkgTotal[i] = 0x0;
  Serial.println("limpei o pacote Total");
}

// função que recebe o pacote da UACT CC
void receivedPkgTimerCallback(){
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
      uactComponent.PkgTotal[i] = uactComponent.packetReceived[i+24]; //   21-5
      uactComponent.PkgTotal[i+1] = uactComponent.packetReceived[i+25]; // 22-5
      uactComponent.PkgTotal[i+2] = uactComponent.packetReceived[i+22]; // 19-5
      uactComponent.PkgTotal[i+3] = uactComponent.packetReceived[i+23]; // 20-5
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
    Serial.print("Pacote Recebido: ");
    modbus.printar(uactComponent.packetReceived,sizeof(uactComponent.packetReceived)); // printa o pacote recebido da UACT CC
    receivedPkgTimerCallback(); // organiza o pacote para transmissão LoRa
    Serial.println("");
    Serial.print("Pacote para envio: ");
    modbus.printar(uactComponent.PkgTotal,sizeof(uactComponent.PkgTotal)); // printa o pacote organizado
    
    LMIC_setTxData2(1, uactComponent.PkgTotal, sizeof(uactComponent.PkgTotal), 0); // envia o pacote organizado por LoRa
    Serial.println("Pacote enviado");
    clearReceivedPackageTotal(); // limpa o pacote de envio para não transmitir a informação anterior, quando houver erros no pacote
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