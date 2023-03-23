#include <Arduino.h>

class MODBUS{
    private:
        int pinoRS485Comunicacao;
        int Led; // para teste

        // Método que calcula o Error Check
        uint16_t ErrorCheck(byte mensagem[], uint8_t tamanho);

        // Método que inverte os 4 bits menos significativos de posição do ErrorCheck
        uint16_t invert(uint16_t data);
    public:
        // Método que seta o pino de controle digital do RS485 e o pino digital do led para teste
        MODBUS(int RS485Comunicacao, int led);

        // Método que faz a requisição dos dados do UAC
        void EnviarPacote(byte EnderecoDoDispositivo,byte CodigoDaFuncao, uint16_t EnderecoInicial, uint16_t QuantidadeDeRegistradores);
        


        // Método que verifica a integridade do pacote
        bool validacaoPacote(byte pacote[]);

        // função para printar pacotes
        void printar(byte pacote[], byte tamanho);

        // variável booleana que indica se o pacote terminou
        bool stillWaitNextBit = false;
};