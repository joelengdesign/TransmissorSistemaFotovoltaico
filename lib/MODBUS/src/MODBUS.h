/* Classe MODBUS */
#include <Arduino.h>

class MODBUS{
    private:
        int pinoRS485Comunicacao;
    public:
        // Método que seta o pino de controle digital do RS485 e o pino digital do led para teste
        MODBUS(int RS485Comunicacao);

        // Método que faz a requisição dos dados do UAC
        void EnviarPacote(byte EnderecoDoDispositivo,byte CodigoDaFuncao, uint16_t EnderecoInicial, uint16_t QuantidadeDeRegistradores);
        
        // Método que calcula o Error Check
        uint16_t ErrorCheck(byte mensagem[], uint8_t tamanho);

        // Método que verifica a integridade do pacote
        bool validacaoPacote(byte pacote[]);

        // função que converte os valores hexadecimais do formato IEEE754 para float
        float IEEE754_HexToFloat(uint8_t dado[], int n);

        // função para printar pacotes
        void printar(byte pacote[], byte tamanho);

        // variável booleana que indica se o pacote terminou
        bool stillWaitNextBit = false;
};