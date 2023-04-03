#include <Arduino.h>

class MODBUS{
    private:
        int pinoRS485Comunicacao;
        int Led; // para teste
        
        // Método que inverte os 4 bits menos significativos de posição
        uint16_t invert(uint16_t data);
    public:
        // Método que seta o pino de controle digital do RS485 e o pino digital do led para teste
        MODBUS(int RS485Comunicacao, int led);

        // Método que faz a requisição dos dados do UAC
        void EnviarPacote(byte EnderecoDoDispositivo,byte CodigoDaFuncao, uint16_t EnderecoInicial, uint16_t QuantidadeDeRegistradores);
        
        // Método que calcula o Error Check
        uint16_t ErrorCheck(byte mensagem[], uint8_t tamanho);

        // Método que verifica a integridade do pacote
        bool validacaoPacote(byte pacote[]);

        // função que converte os valores hexadecimais do formato IEEE754 para float
        float IEEE754_HexToFloat(byte dado[], int n1, int n2, int n3, int n4);

        // função para printar pacotes
        void printar(byte pacote[], byte tamanho);

        // variável booleana que indica se o pacote terminou
        bool stillWaitNextBit = false;
};