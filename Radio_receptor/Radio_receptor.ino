///////////////////////////////////////////////////////////////////////////////
//                             Equipe Hawkings                               //
///////////////////////////////////////////////////////////////////////////////
// Desenvolvido por: Enrique, Felipe, Gustavo, Marcus, Mauro.                //
// Última att: 19/10/2021.                                                   //
///////////////////////////////////////////////////////////////////////////////
// Descrição:                                                                //
//                                                                           //
//  Esse código realiza a comunicação do Arduino Nano com o rádio receptor   //
//  FS82, através do protocolo IBus. E imprime os valores dos 6 canais no    //
//  monitor serial.                                                          //
//                                                                           //
//  Ps_1: Em nossos testes utilizamos o rádio controle FS-i6;                //
//  Ps_2: Como estamos utilizando o Arduino Nano e este conta com apenas     //
//  uma porta UART ao passar o código para o Arduino devemos desconectar     //
//  o pino IBus do receptor do Arduino e ao fim transferência do código      //
//  devemos conecta-lo novamente.                                            //
///////////////////////////////////////////////////////////////////////////////
//  Conexão:                                                                 //
//                                                                           //
//  FS82 - Arduino Nano                                                      //
//  IBus - RX0                                                               //
///////////////////////////////////////////////////////////////////////////////

/* 
canal 1 - roll -- stick horizontal direito
canal 2 - pitch -- stick vertical direito
canal 3 - throttle -- stick vertical esquerdo
canal 4 - yaw  -- stick horizontal esquerdo
canal 5 - switch C (swc) -- chave direita interna
canal 6 - switch D (swd) -- chave direita externa
*/

#include <IBusBM.h>

IBusBM sinal; // Cria um objeto "sinal" da classe "IBusBM"

void setup (){
    Serial.begin(115200); // abre a porta serial, configura a taxa de transferência para 115200 bps
    sinal.begin(Serial);
    Serial.println("Ligado");
}

void loop (){
    int valor;
    for(int i=0; i<6; i++){
        valor = sinal.readChannel(i); // readChannel (número do canal)
        Serial.print("Ch");
        Serial.print(i+1);
        Serial.print(": ");
        Serial.print(valor); // printa no monitor serial os valores de cada canal
        Serial.print(" | ");
    }
    Serial.println("");
}
