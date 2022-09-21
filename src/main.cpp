/* 

Controle de arrefecimento 2 estágios e ar condicionado com transdutor de pressão.
Autor: unBond 20/06/2022
Versão: 0.1

*/

/* 
AR Condicionado - Ainda a definir os valores

Usando as informações do pressostato de 4 vias do Vectra B como base, temos:

- Menos de 29PSI / 2 bar : Sistema sem carga, não aciona compressor
- Mais de 435PSI / 29,99 bar : Sobrepressão, desligar compressor. Rearme somente
abaixo de 275PSI / 18,96 bar.
- Mais de 285PSI / 19,65 bar : Ligar eletroventilador segunda velocidade.
Desligar abaixo de 215PS 14,8I.

https://www.clubedovectra.com.br/forum/viewtopic.php?t=18135


Sistema de arrefecimento - Referência astra 2006 FlexPower

A termostática abre com 87º/102 graus.
Primeiro estágio Arma com 102 graus, quando arma ventilador permanece ativo até 92 graus.
Segundo estágio Arma com 107 e permanece ativado até 102 graus.

*/

//------------------------------------------------------------------------------------
// Bibliotecas
//------------------------------------------------------------------------------------
#include <Arduino.h>


//------------------------------------------------------------------------------------
// Debug Geral -- Descomente para ativar o modo debug via serial
//------------------------------------------------------------------------------------
//#define MODULO_ARREF_AC

//------------------------------------------------------------------------------------
// Definicoes do sistema de arrefecimentio
//------------------------------------------------------------------------------------
// AC
#define TPS_MAX_AC 90                               // Valor máximo em % de tps - Plena carga
#define AC_PRESS_MIN_OFF 40                         // Valor em BAR Proteção por carga inssuficiente de fluído refrigerante    -  // Verificar
#define AC_PRESS_MIN_FAN_STG1_EN 130                // Valor em BAR Mínimo de pressão para acionar velocidade 1 - Verificar
#define AC_PRESS_MAX_FAN_STG1_EN 210                // Valor em BAR Máximo de pressão para manter  velocidade 1 - Verificar
#define AC_PRESS_MIN_FAN_STG2_EN 220                // Valor em BAR Mínimo de pressão para acionar velocidade 2 - Verificar
#define AC_PRESS_MAX_SAFE_OFF 280                   // Valor em BAR Proteção por excesso de pressao desligar o compressor
#define AC_COMP_HISTER 5                            // Valor em sec, histere entre ativações do compressor. 
// Arrefecimento
#define CLT_FAN_STG_1 100                           // Valor para acionar a primeira Velocidade
#define CLT_FAN_HIST_STG_1 92                       // Valor que será mantido uma vez ativado a velocidade 1 - histerese
#define CLT_FAN_STG_2 106                           // Valor para acionar a segunda Velocidade
#define CLT_FAN_HIST_STG_2  101                     // Valor que será mantido uma vez ativado a velocidade 2 - histerese

//------------------------------------------------------------------------------------
// I/O - microcontrolador - 2560
//------------------------------------------------------------------------------------
#define cltBkpPin A8  // Temperatura BKP se não houver CAN
#define acPressPin A6 // Transdutor A/C
#define acSolPin 2    // Solicitação AC 0.5V AC desligado / 12.5V AC LIGADO
#define sigAcPin 13   // Saida Compressor - Input PE1 MS2. //29 d13 pra testar
#define fanStg1Pin 30 // Fan estágio 1 - mosfet
#define fanStg2Pin 31 // Fan estágio 2 - mosfet

//------------------------------------------------------------------------------------
// Variaveis globais
//------------------------------------------------------------------------------------
// ISR
volatile unsigned long counter = 0;  // O valor máximo nesse projeto é  3. 

// Auxiliares e contadores
volatile int16_t cltBkp = 0;
volatile uint16_t acPress = 0 ;
volatile byte acSol = 0, fanStg1 = 0, fanStg2 = 0, statusComp = 0;
// Auxiliares de amostragem
volatile uint8_t indexEngTemp = 0, histComp = 0;
//------------------------------------------------------------------------------------
// Declaracao de todas as funcoes - protótipos
//-----------------------------------------------------------------------------------

void setup();
void loop();
void fanStgControl();
void acSis();
void compOn();
void compOff();
void ativaComp();
void desatComp();
void ativaFanStg1();
void desatFanStg1();
void ativaFanStg2();
void desatFanStg2();
ISR(TIMER1_OVF_vect);

//------------------------------------------------------------------------------------
// Setup
//------------------------------------------------------------------------------------
void setup() {

  noInterrupts();
  analogReference(DEFAULT); //Cuidado isso é referente a cada projeto.
  
  #if defined(MODULO_ARREF_AC)
  Serial.begin(115200);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  #endif
  //---Definicao dos pinos no microcontrolador
  pinMode(cltBkpPin, INPUT);
  pinMode(acPressPin, INPUT);
  pinMode(acSolPin, INPUT_PULLUP);
  pinMode(sigAcPin, OUTPUT);
  pinMode(fanStg1Pin, OUTPUT);
  pinMode(fanStg2Pin, OUTPUT);

  digitalWrite(sigAcPin, LOW);
  digitalWrite(fanStg1, LOW);
  digitalWrite(fanStg2, LOW);

  // Pré-scaller, timer
  TCCR1A = 0;
  TCCR1B |= ((1 << CS10) | (1 << CS11));
  TIMSK1 |= (1 << TOIE1);
  TCNT1 = 3036;

  interrupts();
}

//------------------------------------------------------------------------------------
// loop
//------------------------------------------------------------------------------------
void loop() {
#if defined(MODULO_ARREF_AC)
  Serial.println(" ");
  Serial.print("cltBkp: ");
  Serial.println(cltBkp);
  Serial.print("fanStg1: ");
  Serial.println(fanStg1);
  Serial.print("fanStrg2: ");
  Serial.println(fanStg2);
  //
  Serial.print("AcPress: ");
  Serial.println(acPress);
  Serial.print("acSol: ");
  Serial.println(acSol);  
  Serial.print("statusComp: ");
  Serial.println(statusComp);
  Serial.print("Histerese ");
  Serial.println(histComp);
  Serial.println(" ");
  delay(2000);
#endif
}
//--------- Fim loop  ---------//

//------------------------------------------------------------------------------------
// arref
//------------------------------------------------------------------------------------
void fanStgControl() {

  static int16_t smoothEngTemp = 0;
  static uint8_t fanControl = 0;

  indexEngTemp++;
  // estado de leitura

  //Aqui iremos usar 10 leituras e realizar a média delas, útil para sensores analógicos. 
  //Mesmo assim talvez seja interessante usar um pequeno capacitor nas entradas analógica para leitura mais suavizada.

  if (indexEngTemp <= 10) {
    // 
    smoothEngTemp = analogRead(cltBkpPin) + smoothEngTemp;
  }

  if (indexEngTemp == 10) {
    cltBkp = (int16_t)(smoothEngTemp / indexEngTemp);
    indexEngTemp = 0;
    smoothEngTemp = 0;

    cltBkp = (int16_t)map(cltBkp, 0, 1024, 120, -10);  
    // Amostragem obtida

    // Estrutura de controle de acionamento das velocidades dos ventiladores

    // Liga stg1
    if ((cltBkp >= CLT_FAN_STG_1) || (acPress >= AC_PRESS_MIN_FAN_STG1_EN)) {
      fanControl = 1;
      #if defined(MODULO_ARREF_AC)
      Serial.println("If 01 liga stg1");
      #endif
    }
    // Liga stg2
    if ((cltBkp >= CLT_FAN_STG_2) || (acPress >= AC_PRESS_MIN_FAN_STG2_EN)) {
      fanControl = 2;
      #if defined(MODULO_ARREF_AC)
      Serial.println("If 02 liga stg2");
      #endif
    }
    // retorno stg1 - histerese

    if (((fanControl == 2) && (cltBkp <= CLT_FAN_STG_1)) &&  (acPress <= AC_PRESS_MAX_FAN_STG1_EN)) { //parei aqui, lance do estagio 2 ac.
      fanControl = 1;
      #if defined(MODULO_ARREF_AC)
      Serial.println("If 03 fan = 1");
      #endif
    }

    // Desliga
    if (((fanControl >= 1) && (cltBkp <=CLT_FAN_HIST_STG_1)) && (acPress < AC_PRESS_MIN_FAN_STG1_EN)) {
      fanControl = 0;
      #if defined(MODULO_ARREF_AC)
      Serial.println("If 04 desliga");
      #endif
    }
    #if defined(MODULO_ARREF_AC)
    Serial.print("Fancontrol: ");
    Serial.println(fanControl);
    #endif
    
    switch (fanControl){
      case 2:
        ativaFanStg2();
        break;
      case 1:
        ativaFanStg1();
        break;
      case 0:
        desatFanStg1();
        desatFanStg2();    
        break;
    }
    
  }
}
//--------- Fim arref  ---------//

//------------------------------------------------------------------------------------
// acSis
//------------------------------------------------------------------------------------
void acSis() {
  acPress = analogRead(acPressPin); // Preciso de smooth aqui?
  acPress = (uint16_t)map(acPress, 0, 1024, 310, 0);  
  acSol = digitalRead(acSolPin);
}
//--------- Fim arref  ---------//

//------------------------------------------------------------------------------------
// compOn
//------------------------------------------------------------------------------------
void compOn() {
  if ((acPress <= AC_PRESS_MAX_SAFE_OFF || acPress > AC_PRESS_MIN_OFF) && acSol == true) {
    ativaComp();
  }
}
//--------- Fim compOn  ---------//

//------------------------------------------------------------------------------------
// compOff
//------------------------------------------------------------------------------------
void compOff() {
  if (acPress >= AC_PRESS_MAX_SAFE_OFF || acPress < AC_PRESS_MIN_OFF || acSol == false) {
    desatComp();
  }
}
//--------- Fim compOff  ---------//

//------------------------------------------------------------------------------------
// ativaComp
//------------------------------------------------------------------------------------
void ativaComp() {
  if (statusComp == false && histComp == AC_COMP_HISTER ) {
    digitalWrite(sigAcPin, HIGH);
    statusComp = true;
  }
}
//--------- ativaComp  ---------//

//------------------------------------------------------------------------------------
// desatComp
//------------------------------------------------------------------------------------
void desatComp() {
  if (statusComp == true) {
    digitalWrite(sigAcPin, LOW);
    statusComp = false;
    histComp = 0;
  }
}
//--------- desatComp  ---------//

//------------------------------------------------------------------------------------
// ativaFanStg1
//------------------------------------------------------------------------------------
void ativaFanStg1() {
  if (fanStg1 == false) {
    desatFanStg2(); // Garante que não acionaremos os relê ao mesmo tempo do STG
                    // 1 e STG 2.
    digitalWrite(fanStg1Pin, HIGH);
    #if defined(MODULO_ARREF_AC)
    Serial.println ("fan1 DW");
    #endif
    fanStg1 = true;
  }
}
//--------- Fim ativaFanStg1  ---------//

//------------------------------------------------------------------------------------
// desatFanStg1
//------------------------------------------------------------------------------------
void desatFanStg1() {
  if (fanStg1 == true) {
    digitalWrite(fanStg1Pin, LOW);
    #if defined(MODULO_ARREF_AC)
    Serial.println ("fan1 DW L");
    #endif
    fanStg1 = false;
  }
}
//--------- Fim desatFanStg1  ---------//

//------------------------------------------------------------------------------------
// ativaFanStg2
//------------------------------------------------------------------------------------
void ativaFanStg2() {
  if (fanStg2 == false) {
    desatFanStg1(); // Garante que não acionaremos os relê ao mesmo tempo do STG
                    // 1 e STG 2.
    digitalWrite(fanStg2Pin, HIGH);
    #if defined(MODULO_ARREF_AC)
    Serial.println ("fan2 DW");
    #endif
    fanStg2 = true;
  }
}
//--------- Fim ativaFanStg2  ---------//

//------------------------------------------------------------------------------------
// desatFanStg2
//------------------------------------------------------------------------------------
void desatFanStg2() {
  if (fanStg2 == true) {
    digitalWrite(fanStg2Pin, LOW);
    #if defined(MODULO_ARREF_AC)
    Serial.println ("fan2 DW L");
    #endif
    fanStg2 = false;
  }
}
//--------- Fim desatFanStg2  ---------//
//------------------------------------------------------------------------------------
// Timer principal
//------------------------------------------------------------------------------------
ISR(TIMER1_OVF_vect) {
  
  /* rotinas a cada 250ms */
  fanStgControl(); //Controle dos ventiladores
  acSis();

  counter++;
  // aqui se passou 250ms
  if (counter > 3) {

    /* rotinas a cada 1sec */
    // aqui se passou um segundo
    compOn();
    compOff();

    if (histComp < AC_COMP_HISTER )
      histComp++;
    counter = 0;
  }
  TCNT1 = 3036;
}
//--------- Fim Timer  ---------//