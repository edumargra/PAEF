/******************************
 *
 * Practica_04_PAE Connexió al robot
 * UB, 04/2018.
 *****************************/

#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include "lib_PAE2.h" //Libreria grafica + configuracion reloj MSP432

// VARIABLES GLOBALS ------------------------------------------------------------------------------------------

char saludo[16] = " PRACTICA 5 PAE"; //max 15 caracteres visibles
char cadena[16]; //Una linea entera con 15 caracteres visibles + uno oculto de terminacion de cadena (codigo ASCII 0)
char borrado[] = "               "; //una linea entera de 15 espacios en blanco
uint8_t linea = 1;
uint8_t estado = 0;
uint8_t estado_anterior = 8;
uint8_t moving = 0;
uint8_t Byte_Rebut = 0;
uint16_t comptador = 0;
uint16_t comptTimeOut = 0;
typedef uint8_t byte;
byte DadaLlegida_UART = 0;
uint16_t velocitat_lenta = 0x0155;
uint16_t velocitat_mitja = 0x0200;
uint16_t velocitat_rapida = 0x03ff;
uint8_t STOP_THRESHOLD = 200;
uint8_t threshold_center;
uint8_t lateral_detection_threshold = 20;
uint8_t min_threshold_lateral;
uint8_t max_threshold_lateral;

#define TXD2_READY (UCA2IFG & UCTXIFG)
struct RxReturn {
   byte StatusPacket[32];
   uint8_t timeOut;
   uint8_t checkSum;
};
struct Data {
   byte dades[3];
   uint8_t timeOut;
   uint8_t checkSum;
};

//IDs
uint8_t idML = 0x03;
uint8_t idMR = 0x02;
uint8_t idS = 0x64;

//Definicions (instruccions mòduls, adreces i constants)
#define WRITE 0x03
#define READ 0x02
#define CW 0x06
#define CCW 0x08
#define MOVING_SPEED 0x20
#define ENDAVANT 0x01
#define ENDARRERE 0x00
#define ESQUERRA 0x00
#define CENTRE 0X01
#define DRETA 0x02
#define SENTIT_CW 0x04
#define SENTIT_CCW 0x00
#define SENSOR_L 0
#define SENSOR_C 1
#define SENSOR_R 2


void init_timer_TA0()
{
    //Configurem el timer
    //***************************
    TA0CTL |= TASSEL__ACLK; //Seleccionem ACLK
    TA0CCR0 |= 32; //setejem el valor de CCR0 (el maxim del timer) per desenes de micro segons
    TA0CCTL0 |= CCIE; //Habilitem les interrupcions del timer
    TA0CCTL0 &= ~CCIFG; //Netegem el vector d'interrupcions

}

void init_timer_TA1()
{
    //Configurem el timer
    //***************************
    TA1CTL |= TASSEL__ACLK; //Seleccionem ACLK
    TA1CCR0 |= 32; //setejem el valor de CCR0 (el maxim del timer) per desenes de micro segons
    TA1CCTL0 |= CCIE; //Habilitem les interrupcions del timer
    TA1CCTL0 &= ~CCIFG; //Netegem el vector d'interrupcions

}

void activa_timerA0(){
    TA0CTL |= MC__UP; //seleccionem el mode up i iniciem el timer
}

void Activa_TimerA1(){
    TA1CTL |= MC__UP; //seleccionem el mode up i iniciem el timer
}

uint8_t TimeOut(uint32_t limit){
    //Funció que controla el timeout
    return comptTimeOut >= limit;
}

void Reset_TimeOut(void){
    comptTimeOut = 0;
}

void init_UART(void){
    UCA2CTLW0 |= UCSWRST; //Fem un reset de la USCI. Desactivem la USCI
    UCA2CTLW0 |= UCSSEL__SMCLK;
    UCA2MCTLW = UCOS16;
    UCA2BRW = 3;

    //Configurem els pins de la UART
    P3SEL0 |= BIT2 | BIT3;
    P3SEL1 &= ~(BIT2 | BIT3);
    P3SEL0 &= ~BIT0;
    P3SEL1 &= ~BIT0;
    P3DIR |= BIT0;
    P3OUT &= ~BIT0;
    UCA2CTLW0 &= ~UCSWRST;
    UCA2IE |= UCRXIE;   //Habilitem les interrupcions
}

void Sentit_Dades_Rx(void) {
    //Setejem el sentit de dades com a lectura
    P3OUT &= ~BIT0;
}

void Sentit_Dades_Tx(void) {
    //Setejem el sentit de dades com a escriptura
    P3OUT |= BIT0;
}

void TxUAC2(byte bTxdData) {
    while(!TXD2_READY);
    UCA2TXBUF = bTxdData;
}

byte TxPacket(byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16]) {
    //escriptura d'una trama
    if (Parametros[0] >= 0x06) {    //Controlem no accedir a posicions potencialment perilloses
        byte bCount, bCheckSum, bPacketLength;
        byte TxBuffer[32];
        Sentit_Dades_Tx();
        TxBuffer[0] = 0xff;
        TxBuffer[1] = 0xff;
        TxBuffer[2] = bID;
        TxBuffer[3] = bParameterLength+2;
        TxBuffer[4] = bInstruction;
        for (bCount = 0; bCount < bParameterLength; bCount++) {
            TxBuffer[bCount+5] = Parametros[bCount];
        }
        bCheckSum = 0;
        bPacketLength = bParameterLength + 4 + 2;
        for (bCount = 2; bCount < bPacketLength - 1; bCount++) {
            bCheckSum += TxBuffer[bCount];
        }
        TxBuffer[bCount] = ~bCheckSum;
        for (bCount = 0; bCount < bPacketLength; bCount++) {
            TxUAC2(TxBuffer[bCount]);
        }
        while((UCA2STATW&UCBUSY));
        Sentit_Dades_Rx();
        return(bPacketLength);
    }
    return NULL;
}

struct RxReturn RxPacket(void){
    struct RxReturn respuesta;
    respuesta.timeOut = 0;
    respuesta.checkSum = 0;
    byte bCount, bPacketLength, bCheckSum;
    Sentit_Dades_Rx(); //ponemos la linea half duplex en Rx
    Activa_TimerA1();
    for(bCount = 0; bCount < 4; bCount++)
    {
        Reset_TimeOut();
        Byte_Rebut = 0; //No_se_ha_recibido_byte();
        while(!Byte_Rebut)//se ha recibido un byte
        {
            respuesta.timeOut=TimeOut(2000);
            if(respuesta.timeOut)
                break;
        }
        if(respuesta.timeOut)
            break;
        respuesta.StatusPacket[bCount] = DadaLlegida_UART;
    }
    if(respuesta.timeOut)
        return respuesta;

    bPacketLength = DadaLlegida_UART + 4;
    for(;bCount < bPacketLength; bCount++){
        Reset_TimeOut();
        Byte_Rebut = 0; //No_se_ha_recibido_byte();
        while(!Byte_Rebut)//se ha recibido un byte
        {
            respuesta.timeOut=TimeOut(2000);
            if(respuesta.timeOut)
                break;
        }
        if(respuesta.timeOut)
            break;
        respuesta.StatusPacket[bCount] = DadaLlegida_UART;
    }
    if(respuesta.timeOut)
        return respuesta;

    bCheckSum = 0;
    for (bCount = 2; bCount < bPacketLength - 1; bCount++) {
        bCheckSum += respuesta.StatusPacket[bCount];
    }
    bCheckSum = ~bCheckSum;

    if(bCheckSum != respuesta.StatusPacket[bPacketLength -1])
        respuesta.checkSum = 1;

    return respuesta;
}

struct RxReturn read_IR_sensor(void) {
    byte parametres[2];
    parametres[0] = 0x1A;  //Adreça Left IR Sensor Data
    parametres[1] = 0x03;   //Llegim tres bytes: Left, Center i Right
    TxPacket(idS,0x02,READ,parametres);
    return RxPacket();
}

struct Data distance_sensor(){
    struct RxReturn dataRead = read_IR_sensor();

    struct Data data;
    data.dades[0] = dataRead.StatusPacket[5]; //Guardem dada llegida pel sensor IR Left
    data.dades[1] = dataRead.StatusPacket[6]; //Guardem dada llegida pel sensor IR Center
    data.dades[2] = dataRead.StatusPacket[7]; //Guardem dada llegida pel sensor IR Right
    data.checkSum = dataRead.checkSum;
    data.timeOut = dataRead.timeOut;

    return data;
}

void process_distance(uint8_t sensor) {
    struct Data data = distance_sensor();
    byte value = data.dades[sensor];

    if (data.checkSum){
        sprintf(cadena, "error checksum"); // Guardamos en cadena la siguiente frase: error checksum
        escribir(cadena, linea+1);          // Escribimos la cadena al LCD
        //LED RGB blanc
        P2OUT |= 0x50;  //Leds P2.4 (G), 2.6 (R) a 1 (encesos)
        P5OUT |= 0x40; //Led P5.6(B) a 1 (ences)
    } else if (data.timeOut) {
        sprintf(cadena, "error timeout"); // Guardamos en cadena la siguiente frase: error timeout
        escribir(cadena, linea+1);          // Escribimos la cadena al LCD
        //LED RGB blanc
        P2OUT |= 0x50;  //Leds P2.4 (G), 2.6 (R) a 1 (encesos)
        P5OUT |= 0x40; //Led P5.6(B) a 1 (ences)
    } else if (value < 40) {
        //LED RGB verd
        P2OUT |= 0x10;  //Led P2.4 (G) a 1 (ences)
        P5OUT &= ~0x40; //Led P5.6(B) a 0 (apagat)
        P2OUT &= ~0x40;  //Led P2.6 (R) a 0 (apagat)
    } else if (value < 200) {
        //LED RGB groc
        P2OUT |= 0x40;  //Led P2.6 (R) a 1 (ences)
        P2OUT |= 0x10;  //Led P2.4 (G) a 1 (ences)
        P5OUT &= ~0x40; //Led P5.6 (B) a 0 (apagat)
    } else {
        //LED RGB vermell
        P2OUT |= 0x40;  //Led P2.6 (R) a 1 (ences)
        P2OUT &= ~0x10;  //Led P2.4 (G) a 0 (apagat)
        P5OUT &= ~0x40; //Led P5.6(B) a 0 (apagat)
    }
    sprintf(cadena, "LEFT %03d", data.dades[0]); // Guardamos en cadena la siguiente frase: error checksum
    escribir(cadena, linea+2);          // Escribimos la cadena al LCD
    sprintf(cadena, "CENTER %03d", data.dades[1]); // Guardamos en cadena la siguiente frase: error checksum
    escribir(cadena, linea+3);          // Escribimos la cadena al LCD
    sprintf(cadena, "RIGHT %03d", data.dades[2]); // Guardamos en cadena la siguiente frase: error checksum
    escribir(cadena, linea+4);          // Escribimos la cadena al LCD
}

void activate_led(void) {
    byte parametres[2];
    parametres[0] = 0x19;
    parametres[1] = 0x01;
    TxPacket(idML,0x02,WRITE,parametres);
    TxPacket(idMR,0x02,WRITE,parametres);
}

void move_motor(uint8_t id, uint8_t sentit, uint16_t velocitat){
    byte parametres[3];
    parametres[0] = MOVING_SPEED; //adreça MOVING_SPEED
    parametres[1] = velocitat & 0xff; //parse de la velocitat(registre low) utilitzant una mascara i operacio binaria
    parametres[2] = (velocitat >> 8) & 0xff;//parse de la velocitat(registre high) utilitzant una mascara i operacio binaria
    parametres[2] |= sentit; //direction
    TxPacket(id,0x03,WRITE,parametres);
    RxPacket();
}

void stop_motor(uint8_t id){
    byte parametres[3];
    parametres[0] = MOVING_SPEED; //adreça MOVING_SPEED
    parametres[1] = 0x00; //speed value
    parametres[2] = 0x00; //speed value
    TxPacket(id,0x03,WRITE,parametres);
    RxPacket();
}

void move_robot(uint8_t sentit, uint16_t velocitat){
    if(sentit == ENDAVANT){
        move_motor(idML,SENTIT_CCW,velocitat);
        move_motor(idMR,SENTIT_CW,velocitat);
    } else {
        move_motor(idML,SENTIT_CW,velocitat);
        move_motor(idMR,SENTIT_CCW,velocitat);
    }
}

void hard_turn_robot(uint8_t sentit, uint16_t velocitat){
    if(sentit == ESQUERRA){
        stop_motor(idML);
        move_motor(idMR,SENTIT_CW,velocitat);
    } else {
        move_motor(idML,SENTIT_CCW,velocitat);
        stop_motor(idMR);
    }
}

void turn_robot(uint8_t sentit, uint16_t velocitat){
    if(sentit == ESQUERRA){
        move_motor(idML,SENTIT_CCW,velocitat);
        move_motor(idMR,SENTIT_CW,velocitat_mitja);
    } else {
        move_motor(idML,SENTIT_CW,velocitat_mitja);
        move_motor(idMR,SENTIT_CCW,velocitat);
    }
}

// TODO: gir 90 graus (funcions move_motor_angle)
void turn_robot_90_degrees(uint8_t direction, uint16_t velocitat) {
    if(sentit == ESQUERRA){
        move_motor(idML,SENTIT_CW,velocitat);
        move_motor(idMR,SENTIT_CW,velocitat);
    } else {
        move_motor(idML,SENTIT_CCW,velocitat);
        move_motor(idMR,SENTIT_CCW,velocitat);
    }
}

void stop_robot() {
    moving = 0;
    stop_motor(idML);
    stop_motor(idMR);
}

struct RxReturn read_sound_sensor(void) {
    byte parametres[2];
    parametres[0] = 0x25;  //Adreça Sound Count
    parametres[1] = 0x01;   //Llegim un byte
    TxPacket(idS,0x02,READ,parametres);
    return RxPacket();
}

struct Data sound_sensor() {
    struct RxReturn dataRead = read_sound_sensor();
    struct Data data;

    data.timeOut = dataRead.timeOut;
    data.checkSum = dataRead.checkSum;
    data.dades[0] = dataRead.StatusPacket[5]; //Guardem dada llegida pel sensor
    return data;
}

struct RxReturn read_luminosity_sensor(void) {
    byte parametres[2];
    parametres[0] = 0x1D;  //Adreça Luminosity
    parametres[1] = 0x03;   //Llegim tres bytes (un per a cada sensor: Left, Center i Right)
    TxPacket(idS,0x02,READ,parametres);
    return RxPacket();
}

struct Data luminosity_sensor(uint8_t sensor) {
    struct RxReturn dataRead = read_luminosity_sensor();
    struct Data data;

    data.timeOut = dataRead.timeOut;
    data.checkSum = dataRead.checkSum;
    data.dades[0] = dataRead.StatusPacket[5]; //Guardem dada llegida pel sensor Left
    data.dades[1] = dataRead.StatusPacket[6]; //Guardem dada llegida pel sensor Center
    data.dades[2] = dataRead.StatusPacket[7]; //Guardem dada llegida pel sensor Right
    return data;
}

void read_distance_wall(uint8_t sensor) {
    struct Data data = distance_sensor();
    uint8_t distancia = data.dades[sensor];
    if (sensor == CENTRE) {
        threshold_center = distancia;
        sprintf(cadena, "Center done "); // Guardamos en cadena la siguiente frase: error checksum
        escribir(cadena, linea+5);          // Escribimos la cadena al LCD
    } else {
        min_threshold_lateral = distancia;
        max_threshold_lateral = distancia + distancia/10;
        sprintf(cadena, "Lateral done"); // Guardamos en cadena la siguiente frase: error checksum
        escribir(cadena, linea+5);          // Escribimos la cadena al LCD
    }
}

/*uint8_t move_objects () {
    struct Data data = distance_sensor();

    //Delay
    uint8_t t = 0;
    while(t < 100){
        t++;
    }

    if (data.checkSum){
        sprintf(cadena, "error checksum"); // Guardamos en cadena la siguiente frase: error checksum
        escribir(cadena, linea+1);          // Escribimos la cadena al LCD
        return 1;
    } else if (data.timeOut) {
        sprintf(cadena, "error timeout"); // Guardamos en cadena la siguiente frase: error timeout
        escribir(cadena, linea+1);          // Escribimos la cadena al LCD
        return 1;
    }

    //casualistica del wall-follower algorithm
    if (data.dades[1] > STOP_THRESHOLD) {
        stop_robot();
    }else if (data.dades[1] > threshold_center){
        object_center = 1;
        hard_turn_robot(ESQUERRA, velocitat_lenta);
    }else if (data.dades[2] > max_threshold_lateral ){
        hard_turn_robot(ESQUERRA, velocitat_lenta);
        object_center = 0;
    }else if (data.dades[2] < min_threshold_lateral && !object_center){
        turn_robot(DRETA, velocitat_lenta);
    }else{
        move_robot(ENDAVANT, velocitat_lenta);
    }

    return 0;
}*/

/**************************************************************************
 * INICIALIZACIï¿½N DEL CONTROLADOR DE INTERRUPCIONES (NVIC).
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 **************************************************************************/

void init_interrupciones()
{
    // Configuracion al estilo MSP430 "clasico":
    // Enable Port 4 interrupt on the NVIC
    // segun datasheet (Tabla "6-12. NVIC Interrupts", capitulo "6.6.2 Device-Level User Interrupts", p80-81 del documento SLAS826A-Datasheet),
    // la interrupcion del puerto 4 es la User ISR numero 38.
    // Segun documento SLAU356A-Technical Reference Manual, capitulo "2.4.3 NVIC Registers"
    // hay 2 registros de habilitacion ISER0 y ISER1, cada uno para 32 interrupciones (0..31, y 32..63, resp.),
    // accesibles mediante la estructura NVIC->ISER[x], con x = 0 o x = 1.
    // Asimismo, hay 2 registros para deshabilitarlas: ICERx, y dos registros para limpiarlas: ICPRx.

    //Int. port 3 = 37 corresponde al bit 5 del segundo registro ISER1:
    NVIC->ICPR[1] |= BIT5; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= BIT5; //y habilito las interrupciones del puerto
    //Int. port 4 = 38 corresponde al bit 6 del segundo registro ISERx:
    NVIC->ICPR[1] |= BIT6; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= BIT6; //y habilito las interrupciones del puerto
    //Int. port 5 = 39 corresponde al bit 7 del segundo registro ISERx:
    NVIC->ICPR[1] |= BIT7; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= BIT7; //y habilito las interrupciones del puerto

    NVIC->ICPR[0] |= BIT8; //Primer, ens assegurem que no quedi cap interrupció residual pendent pel timer TA0
    NVIC->ISER[0] |= BIT8; //Habilitem les interrupcions del timer A0 a nivell NVIC

    NVIC->ICPR[0] |= BITA; //Primer, ens assegurem que no quedi cap interrupció residual pendent pel timer TA1
    NVIC->ISER[0] |= BITA; //Habilitem les interrupcions del timer A1 a nivell NVIC

    NVIC->ICPR[0] |= BIT(18); //Primer, ens assegurem que no quedi cap interrupció residual pendent pel mòdul AX-S1
    NVIC->ISER[0] |= BIT(18); //Habilitem les interrupcions del mòdul AX-S1 a nivell NVIC

    __enable_interrupt(); //Habilitamos las interrupciones a nivel global del micro.
}

/**************************************************************************
 * INICIALIZACIï¿½N DE LA PANTALLA LCD.
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 **************************************************************************/
void init_LCD(void) {
    halLcdInit(); //Inicializar y configurar la pantallita
    halLcdClearScreenBkg(); //Borrar la pantalla, rellenando con el color de fondo
}

/**************************************************************************
 * BORRAR LINEA
 *
 * Datos de entrada: Linea, indica la linea a borrar
 *
 * Sin datos de salida
 *
 **************************************************************************/
void borrar(uint8_t Linea) {
    halLcdPrintLine(borrado, Linea, NORMAL_TEXT); //escribimos una linea en blanco
}

/**************************************************************************
 * ESCRIBIR LINEA
 *
 * Datos de entrada: Linea, indica la linea del LCD donde escribir
 *                   String, la cadena de caracteres que vamos a escribir
 *
 * Sin datos de salida
 *
 **************************************************************************/
void escribir(char String[], uint8_t Linea) {
    halLcdPrintLine(String, Linea, NORMAL_TEXT); //Enviamos la String al LCD, sobreescribiendo la Linea indicada.
}

/**************************************************************************
 * INICIALIZACIï¿½N DE LOS BOTONES & LEDS DEL BOOSTERPACK MK II.
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 **************************************************************************/
void init_botons(void) {
    //Configuramos botones y leds
    //***************************

    //Leds RGB del MK II:
    P2DIR |= 0x50;  //Pines P2.4 (G), 2.6 (R) como salidas Led (RGB)
    P5DIR |= 0x40;  //Pin P5.6 (B)como salida Led (RGB)
    P2OUT &= 0xAF;  //Inicializamos Led RGB a 0 (apagados)
    P5OUT &= ~0x40; //Inicializamos Led RGB a 0 (apagados)

    //Boton S1 del MK II:
    P5SEL0 &= ~0x02;   //Pin P5.1 como I/O digital,
    P5SEL1 &= ~0x02;   //Pin P5.1 como I/O digital,
    P5DIR &= ~0x02; //Pin P5.1 como entrada
    P5IES &= ~0x02;   // con transicion L->H
    P5IE |= 0x02;     //Interrupciones activadas en P5.1,
    P5IFG = 0;    //Limpiamos todos los flags de las interrupciones del puerto 5
    //P5REN: Ya hay una resistencia de pullup en la placa MK II

    //Boton S2 del MK II:
    P3SEL0 &= ~0x20;   //Pin P3.5 como I/O digital,
    P3SEL1 &= ~0x20;   //Pin P3.5 como I/O digital,
    P3DIR &= ~0x20; //Pin P3.5 como entrada
    P3IES &= ~0x20;   // con transicion L->H
    P3IE |= 0x20;   //Interrupciones activadas en P3.5
    P3IFG = 0;  //Limpiamos todos los flags de las interrupciones del puerto 3
    //P3REN: Ya hay una resistencia de pullup en la placa MK II

    //Configuramos los GPIOs del joystick del MK II:
    P4DIR &= ~(BIT1 + BIT5 + BIT7 );   //Pines P4.1, 4.5 y 4.7 como entrades,
    P4SEL0 &= ~(BIT1 + BIT5 + BIT7 );  //Pines P4.1, 4.5 y 4.7 como I/O digitales,
    P4SEL1 &= ~(BIT1 + BIT5 + BIT7 );
    P4REN |= BIT1 + BIT5 + BIT7;  //con resistencia activada
    P4OUT |= BIT1 + BIT5 + BIT7;  // de pull-up
    P4IE |= BIT1 + BIT5 + BIT7;   //Interrupciones activadas en P4.1, 4.5 y 4.7,
    P4IES &= ~(BIT1 + BIT5 + BIT7 ); //las interrupciones se generaran con transicion L->H
    P4IFG = 0;    //Limpiamos todos los flags de las interrupciones del puerto 4

    P5DIR &= ~(BIT4 + BIT5 );  //Pines P5.4 y 5.5 como entrades,
    P5SEL0 &= ~(BIT4 + BIT5 ); //Pines P5.4 y 5.5 como I/O digitales,
    P5SEL1 &= ~(BIT4 + BIT5 );
    P5IE |= BIT4 + BIT5;  //Interrupciones activadas en 5.4 y 5.5,
    P5IES &= ~(BIT4 + BIT5 );  //las interrupciones se generaran con transicion L->H
    P5IFG = 0;    //Limpiamos todos los flags de las interrupciones del puerto 4
    // - Ya hay una resistencia de pullup en la placa MK II
}

struct Data moving_state_selector() {
    Data dataRead = process_distance();
    
    if (dataRead.checkSum){
        sprintf(cadena, "error checksum"); // Guardamos en cadena la siguiente frase: error checksum
        escribir(cadena, linea+1);          // Escribimos la cadena al LCD
        return 1;
    } else if (dataRead.timeOut) {
        sprintf(cadena, "error timeout"); // Guardamos en cadena la siguiente frase: error timeout
        escribir(cadena, linea+1);          // Escribimos la cadena al LCD
        return 1;
    }

    if (wall_on_left(dataRead.dades)) {
        moving_state = 1;
    } else if (wall_on_right(dataRead.dades)) {
        moving_state = 2;
    } else if (wall_on_front(dataRead.dades)) {
        moving_state = 3;
    } else if (wall_on_front_left_right(dataRead.dades)) {
        moving_state = 4;
    } else {
        //Cap paret a davant
        moving_state = 5;
    }
    
    return dataRead;
}

void follow_left(Data dataRead) {
    
    if (dataRead.dades[0] > max_threshold_lateral) {
        hard_turn_robot(DRETA, velocitat_lenta);
    } else if (dataRead.dades[0] < min_threshold_lateral) {
        hard_turn_robot(ESQUERRA, velocitat_lenta);
    } else {
        move_robot(ENDAVANT, velocitat_lenta);
    }
    
}

void follow_right(Data dataRead) {
    
    if (dataRead.dades[2] > max_threshold_lateral) {
        hard_turn_robot(ESQUERRA, velocitat_lenta);
    } else if (dataRead.dades[2] < min_threshold_lateral) {
        hard_turn_robot(DRETA, velocitat_lenta);
    } else {
        move_robot(ENDAVANT, velocitat_lenta);
    }
    
}

void change_robot_direction(Data dataRead) {
    
    if (prev_moving_state == 1) { //Venim de paret a l'esquerra
        turn_robot(ESQUERRA, velocitat_lenta);
    } else if (prev_moving_state == 2) { //Venim de paret a la dreta
        turn_robot(ESQUERRA, velocitat_lenta);
    }
    
}

uint8_t delay (uint8_t limit) {
    return comptador < limit;
}

void go_back_find_way(Data dataRead) {

    uint8_t sensor;
    uint8_t direction;
    
    if (prev_moving_state == 1) { //Venim de paret a l'esquerra
        sensor = 2;
        direction = ESQUERRA;
    } else if (prev_moving_state == 2) { //Venim de paret a la dreta
        sensor = 0;
        direction = DRETA;
    }
    
    while (dataRead.dades[sensor] < lateral_detection_threshold) {
        move_robot(ENDARRERE, velocitat_lenta);
        dataRead = process_distance();
    }
    
    turn_robot_90_degrees(direction, velocitat_lenta);
    
    comptador = 0;
    while (delay(1000)); //Temps de gir 90 graus a velocitat_lenta
    
    move_robot(ENDAVANT, velocitat_lenta);
    
    comptador = 0;
    while (delay(1000)); //Temps de gir abans de trobar la paret de nou
}

void open_turn(Data dataRead) {
    comptador = 0;
    while (delay(1000)); //Petit delay per agafar millor la curva
    
    if (prev_moving_state == 1) { //Venim de paret a l'esquerra
        turn_robot(ESQUERRA, velocitat_lenta);
    } else if (prev_moving_state == 2) { //Venim de paret a la dreta
        turn_robot(DRETA, velocitat_lenta);
    }
}

void main(void)
{

    WDTCTL = WDTPW + WDTHOLD;           // Paramos el watchdog timer

    //Inicializaciones:
    init_ucs_24MHz();       //Ajustes del clock (Unified Clock System)
    init_botons();         //Configuramos botones y leds
    init_LCD();             // Inicializamos la pantalla
    init_timer_TA0();   //inicialitzem el timer TA0
    init_timer_TA1();   //inicialitzem el timer TA1
    init_interrupciones();  //Configurar y activar las interrupciones de los botones
    halLcdPrintLine(saludo, linea, INVERT_TEXT); //escribimos saludo en la primera linea
    linea++; //Aumentamos el valor de linea y con ello pasamos a la linea siguiente
    init_UART();
    uint8_t error = 0;
    Data dataRead;

    //Bucle principal (infinito):
    do
    {
        /*if(moving && comptador >= 50 ){
            process_distance(ESQUERRA);
            error = move_objects();
            comptador = 0;
        }*/
        if (estado_anterior != estado)// Dependiendo del valor del estado se encenderï¿½ un LED u otro.
        {
            sprintf(cadena, "estado %d", estado); // Guardamos en cadena la siguiente frase: estado "valor del estado"
            escribir(cadena, linea);          // Escribimos la cadena al LCD
            estado_anterior = estado; // Actualizamos el valor de estado_anterior, para que no estï¿½ siempre escribiendo.

            /**********************************************************+
            A RELLENAR POR EL ALUMNO BLOQUE switch ... case
            Para gestionar las acciones:
            Boton S1, estado = 1
            Boton S2, estado = 2
            Joystick left, estado = 3
            Joystick right, estado = 4
            Joystick up, estado = 5
            Joystick down, estado = 6
            Joystick center, estado = 7
            ***********************************************************/
            switch (estado) {
                case 1:
                    //Si polsem S1, encenem els 3 LEDs RGB segons la distància a la que es detecta un objecte amb el sensor
                    //process_distance(CENTRE);
                    read_distance_wall(DRETA);
                    break;

                case 2:
                    //Si polsem S2, el robot fa stop
                    read_distance_wall(CENTRE);
                    stop_robot();
                    break;

                case 3:
                    //Si polsem el joystick  a l'esquerra, el robot gira sobre si mateix cap a l'esquerra
                    turn_robot(ESQUERRA, velocitat_lenta);
                    break;

                case 4:
                    //Si polsem el joystick  a la dreta, el robot gira sobre si mateix cap a la dreta
                    turn_robot(DRETA, velocitat_lenta);
                    break;

                case 5:
                    //Si polsem el joystick  amunt, el robot avança
                    activa_timerA0();
                    move_robot(ENDAVANT, velocitat_lenta);
                    escribir(borrado, linea+5);          // Escribimos la cadena al LCD
                    break;

                case 6:
                    //Si polsem el joystick  avall, el robot retrocedeix
                    move_robot(ENDARRERE, velocitat_lenta);
                    break;

                case 7:
                    //Si polsem el joystick al centre, activem els leds dels motors
                    activate_led();
                    break;
            }
        }
        
        
        dataRead = moving_state_selector();
        
        if (moving_state != prev_moving_state){
            prev_moving_state = moving_state;
            
            switch (moving_state) {
            
                case 1:
                    //Paret a l'esquerra
                    follow_left(dataRead);
                    break;
                    
                case 2:
                    //Paret a la dreta
                    follow_right(dataRead);
                    break;
                    
                case 3:
                    //Trobat paret al davant
                    change_robot_direction(dataRead);
                    break;
                    
                case 4:
                    //Paret davant i dos costats
                    go_back_find_way(dataRead);
                    break;
                    
                case 5:
                    //Cap paret detectada
                    open_turn(dataRead);
                    break;
            }
        } 
    }
    while (1); //Condicion para que el bucle sea infinito
}

/**************************************************************************
 * RUTINAS DE GESTION DE LOS BOTONES:
 * Mediante estas rutinas, se detectaria que botonn se ha pulsado
 *
 * Sin Datos de entrada
 *
 * Sin datos de salida
 *
 * Actualizar el valor de la variable global estado
 *
 **************************************************************************/

//ISR para las interrupciones del puerto 3:
void PORT3_IRQHandler(void) { //interrupcion del pulsador S2
    uint8_t flag = P3IV; //guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P3IE &= 0xDF;  //interrupciones del boton S2 en port 3 desactivadas
    estado_anterior = 0;

    /**********************************************************+
    A RELLENAR POR EL ALUMNO
    Para gestionar los estados:
    Boton S1, estado = 1
    Boton S2, estado = 2
    Joystick left, estado = 3
    Joystick right, estado = 4
    Joystick up, estado = 5
    Joystick down, estado = 6
    Joystick center, estado = 7
    ***********************************************************/
    estado = 2;

    P3IE |= 0x20;   //interrupciones S2 en port 3 reactivadas
}

//ISR para las interrupciones del puerto 4:
void PORT4_IRQHandler(void) { //interrupciï¿½n de los botones. Actualiza el valor de la variable global estado.
    uint8_t flag = P4IV; //guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P4IE &= 0x5D;   //interrupciones Joystick en port 4 desactivadas
    estado_anterior = 0;

    /**********************************************************+
    A RELLENAR POR EL ALUMNO BLOQUE switch ... case
    Para gestionar los estados:
    Boton S1, estado = 1
    Boton S2, estado = 2
    Joystick left, estado = 3
    Joystick right, estado = 4
    Joystick up, estado = 5
    Joystick down, estado = 6
    Joystick center, estado = 7
    ***********************************************************/
    switch (flag)
    {
    case 0x04: //pin 1
        estado = 7; //center
        break;
    case 0x0C: //pin 5
        estado = 4; //right
        break;
    case 0x10: //pin 7
        estado = 3; //Left
        break;
    }

    /***********************************************
    * HASTA AQUI BLOQUE CASE
    ***********************************************/

    P4IE |= 0xA2;   //interrupciones Joystick en port 4 reactivadas
}

//ISR para las interrupciones del puerto 5:
void PORT5_IRQHandler(void) { //interrupciï¿½n de los botones. Actualiza el valor de la variable global estado.
    uint8_t flag = P5IV; //guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P5IE &= 0xCD;   //interrupciones Joystick y S1 en port 5 desactivadas
    estado_anterior = 0;

    /**********************************************************+
    A RELLENAR POR EL ALUMNO BLOQUE switch ... case
    Para gestionar los estados:
    Boton S1, estado = 1
    Boton S2, estado = 2
    Joystick left, estado = 3
    Joystick right, estado = 4
    Joystick up, estado = 5
    Joystick down, estado = 6
    Joystick center, estado = 7
    ***********************************************************/
    switch (flag)
    {
    case 0x0A: //pin 4
        estado = 5; //up
        break;
    case 0x0C: //pin 5
        estado = 6; //down
        break;
    case 0x04: //pin 1
        estado = 1; //polsador S1
        break;
    }

    /***********************************************
    * HASTA AQUI BLOQUE CASE
    ***********************************************/

    P5IE |= 0x32;   //interrupciones Joystick y S1 en port 5 reactivadas
}

void TA0_0_IRQHandler(void)
{
    TA0CCTL0 &= ~CCIE;//Convé inhabilitar la interrupció al començament
    comptador++;    //Incrementem la variable global comptador
    TA0CCTL0 &= ~CCIFG;//Hem de netejar el flagde la interrupció
    TA0CCTL0 |= CCIE;//S’ha d’habilitar la interrupció abans de sortir
}

//Interrupció timer TimeOut
void TA1_0_IRQHandler(void)
{
    TA1CCTL0 &= ~CCIE; //Convé inhabilitar la interrupció al començament
    comptTimeOut++;    //Incrementem variable global seg
    TA1CCTL0 &= ~CCIFG; //Hem de netejar el flag de la interrupció
    TA1CCTL0 |= CCIE; //S’ha d’habilitar la interrupció abans de sortir
}

//Interrupció dada llegida UART
void EUSCIA2_IRQHandler(void){
    UCA2IE &= ~UCRXIE; //Convé inhabilitar la interrupció al començament
    DadaLlegida_UART = UCA2RXBUF; //Guardem la dada llegida
    Byte_Rebut = 1;	//Posem la variable global de control a 1
    UCA2IE |= UCRXIE; //S’ha d’habilitar la interrupció abans de sortir
}
