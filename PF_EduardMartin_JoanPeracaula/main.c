/******************************
 *
 * Practica_04_PAE Connexi� al robot
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
uint8_t sideWall = 2;//per defecte que segueixi la paret dreta

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

//Definicions (instruccions m�duls, adreces i constants)
#define WRITE 0x03
#define READ 0x02
//motor
#define CW 0x06
#define CCW 0x08
#define MOVING_SPEED 0x20
#define SENTIT_CW 0x04
#define SENTIT_CCW 0x00
//direccions del robot
#define ENDAVANT 0x01
#define ENDARRERE 0x00
#define ESQUERRA 0x00
#define CENTRE 0X01
#define DRETA 0x02
//sensors
#define SENSOR_L 0
#define SENSOR_C 1
#define SENSOR_R 2
//estats del robot
#define JAILED 0
#define INTERIOR_TURN 1
#define EXTERIOR_TURN 2
#define TOO_CLOSE 3
#define TOO_FAR 4
#define GO_STRAIGHT 5
#define STOP 6
#define INIT 7

uint8_t moving_state = STOP;
uint8_t prev_moving_state = INIT;


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
    //Funci� que controla el timeout
    return comptTimeOut >= limit;
}

void Reset_TimeOut(void){
    comptTimeOut = 0;
}

uint8_t delay_timer (uint32_t limit) {
    return comptador < limit;
}

void reset_delay_timer() {
  comptador = 0;
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
    parametres[0] = 0x1A;  //Adreca Left IR Sensor Data
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

void process_distance(struct Data data) {

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
    }
    sprintf(cadena, "LEFT %03d", data.dades[0]); // Guardamos en cadena la siguiente frase: error checksum
    escribir(cadena, linea+2);          // Escribimos la cadena al LCD
    sprintf(cadena, "CENTER %03d", data.dades[1]); // Guardamos en cadena la siguiente frase: error checksum
    escribir(cadena, linea+3);          // Escribimos la cadena al LCD
    sprintf(cadena, "RIGHT %03d", data.dades[2]); // Guardamos en cadena la siguiente frase: error checksum
    escribir(cadena, linea+4);          // Escribimos la cadena al LCD
}

//activem els leds dels motors
void activate_led(void) {
    byte parametres[2];
    parametres[0] = 0x19;
    parametres[1] = 0x01;
    TxPacket(idML,0x02,WRITE,parametres);
    TxPacket(idMR,0x02,WRITE,parametres);
}

/* Metode per fer que un motor es mogui indefinidament.
*  Te com a parametre la id del motor, el sentit i la velocitat a la que volem que es mogui
*/
void move_motor(uint8_t id, uint8_t sentit, uint16_t velocitat){
    byte parametres[3];
    parametres[0] = MOVING_SPEED; //adre�a MOVING_SPEED
    parametres[1] = velocitat & 0xff; //parse de la velocitat(registre low) utilitzant una mascara i operacio binaria
    parametres[2] = (velocitat >> 8) & 0xff;//parse de la velocitat(registre high) utilitzant una mascara i operacio binaria
    parametres[2] |= sentit; //direction
    TxPacket(id,0x03,WRITE,parametres);
    RxPacket();
}

/*Para un motor
* Te com a parametre la id del motor
*/
void stop_motor(uint8_t id){
    byte parametres[3];
    parametres[0] = MOVING_SPEED; //adre�a MOVING_SPEED
    parametres[1] = 0x00; //speed value
    parametres[2] = 0x00; //speed value
    TxPacket(id,0x03,WRITE,parametres);
    RxPacket();
}

/* Mou el robot indefinidament cap a una direccio.
*  Te com a parametres la direccio i la velocitat a la que volem que es mogui.
*/
void move_robot(uint8_t sentit, uint16_t velocitat){
    if(sentit == ENDAVANT){
        move_motor(idML,SENTIT_CCW,velocitat);
        move_motor(idMR,SENTIT_CW,velocitat);
    } else {
        move_motor(idML,SENTIT_CW,velocitat);
        move_motor(idMR,SENTIT_CCW,velocitat);
    }
}

/* Gir tancat que te com a eix una de les rodes del robot.
*  Te com a parametres el sentit del gir i la velocitat
*/
void hard_turn_robot(uint8_t sentit, uint16_t velocitat){
    if(sentit == ESQUERRA){
        stop_motor(idML);
        move_motor(idMR,SENTIT_CW,velocitat);
    } else {
        move_motor(idML,SENTIT_CCW,velocitat);
        stop_motor(idMR);
    }
}

/* Gir natural del robot.
*  Te com parametres el sentit i la velocitat del gir.
*/
void turn_robot(uint8_t sentit, uint16_t velocitat){
    if(sentit == ESQUERRA){
        move_motor(idML,SENTIT_CCW,velocitat);
        move_motor(idMR,SENTIT_CW,velocitat_mitja);
    } else {
        move_motor(idML,SENTIT_CCW,velocitat_mitja);
        move_motor(idMR,SENTIT_CW,velocitat);
    }
}

// TODO: gir 90 graus (funcions move_motor_angle)
void turn_robot_90_degrees(uint8_t direction, uint16_t velocitat) {
    if(direction == ESQUERRA){
        move_motor(idML,SENTIT_CW,velocitat);
        move_motor(idMR,SENTIT_CW,velocitat);
    } else {
        move_motor(idML,SENTIT_CCW,velocitat);
        move_motor(idMR,SENTIT_CCW,velocitat);
    }
}

//Para el robot
void stop_robot() {
    stop_motor(idML);
    stop_motor(idMR);
}

struct RxReturn read_sound_sensor(void) {
    byte parametres[2];
    parametres[0] = 0x25;  //Adre�a Sound Count
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
    parametres[0] = 0x1D;  //Adre�a Luminosity
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

/* Metode que utilitzem per inicialitzar els valors dels tresholds dels diferents sensors.
*  Diferenciem entre el del centre i els laterals, tenin especial cura d aquest ultims, afegint
*  una tolerancia d un 10 per cent.
*/
void read_distance_wall(uint8_t sensor) {
    struct Data data = distance_sensor();
    uint8_t distancia = data.dades[sensor];
    if (sensor == CENTRE) {
        threshold_center = distancia;
        sprintf(cadena, "Center: %03d", distancia); // Guardamos en cadena la siguiente frase: error checksum
        escribir(cadena, linea+5);          // Escribimos la cadena al LCD
    } else {
        min_threshold_lateral = distancia;
        max_threshold_lateral = distancia + distancia/10;
        sprintf(cadena, "Lateral: %03d", distancia); // Guardamos en cadena la siguiente frase: error checksum
        escribir(cadena, linea+6);          // Escribimos la cadena al LCD
    }
}

/**************************************************************************
 * INICIALIZACI�N DEL CONTROLADOR DE INTERRUPCIONES (NVIC).
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

    NVIC->ICPR[0] |= BIT8; //Primer, ens assegurem que no quedi cap interrupci� residual pendent pel timer TA0
    NVIC->ISER[0] |= BIT8; //Habilitem les interrupcions del timer A0 a nivell NVIC

    NVIC->ICPR[0] |= BITA; //Primer, ens assegurem que no quedi cap interrupci� residual pendent pel timer TA1
    NVIC->ISER[0] |= BITA; //Habilitem les interrupcions del timer A1 a nivell NVIC

    NVIC->ICPR[0] |= BIT(18); //Primer, ens assegurem que no quedi cap interrupci� residual pendent pel m�dul AX-S1
    NVIC->ISER[0] |= BIT(18); //Habilitem les interrupcions del m�dul AX-S1 a nivell NVIC

    __enable_interrupt(); //Habilitamos las interrupciones a nivel global del micro.
}

/**************************************************************************
 * INICIALIZACI�N DE LA PANTALLA LCD.
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
 * INICIALIZACI�N DE LOS BOTONES & LEDS DEL BOOSTERPACK MK II.
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

uint8_t wall_jailing(byte dades[3]){ //jail
  return dades[0] > max_threshold_lateral && dades[1] > threshold_center && dades[2] > max_threshold_lateral;
}
uint8_t wall_narrow_path(byte dades[3]){ //narrowing sides
  return dades[0] > max_threshold_lateral && dades[2] > max_threshold_lateral ;
}
uint8_t wall_on_front(byte dades[3]){ //inner turn
  return dades[1] >= threshold_center;
}
uint8_t wall_on_side_TOO_FAR(byte dades[3]){ //get farther from the wall
  return dades[sideWall] >= max_threshold_lateral;
}
uint8_t wall_on_side_too_far(byte dades[3]){ //get closer to the wall
  return dades[sideWall] < min_threshold_lateral;
}
uint8_t wall_on_side_perfect(byte dades[3]){ //go straight
  return dades[sideWall] < max_threshold_lateral && dades[sideWall] > min_threshold_lateral;
}

uint8_t no_wall(byte dades[3]){
  return dades[sideWall] < lateral_detection_threshold && dades[1] < threshold_center;
}

struct Data moving_state_selector() {
    struct Data dataRead = distance_sensor(); //mirem l'entorn del robot

    if (dataRead.checkSum){
        sprintf(cadena, "error checksum"); // Guardamos en cadena la siguiente frase: error checksum
        escribir(cadena, linea+1);          // Escribimos la cadena al LCD

    } else if (dataRead.timeOut) {
        sprintf(cadena, "error timeout"); // Guardamos en cadena la siguiente frase: error timeout
        escribir(cadena, linea+1);          // Escribimos la cadena al LCD
    }

    else if (wall_jailing(dataRead.dades)) {
        moving_state = JAILED; //aquest cas sempre voldra dir que esta en un lloc sense sortida
    } else if (wall_narrow_path(dataRead.dades)){
        moving_state = JAILED; //les parets al dos costat son massa estretes
    } else if (wall_on_front(dataRead.dades)) {
        moving_state = INTERIOR_TURN; //ara mateix tota la casualistica que pot haver-hi porta aquí
        //en un futur podriem afegir un estat que sigui "busca paret" llavors necessiteriem un if
    } else if (wall_on_side_TOO_FAR(dataRead.dades)) {
        moving_state = TOO_CLOSE; //si ens allunyem del mur definit per la variable global, apropat
    }  /*else if (wall_on_side_perfect(dataRead.dades)) {
        moving_state = GO_STRAIGHT; //si tenim una bona distancia amb la paret segueix recta
    }*/ else if (no_wall(dataRead.dades)) {
        //if(prev_moving_state == GO_STRAIGHT || prev_moving_state == TOO_CLOSE || prev_moving_state == TOO_FAR){
            moving_state = EXTERIOR_TURN;
            //si estavem seguint una paret i de cop i volta no hi ha res, vol dir que estavem
            //reseguint una cantonada per l'exterior
        //}
        //TODO: pdriem afegir un altre estat que sigui "buscar paret" des del mig de lhabitacio
    } else if (wall_on_side_too_far(dataRead.dades)) {
        moving_state = TOO_FAR; //si ens apropem del mur definit per la variable global, allunyat
    } else{
        moving_state = GO_STRAIGHT;
    }

    return dataRead;
}


/*
*Aquesta funcio la utilitzem per que el robot giri cap al canto contrari de la paret
*que estava seguint.
*No fa res més que invertir la direcció que porta el robot i crida a una altra funcio de girar
*/
void get_away() {
    if (sideWall == ESQUERRA) { //Venim de paret a l'esquerra
        hard_turn_robot(DRETA, velocitat_lenta);
    } else { //Venim de paret a la dreta
        hard_turn_robot(ESQUERRA, velocitat_lenta);
    }

}

/*
*Aquesta funcio la utilitzem per que el robot giri cap al canto de la paret
*que estava seguint.
*No fa res més que amagar el fet de saber la direccio que seguiem i crida a una altra funcio de girar
*/
void get_closer() {
    if (sideWall == ESQUERRA) { //Venim de paret a l'esquerra
        turn_robot(ESQUERRA, velocitat_lenta);
    } else { //Venim de paret a la dreta
        turn_robot(DRETA, velocitat_lenta);
    }

}

/*
*Aquesta funcio la utilitzem per que el robot giri cap al canto contrari de la paret
*que estava seguint.
*No fa res més que invertir la direcció que porta el robot i crida a una altra funcio de girar
*/
void interior_turn() {

    if (sideWall == ESQUERRA) { //Venim de paret a l'esquerra
        hard_turn_robot(DRETA, velocitat_lenta);
    } else { //Venim de paret a la dreta
        hard_turn_robot(ESQUERRA, velocitat_lenta);
    }

    reset_delay_timer();
    while (delay_timer(500)); //Petit delay_timer per agafar millor la curva
}

/* Metode que utilitzem per guiar al robot fora de obstacles en forma de C o passos estrets.
* Aquest metode pot ser molt util per quan no hi hagi suficient espai com per girar sobre si mateix.
*/
void go_back_find_way(struct Data dataRead) {
    uint8_t sensor;
    uint8_t direction;


    if (sideWall == ESQUERRA) { //volem el sensor contrari a la paret que estem seguint
        sensor = 2;
        direction = DRETA;
    } else if (sideWall == DRETA) {
        sensor = 0;
        direction = ESQUERRA;
    }

    while (dataRead.dades[sensor] > max_threshold_lateral) { //mentres estiguem encara engarjolats
        move_robot(ENDARRERE, velocitat_lenta);
        dataRead = distance_sensor();
    }

    turn_robot_90_degrees(direction, velocitat_lenta); //girarem 90 graus en direccio del sensor
    reset_delay_timer();
    while (delay_timer(1000)); //Temps de gir 90 graus a velocitat_lenta

    move_robot(ENDAVANT, velocitat_lenta); //ens apropem a lobstacle
    reset_delay_timer();
    while (delay_timer(1000)); //Temps de gir abans de trobar la paret de nou
}

/*Aquest metode el fem per fer girs exteriors
* El delay_timer es per deixar prou marge amb la cantonada
*/
void open_turn() {
    if (prev_moving_state != EXTERIOR_TURN) {
        reset_delay_timer();
        while (delay_timer(1000)); //Petit delay_timer per agafar millor la curva
    }

    if (sideWall == ESQUERRA) { //Venim de paret a l'esquerra
        hard_turn_robot(ESQUERRA, velocitat_lenta);
    } else { //Venim de paret a la dreta
        hard_turn_robot(DRETA, velocitat_lenta);
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
    struct Data dataRead;

    //Bucle principal (infinito):
    do
    {
        if (estado_anterior != estado)// Dependiendo del valor del estado se encender� un LED u otro.
        {
            sprintf(cadena, "estado %d", estado); // Guardamos en cadena la siguiente frase: estado "valor del estado"
            escribir(cadena, linea);          // Escribimos la cadena al LCD
            estado_anterior = estado; // Actualizamos el valor de estado_anterior, para que no est� siempre escribiendo.

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
                    //Si polsem S1, encenem els 3 LEDs RGB segons la dist�ncia a la que es detecta un objecte amb el sensor
                    read_distance_wall(DRETA); //setejem els tresholds laterals
                    moving_state = GO_STRAIGHT; //arranquem el robot
                    activa_timerA0();
                    break;

                case 2:
                    //Si polsem S2, el robot fa stop
                    stop_robot();
                    read_distance_wall(CENTRE); //llegim el threshold del centre
                    moving_state = STOP;
                    prev_moving_state = STOP;
                    break;

                case 3:
                    //Si polsem el joystick  a l'esquerra, el robot gira sobre si mateix cap a l'esquerra
                    //turn_robot(ESQUERRA, velocitat_lenta);
                    //turn_robot_90_degrees(ESQUERRA, velocitat_lenta);
                    activa_timerA0();
                    interior_turn();
                    stop_robot();
                    break;

                case 4:
                    //Si polsem el joystick  a la dreta, el robot gira sobre si mateix cap a la dreta
                    hard_turn_robot(DRETA, velocitat_lenta);
                    //turn_robot_90_degrees(DRETA, velocitat_lenta);
                    break;

                case 5:
                    //Si polsem el joystick  amunt, el robot avan�a
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
                    //read_distance_wall(CENTRE); //llegim el treshold del centre
                    break;
            }
        }

        if(moving_state != STOP){
          dataRead = moving_state_selector();
          process_distance(dataRead);
        }

        if (moving_state != prev_moving_state){
            prev_moving_state = moving_state;
            switch (moving_state) {
                case JAILED: //Paret als tres cantons o hem arribat a un estret
                    go_back_find_way(dataRead);
                    break;
                case INTERIOR_TURN: //paret al davant mentres seguiem una paret
                    interior_turn();
                    break;
                case EXTERIOR_TURN: //estavem seguint una paret i ara no hi ha res
                    open_turn();
                    break;
                case TOO_CLOSE: //ens apropem massa a la paret que seguiem
                    get_away();
                    break;
                case TOO_FAR: //ens allunyem massa de la paret que seguiem
                    get_closer();
                    break;
                case GO_STRAIGHT: //rang correcte amb la paret que seguiem
                    move_robot(ENDAVANT, velocitat_lenta);
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
void PORT4_IRQHandler(void) { //interrupci�n de los botones. Actualiza el valor de la variable global estado.
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
void PORT5_IRQHandler(void) { //interrupci�n de los botones. Actualiza el valor de la variable global estado.
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
    TA0CCTL0 &= ~CCIE;//Conv� inhabilitar la interrupci� al comen�ament
    comptador++;    //Incrementem la variable global comptador
    TA0CCTL0 &= ~CCIFG;//Hem de netejar el flagde la interrupci�
    TA0CCTL0 |= CCIE;//S�ha d�habilitar la interrupci� abans de sortir
}

//Interrupci� timer TimeOut
void TA1_0_IRQHandler(void)
{
    TA1CCTL0 &= ~CCIE; //Conv� inhabilitar la interrupci� al comen�ament
    comptTimeOut++;    //Incrementem variable global seg
    TA1CCTL0 &= ~CCIFG; //Hem de netejar el flag de la interrupci�
    TA1CCTL0 |= CCIE; //S�ha d�habilitar la interrupci� abans de sortir
}

//Interrupci� dada llegida UART
void EUSCIA2_IRQHandler(void){
    UCA2IE &= ~UCRXIE; //Conv� inhabilitar la interrupci� al comen�ament
    DadaLlegida_UART = UCA2RXBUF; //Guardem la dada llegida
    Byte_Rebut = 1; //Posem la variable global de control a 1
    UCA2IE |= UCRXIE; //S�ha d�habilitar la interrupci� abans de sortir
}
