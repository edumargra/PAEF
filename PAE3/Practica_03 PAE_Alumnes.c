/******************************
 *
 * Practica_03_PAE Timers i interrupcions
 * UB, 02/2017.
 *****************************/

#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include "lib_PAE2.h" //Libreria grafica + configuracion reloj MSP432

char saludo[16] = " PRACTICA 3 PAE"; //max 15 caracteres visibles
char cadena[16]; //Una linea entera con 15 caracteres visibles + uno oculto de terminacion de cadena (codigo ASCII 0)
char borrado[] = "               "; //una linea entera de 15 espacios en blanco
uint8_t linea = 1;
uint8_t estado = 0;
uint8_t estado_anterior = 8;
uint16_t retraso = 2000;
uint16_t comptador = 0;
uint8_t segons_anterior = 1;
uint8_t seg = 0;
uint8_t min = 0;
uint8_t hora = 0;
uint8_t min_a = 1;
uint8_t hora_a = 0;
uint8_t modificar_hora = 0;
uint8_t modificar_alarma = 0;

/**************************************************************************
 * INICIALIZACIï¿½N DEL CONTROLADOR DE INTERRUPCIONES (NVIC).
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 **************************************************************************/
void init_timer()
{
    //Configurem el timer
    //***************************
    TA0CTL |= TASSEL__ACLK; //Seleccionem ACLK
    TA0CCR0 |= 32; //setejem el valor del registre TA0CCR0 (el maxim del timer) per tal que el temps sigui 1 milisegon
    TA0CCTL0 |= CCIE; //Habilitem les interrupcions del timer
    TA0CCTL0 &= ~CCIFG; //Netegem el vector d'interrupcions
}

void init_timer_clock()
{
    //Configurem el timer
    //***************************
    TA1CTL |= TASSEL__ACLK; //Seleccionem ACLK
    TA1CTL |= MC__UP; //seleccionem el mode up i iniciem el timer
    TA1CCR0 |= 32000; //setejem el valor de CCR0 (el maxim del timer) per tal que el temps sigui 1 segon
    TA1CCTL0 |= CCIE; //Habilitem les interrupcions del timer
    TA1CCTL0 &= ~CCIFG; //Netegem el vector d'interrupcions

}

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
void init_LCD(void)
{
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
void borrar(uint8_t Linea)
{
    halLcdPrintLine(borrado, Linea, NORMAL_TEXT); //escribimos una linea en blanco
}

/**************************************************************************
 * ESCRIBIR LINEA
 *
 * Datos de entrada: Linea, indica la linea del LCD donde escribir
 * 					 String, la cadena de caracteres que vamos a escribir
 *
 * Sin datos de salida
 *
 **************************************************************************/
void escribir(char String[], uint8_t Linea)

{
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
void init_botons(void)
{
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

/**************************************************************************
 * DELAY - A CONFIGURAR POR EL ALUMNO - con bucle while
 *
 * Datos de entrada: Tiempo de retraso. 1 segundo equivale a un retraso de 1000000 (aprox)
 *
 * Sin datos de salida
 *
 **************************************************************************/
void delay_t(uint32_t limit)
{
    TA0CTL |= MC__UP; //seleccionem el mode UP i iniciem el timer
    comptador = 0;    //reiniciem el comptador a 0
    do{
    }while (comptador < limit);
    TA0CTL |= MC__STOP; //seleccionem el mode STOP i parem el timer
}

/*****************************************************************************
 * CONFIGURACIï¿½N DEL PUERTO 7. A REALIZAR POR EL ALUMNO
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 ****************************************************************************/

void config_P7_LEDS(void)
{   
    P7SEL0 = 0x00; //Configurem els pins del port 7 com a GPIOs
    P7SEL1 = 0x00;
    P7DIR |= 0xFF;  //Tots els pins del port 7 configurats com a sortides
    P7OUT = 0x00;  //Inicialitzem els leds del port 7 a 0 (apagats)
}

void ledsprogressiusL2R(void)
{
    uint8_t i;
    for (i = 0x01; i <= 0x80; i = i << 1)
    {	//encenem un led, l'apaguem i passem al seguent d'esquerra a dreta
        P7OUT |= i;
        delay_t(retraso);
        P7OUT &= ~i;
    }
}

void ledsprogressiusR2L(void)
{
    uint8_t i;
    for (i = 128; i >= 1; i = i >> 1)
    {	//encenem un led, l'apaguem i passem al seguent de dreta a esquerra
        P7OUT |= i;
        delay_t(retraso);
        P7OUT &= ~i;
    }
}

void ledsprogressius2F(void)
{
    if (retraso < 5000)
        {	//si el retard no supera 5000, augmentem per 500
            retraso += 500;
        }
}

void ledsprogressius2S(void)
{
    if (retraso > 0)
        {	//si el retard es mes gran que 0, reduim per 500
            retraso -= 500;
        }
}
void print_time(void){
    if (seg == 60) {    //Si arribem a 60 segons cal posar seg a 0 i incrementar els minuts
        seg = 0;
        min++;
    }
    if (min == 60) {    //Si arribem a 60 minuts cal posar min a 0 i incrementar les hores
        min = 0;
        hora++;
    }
    if (hora == 24) {   //Si arribem a 24 hores cal posar hora a 0
        hora = 0;
    }

    sprintf(cadena, "tiempo %02d:%02d:%02d", hora, min, seg); // Guardamos en cadena la siguiente frase: tiempo hora:min:segundos
    escribir(cadena, linea+2);          // Escribimos la cadena al LCD
}

void apujar_hora() {
    // Incrementem en un els minuts del rellotge
    seg = 0;
    min++;
    segons_anterior = 60; //Posem a segons_anterior un valor diferent a segons (seg no arribarà mai a 60)
}

void abaixar_hora() {
    // Decrementem en un els minuts del rellotge (cal comprovar que els minuts no siguin 0)
    seg = 0;
    if (min > 0) {
       min--;
    }
    segons_anterior = 60; //Posem a segons_anterior un valor diferent a segons (seg no arribarà mai a 60)
}

void apujar_alarma() {
    // Incrementem en un els minuts de l'alarma
    if (min_a < 60) { 
       min_a++;
    } else if (min_a == 60) { //Si els minuts de l'alarma són iguals a 60 cal posar-los a 0 i incrementar l'hora de l'alarma
        min_a = 0;
        hora_a++;
    }
    if (hora_a == 24) { //Si l'hora de l'alarma és igual a 24 cal posar-la a 0
        hora_a = 0;
    }
}

void abaixar_alarma() {
    // Decrementem en un els minuts de l'alarma (cal comprovar que els minuts no siguin 0)
    if (min_a > 0) {
       min_a--;
    }
}

void comprova_alarma(){
    // Comprovem si cal fer sonar l'alarma
    if (hora == hora_a && min == min_a && seg == 0) { //Si la hora i els minuts del rellotge coincideixen amb l'alarma, cal fer-la sonar
        sprintf(cadena, "   ALARMA!!!!"); // Guardamos en cadena la siguiente frase: ALARMA!!!
        escribir(cadena, linea+5);          // Escribimos la cadena al LCD
    }
}

void main(void)
{

    WDTCTL = WDTPW + WDTHOLD;       	// Paramos el watchdog timer

    //Inicializaciones:
    init_ucs_24MHz();       //Ajustes del clock (Unified Clock System)
    init_botons();         //Configuramos botones y leds
    init_LCD();			    // Inicializamos la pantalla
    config_P7_LEDS(); //Inicialitzem els leds del port 7 i els apaguem
    init_timer();   //inicialitzem el timer TA0
    init_timer_clock();   //inicialitzem el timer TA1
    init_interrupciones();  //Configurar y activar las interrupciones de los botones
    halLcdPrintLine(saludo, linea, INVERT_TEXT); //escribimos saludo en la primera linea
    linea++; //Aumentamos el valor de linea y con ello pasamos a la linea siguiente

    //Bucle principal (infinito):
    do
    {   
        if (segons_anterior != seg){
            print_time();
            comprova_alarma();
            segons_anterior = seg;
        }
        if (estado_anterior != estado)// Dependiendo del valor del estado se encenderï¿½ un LED u otro.
        {
            sprintf(cadena, "estado %d", estado); // Guardamos en cadena la siguiente frase: estado "valor del estado"
            escribir(cadena, linea);          // Escribimos la cadena al LCD
            sprintf(cadena, "alarma %02d:%02d", hora_a, min_a); // Guardamos en cadena la siguiente frase: alarma hora:min
            escribir(cadena, linea+3);          // Escribimos la cadena al LCD
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
            switch (estado)
            {
            case 1:
                //Si polsem S1, encenem els 3 LEDs RGB
                P2OUT |= 0x50;  //Leds P2.4 (G), 2.6 (R) a 1 (encesos)
                P5OUT |= 0x40; //Led P5.6(B) a 1 (ences)
                modificar_hora = !modificar_hora;
                break;

            case 2:
                //Si polsem S2, apaguem els 3 LEDs RGB
                P2OUT &= ~0x50;  //Leds P2.4 (G), 2.6 (R) a 0 (apagats)
                P5OUT &= ~0x40; //Led P5.6(B) a 1 (apagat)
                modificar_alarma = !modificar_alarma;
                sprintf(cadena, "             "); // Guardamos en cadena una frase vacia
                escribir(cadena, linea+5);          // Escribimos la cadena al LCD
                break;

            case 3:
                //Si polsem el joystick  a l'esquerra, encenem els 3 LEDs RGB
                P2OUT |= 0x50;  //Leds P2.4 (G), 2.6 (R) a 1 (encesos)
                P5OUT |= 0x40; //Led P5.6(B) a 1 (ences)
                ledsprogressiusR2L();
                break;

            case 4:
                //Si polsem el joystick  a la dreta, LEDs vermell (R) i verd (G)  encesos,  blau (B) apagat
                P2OUT |= 0x50;  //Leds P2.4 (G), 2.6 (R) a 1 (encesos)
                P5OUT &= ~0x40; //Led P5.6(B) a 0 (apagat)
                ledsprogressiusL2R();
                break;

            case 5:
                //Si polsem el joystick  amunt, LEDs vermell (R) i  blau (B) encesos,  verd (G) apagat
                P2OUT |= 0x40;  //Led P2.6 (R) a 1 (ences)
                P2OUT &= ~0x10;  //Led P2.4 (G) a 0 (apagat)
                P5OUT |= 0x40; //Led P5.6(B) a 1 (ences)
                ledsprogressius2S();
                if (modificar_hora) {
                    apujar_hora();
                }
                if (modificar_alarma) {
                    apujar_alarma();
                }
                break;

            case 6:
                //Si polsem el joystick  avall, LEDs   verd (G)  i blau (B) encesos, vermell (R) apagat
                P2OUT &= ~0x40;  //Led P2.6 (R) a 0 (apagat)
                P2OUT |= 0x10;  //Led P2.4 (G) a 1 (ences)
                P5OUT |= 0x40; //Led P5.6(B) a 1 (ences)
                ledsprogressius2F();
                if (modificar_hora) {
                    abaixar_hora();
                }
                if (modificar_alarma) {
                    abaixar_alarma();
                }
                break;

            case 7:
                //Si polsem el joystick al centre, s'ha d'invertir l'estat dels 3 LEDs RGB
                P2OUT ^= 0x50;  //Invertim leds P2.4 (G), 2.6 (R)
                P5OUT ^= 0x40; //Invertim led P5.6(B)
                break;
            }

            sprintf(cadena, "retraso %04d", retraso); // Guardamos en cadena la siguiente frase: retraso "valor de retraso actual"
            escribir(cadena, linea+1);          // Escribimos la cadena al LCD
        }


    }
    while (1); //Condicion para que el bucle sea infinito
}

/**************************************************************************
 * RUTINAS DE GESTION DE LOS BOTONES:
 * Mediante estas rutinas, se detectarï¿½ quï¿½ botï¿½n se ha pulsado
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
    P4IE &= 0x5D; 	//interrupciones Joystick en port 4 desactivadas
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

    P4IE |= 0xA2; 	//interrupciones Joystick en port 4 reactivadas
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

void TA1_0_IRQHandler(void)
{
    TA1CCTL0 &= ~CCIE;//Convé inhabilitar la interrupció al començament
    seg++;    //Incrementem variable global seg
    TA1CCTL0 &= ~CCIFG;//Hem de netejar el flagde la interrupció
    TA1CCTL0 |= CCIE;//S’ha d’habilitar la interrupció abans de sortir
}
