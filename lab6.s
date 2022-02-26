
;Archivo: Prelab6.s
;Dispositivo: PIC16F887
;Autor: Jimena de la Rosa
;Compilador: pic-as (v2.30). MPLABX v5.40
;Programa: laboratorio 5
;Hardware: LEDs en el puerto A
;Creado: 21 FEB, 2022
;Ultima modificacion: 26 FEB, 2022
    
PROCESSOR 16F887

; PIC16F887 Configuration Bit Settings

; Assembly source line config statements

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

// config statements should precede project file includes.
#include <xc.inc>

;-------- Declracion de variables------
 UP EQU 6; NOMBRAR EL BIT 6 COMO UP PARA INCREMENTAR
 DOWN EQU 7; NOMBRAR EL BIT 7 COMO DOWN PARA DECREMENTAR
PSECT UDATA_BANK0,global,class=RAM,space=1,delta=1,noexec
  
  GLOBAL  CONT, BANDERAS, NIBBLES, DISPLAY, CONTC, NUMERO, CONTD, CONTU, BANDERA1, DECIMAL
    CONT: DS 2 ;SE NOMBRA UNA VARIABLE DE CONTADOR DE 4 BITS
    BANDERAS: DS 1	; Indica que display hay que encender
    NIBBLES:  DS 2	; Contiene los nibbles alto y bajo de valor
    DISPLAY:  DS 2	;contien las dos variables de los nibbles
    NUMERO:    DS 2	;variable que almacena el valor de PORTA
    CONTC:    DS 1	; contador de centenas
    CONTD:    DS 1	; contador de decenas
    CONTU:    DS 1	; contador de decenas
    BANDERA1:  DS 1	;bandera para el numero decimal 
    DECIMAL:  DS 3	; contine los valores decimales para los displays

; ------- VARIABLES EN MEMORIA --------
PSECT udata_shr		    ; Memoria compartida
    W_TEMP:		DS 1
    STATUS_TEMP:	DS 1

PSECT resVect, class=CODE, abs, delta=2
ORG 00h			    ; posición 0000h para el reset
;------------ VECTOR RESET --------------
resetVec:
    PAGESEL MAIN	    ; Cambio de pagina
    GOTO    MAIN
    
PSECT intVect, class=CODE, abs, delta=2
ORG 04h			    ; posición 0004h para interrupciones
;------- VECTOR INTERRUPCIONES ----------
PUSH:
    MOVWF   W_TEMP	    ; Guardamos W
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP	    ; Guardamos STATUS
    
ISR:
    BTFSC   T0IF; SE REVISA SI ESTA ENCENDIDO
    CALL    INT_TMR0	    ; SE EJECUTA LA INSTRUCCI[ON DE LA INTERRUPCION DEL TMR0
    BTFSC   RBIF	    ; Fue interrupción del PORTB? No=0 Si=1
    CALL    INT_IOCB	    ; Si -> Subrutina o macro con codigo a ejecutar
    
    
POP:
    SWAPF   STATUS_TEMP, W  
    MOVWF   STATUS	    ; Recuperamos el valor de reg STATUS
    SWAPF   W_TEMP, F	    
    SWAPF   W_TEMP, W	    ; Recuperamos valor de W
    RETFIE		    ; Regresamos a ciclo principal
    
    
PSECT code, delta=2, abs
ORG 100h		    ; posición 100h para el codigo
;------------- CONFIGURACION ------------
MAIN:
    CALL    CONFIG_IO	    ; Configuración de I/O
    CALL    CONFIG_RELOJ    ; Configuración de Oscilador
    CALL    CONFIG_TMR0	    ; Configuración de TMR0
    CALL    CONFIG_INT	    ; Configuración de interrupciones
    CALL    CONFIG_IOCRB
    BANKSEL PORTB	    ; Cambio a banco 00
 
LOOP:
    MOVF    PORTA, W		; Valor del PORTA a W
    MOVWF   CONT		; Movemos W a variable valor	
    CALL    OBTENER_NIBBLE	; Guardamos nibble alto y bajo de valor
    CALL    SET_DISPLAY		; Guardamos los valores a enviar en PORTC para mostrar valor en hex
    CALL    SET_DECIMAL		; se guardan los valores a enviar al PORTD
    GOTO    LOOP

;--------------------- SUBRUTINAS-----------------------
INT_IOCB:
    BANKSEL PORTB ; SE SELECCIONA EL BANCO 0
    BTFSS PORTB, UP ; SE REVISA SI EL BIT ESTA ENCENDIDO
    CALL INCREMENTAR    ;SI NO ESTA ENCENDIDO, SE INCREMNATA EL PORTB
    BTFSS PORTB, DOWN; SE REVISA SI EL BIT ESTA ENCENDIDO
    CALL DECREMENTAR ; SI NO ESTA ENCENDIDO SE DECREMENTA
    BCF RBIF	    ; se limpia la bandera del cambio en PORTB
    
    RETURN
INCREMENTAR:
    INCF PORTA			; se incrementa el valor de PORTA
    MOVF    PORTA, W		; Valor del PORTA a W
    MOVWF   NUMERO		; se guarda del valor de PORTA en la variable
    CALL    OBTENER_DECIMAL	;se guardan los valores de centenas, decenas y unidades
    RETURN
    
DECREMENTAR:
    DECF PORTA			; se decrementa el valor de PORTA
    MOVF    PORTA, W		; Valor del PORTA a W
    MOVWF   NUMERO		; se guarda del valor de PORTA en la variable
    CALL    OBTENER_DECIMAL	;se guardan los valores de centenas, decenas y unidades
    RETURN
    
CONFIG_IOCRB:
    BANKSEL TRISB
    BSF IOCB, UP    ; SE CONFIGURAN LOS PULL UPS DE LAS ENTRADAS
    BSF IOCB, DOWN
    
    BANKSEL PORTB
    MOVF    PORTB, W 
    BCF	    RBIF    ; SE LIMPIA LA BANDERA DEL CAMBIO EN EL PORT B
    RETURN
 
 CONFIG_IO:
    BANKSEL ANSEL
    CLRF    ANSEL
    CLRF    ANSELH	    ; I/O digitales
    BANKSEL TRISB
    CLRF   TRISA ;DEJAR PORTA COMO SALIDA
    CLRF   TRISC
    CLRF   TRISD
    BCF	    TRISB, 0		; RB0 como salida / display nibble alto
    BCF	    TRISB, 1		; RB1 como salida / display nibble bajo
    BCF	    TRISB, 2		; RB2 como salida / CENTENA
    BCF	    TRISB, 3		; RB3 como salida / DECENA
    BCF	    TRISB, 4		; RB4 como salida / UNIDAD
    BSF	    TRISB, UP ; UP Y DOWN COMO ENTRADAS
    BSF	    TRISB, DOWN 
    BCF	    OPTION_REG, 7; SE HABILITAN LOS PULL_UPS
    BSF	    WPUB, UP	
    BSF	    WPUB, DOWN
    BANKSEL PORTC
    CLRF    PORTA	    ; Apagamos PORTA, PORTB, PORTC, PORTD
    CLRF    PORTC
    CLRF    PORTD
    CLRF    PORTB
    CLRF  CONT     ;dejar los contadires y banderas en cero
    CLRF  BANDERAS
    CLRF  NIBBLES
    MOVLW   10111111B ; dejar los valores de los display decimales en cero del 7seg
    MOVWF  DISPLAY
    MOVWF  DISPLAY+1
    MOVWF  DISPLAY+2
    CLRF  CONTC
    CLRF  CONTD
    CLRF  CONTU
    CLRF  NUMERO
    RETURN
    
CONFIG_INT:
    BANKSEL INTCON
    BSF	    GIE		    ; Habilitamos interrupciones
    BSF	    T0IE	    ; Habilitamos interrupcion TMR0
    BCF	    T0IF	    ; Limpiamos bandera de TMR0
    bsf	    RBIE	    ; Habilitamos interrupciones de PORTB
    bcf	    RBIF	    ; Limpiamos bandera de PORTB
    movf    PORTB, F	    ; se hace lectura del PORTB
    RETURN

CONFIG_RELOJ:
    BANKSEL OSCCON
    BSF OSCCON, 0; RELOJ INTERNO
    BCF OSCCON, 4; OSCILADOR DE 4MH
    BSF OSCCON, 5
    BSF OSCCON, 6
    RETURN
    
CONFIG_TMR0:
   BANKSEL OPTION_REG
    BCF PSA
    BCF PS0; PRESCALER DE 1:128
    BSF PS1
    BSF PS2 
    BCF T0CS ; RELOJ INTERNO
    MOVLW 178 ; 20MS
    
    BANKSEL TMR0
    MOVWF TMR0 ; CARGAMOS EL VALOR INICIAL
    BCF   T0IF; LIMPIAMOS LA BANDERA
    RETURN
    
    
REINICIO_TMR0:
    BANKSEL TMR0
    MOVLW   178		; 20 ms 
    MOVWF   TMR0	; Cargamos valor inicial
    BCF	    T0IF	; Limpiamos bandera
    RETURN

ORG 200H
TABLA:
    CLRF PCLATH
    BSF  PCLATH, 1
    ANDLW 0X0F; SE ASEGURA QUE SOLO EXISTAN 4 BITS
    ADDWF PCL
    RETLW 10111111B; 01000000B 0
    RETLW 10000110B;01111001B 1
    RETLW 11011011B; 00100100B;2
    RETLW 11001111B ;00110000B;3
    RETLW 11100110B ;00011001B;4
    RETLW 11101101B ;00010010B;5
    RETLW 11111101B ;00000010B;6
    RETLW 10000111B ;01111000B;7
    RETLW 11111111B ;00000000B;8
    RETLW 11101111B ;00010000B;9
    RETLW 11110111B ;00001000B;A
    RETLW 11111100B ;00000011B;B
    RETLW 10111001B ;01000110B;C
    RETLW 11011110B ;00100001B;D
    RETLW 11111001B ;00000110B;E
    RETLW 11110001B ;00001110B;F

OBTENER_NIBBLE:			; 
    MOVLW   0x0F		; se asegura de que existan solamente 4 bits    
    ANDWF   CONT, W		;	
    MOVWF   NIBBLES		; se guradan los nibbles de menor significancia en la variable    
			
    MOVLW   0xF0		;  se asegura de que existan solamente 4 bits   
    ANDWF   CONT, W		;	
    MOVWF   NIBBLES+1		;se guradan los nibbles de mayor significancia en la variable	    
    SWAPF   NIBBLES+1, F	; se cambia de posicion los nibbles
    RETURN
    
OBTENER_DECIMAL:
    CLRF CONTC; se limpian los cotadores de centenas, decenas y unidades
    CLRF CONTD
    CLRF CONTU
    MOVLW   100		    ;se gurada en W el literal 100
    SUBWF   NUMERO, F	    ; se le resta el literal al numero y se guarda en si mismo
    INCF    CONTC	    ; se incrementa el contador de centenas
    BTFSC   STATUS, 0	    ; se revisa si hubo un bit de BORROW 
    GOTO    $-4		    ; si no hubo, regresa a restar
    DECF    CONTC	    ; si hubo, se decrementa el conatdor de centenas
    MOVLW   100		    ;se gurada en W el literal 100
    ADDWF   NUMERO, F	    ; se le suma el literal al numero y se guarda en si mismo
    MOVLW   10		    ;se gurada en W el literal 10
    SUBWF   NUMERO, F	    ; se le resta el literal al numero y se guarda en si mismo
    INCF    CONTD	    ; se incrementa el contador de decenas
    BTFSC   STATUS, 0	    ; se revisa si hubo un bit de BORROW
    GOTO    $-4		    ; si no hubo, regresa a restar
    DECF    CONTD	    ; si hubo, se decrementa el conatdor de decenas
    MOVLW   10		    ;se gurada en W el literal 10
    ADDWF   NUMERO, F	    ; se le suma el literal al numero y se guarda en si mismo
    MOVLW   1		    ;se gurada en W el literal 1
    SUBWF   NUMERO, F	    ; se le resta el literal al numero y se guarda en si mismo
    INCF    CONTU	    ; se incrementa el contador de unidades
    BTFSC   STATUS, 0	    ; se revisa si hubo un bit de BORROW
    GOTO    $-4		    ; si no hubo, regresa a restar
    DECF    CONTU	    ; si hubo, se decrementa el conatdor de unidades
   RETURN
    
    
SET_DISPLAY:
    MOVF    NIBBLES, W		; Movemos nibble bajo a W
    CALL    TABLA		; Buscamos valor a cargar en PORTC
    MOVWF   DISPLAY		; Guardamos en display
    
    MOVF    NIBBLES+1, W	; Movemos nibble alto a W
    CALL    TABLA		; Buscamos valor a cargar en PORTC
    MOVWF   DISPLAY+1		; Guardamos en display+1
    RETURN
    
SET_DECIMAL:
    MOVF    CONTC, W ;se llama el valor de la centana en la tabla
    CALL    TABLA
    MOVWF   DECIMAL; se asigna el valor de la tabla a la variable
    
    MOVF    CONTD, W;se llama el valor de la decena en la tabla
    CALL    TABLA
    MOVWF   DECIMAL+1; se asigna el valor de la tabla a la variable
    
    MOVF    CONTU, W;se llama el valor de la unidad en la tabla
    CALL    TABLA
    MOVWF   DECIMAL+2; se asigna el valor de la tabla a la variable
    RETURN
    
    
MOSTRAR_VALOR:
    BCF	    PORTB, 0		; Apagamos display de nibble alto
    BCF	    PORTB, 1		; Apagamos display de nibble bajo
    BTFSC   BANDERAS, 0		; Verificamos bandera
    GOTO    DISPLAY_1		;  

    DISPLAY_0:			
	MOVF    DISPLAY, W	; Movemos display a W
	MOVWF   PORTC		; Movemos Valor de tabla a PORTC
	BSF	PORTB, 1	; Encendemos display de nibble bajo
	BSF	BANDERAS, 0	; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
    RETURN
    DISPLAY_1:
	MOVF    DISPLAY+1, W	; Movemos display+1 a W
	MOVWF   PORTC		; Movemos Valor de tabla a PORTC
	BSF	PORTB, 0	; Encendemos display de nibble bajo
	BCF	BANDERAS, 0	; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
    RETURN
    
MOSTRAR_DECIMAL:
    BCF	    PORTB, 2		; Apagamos display de nibble alto
    BCF	    PORTB, 3		; Apagamos display de nibble bajo
    BCF	    PORTB, 4
    BTFSC   BANDERA1, 0		; Verificamos bandera
    GOTO    DECENA		;  
    BTFSC   BANDERA1, 1
    GOTO    UNIDAD
    
    CENTENA:			
	MOVF    DECIMAL, W
	MOVWF   PORTD		; Movemos Valor de tabla a PORTd
	BSF	PORTB, 2	; Encendemos display de centenas
	BSF	BANDERA1, 0	; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
	BCF	BANDERA1, 1
    RETURN
    
    DECENA:
	MOVF    DECIMAL+1, W	; Movemos decimal+1 a W
	MOVWF   PORTD		; Movemos Valor de tabla a PORTd
	BSF	PORTB, 3	; Encendemos display de decenas
	BCF	BANDERA1, 0	; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
	BSF	BANDERA1, 1
    RETURN
    
    UNIDAD:
	MOVF    DECIMAL+2, W	; Movemos decimal+2 a W
	MOVWF   PORTD		; Movemos Valor de tabla a PORTd
	BSF	PORTB, 4	; Encendemos display de unidades
	BCF	BANDERA1, 0	; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
	BCF	BANDERA1, 1
    RETURN
        
INT_TMR0:
    CALL REINICIO_TMR0	; Reiniciamos TMR0 para 50ms
    CALL    MOSTRAR_VALOR	; Mostramos valor en hexadecimal en los displays
    CALL    MOSTRAR_DECIMAL	; se muestra el valor decimal en los displays
    RETURN
END
