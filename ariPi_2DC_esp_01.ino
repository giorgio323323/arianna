/*
 * arianna
 * giorgio323@gmail.com
 * infostuffcube@gmail.com
 * 
 * 16nov16	inizio
 
	test su sensore ir
	l'idea è trovare il tempo a 1 rispetto al tempo totale
	questo è un indicatore della distanza
	
	10dic16	sensore rotazione motore (odometro)	
			spostato su Raspberry la compilazione
			ID_000  digitalPinToInterrupt(R_SIDE_IR) non dichiarato

	27dic16	comandi da seriale
			gestione arrivo in posizione 
			aggiunto sonar montato su servo
			
			per trovare la direzione da puntare cerco il punto a distanza minima
			con un algoritmo di bisezione
			faccio con uno script esterno
 
	02gen17	penso a gestire sensore dx e sx
	numbers 0 (on digital pin 2) and 1 (on digital pin 3)
	
	19gen17	differenziale elettronico
			tolgo ovfTimer perchè usato dai PWM
			gestire misura sensore senza interrupt timer

	28gen17	add blueTooth on serial port1
			vanno ignorati i vari messaggi da BT
			quando non è connesso
			code 0000
			http://wiki.seeedstudio.com/wiki/Bluetooth_Bee
			
	23mar17 ID_001 rimosso limite angolo teta		

	14mag17 si aggiunge comunicazione con esp ID_002
			si usa la serial2 verso esp
			modifica protocollo Alessandro Airaghi
			
	21mag17 ripristinato BT in parallelo ID_003
			Ok

	07giu17 cambiati encoder




10	verde con punto nero
11  blu e bianco
12	blu

 * Simple test for the DRV8833 library.
 * The DRV8833 is a dual motor driver carrier made by Pololu.
 * You can find it here: https://www.pololu.com/product/2130

 */

#include <SPI.h>
#include <SD.h>
#include <Servo.h> 
#include <TimerOne.h>
// #include <NewPing.h> usa interrupt timer2
#define	DUE_PI	6.28318

//Servo sterzo;          	// create servo object to control a servo 
Servo servoPan;      	// create servo object to control a servo 
Servo servoTilt;      	// create servo object to control a servo 

#define TRIGGER_PIN	12
#define ECHO_PIN	11		// FILO BIANCO BLU



//********** ID_002
//seriale di comunicazione con esp uso pin 7 e 9 
//SoftwareSerial ESPserial(7, 9); // RX | TX
#define ESPserial Serial2

String inputString 		= "";
String inputStringTmp 	= "";
int okcomm				= 0;
static String risposta;
char port	= 0;

//**********fine ID_002

  
//NewPing sonar(TRIGGER_PIN, ECHO_PIN , 200); // usa intTimer2 conflitto

// dc motor
#include <DRV8833.h>

// Create an instance of the DRV8833:
DRV8833 driver = DRV8833();

// Pin numbers. Replace with your own!
// Attach the Arduino's pin numbers below to the
// Ain1 and Ain2 DRV8833 pins.
// N.B. la libreria sembra non gestire i pin 44-46 (timer diverso) 
const int MTR_A_P1 	= 5;
const int MTR_A_P2 	= 6;
const int MTR_B_P1 	= 9;
const int MTR_B_P2 	= 10;


int motorSpeedRef 	= 0;
int state 			= 0;
int direzione 		= 1;




// #define SERVO_PIN   		9    	// Digital IO pin connected to the servo pin.
#define SERVO_PAN_PIN  		45    	// Digital IO pin connected to the servo pin.
#define SERVO_TILT_PIN  	44    	// Digital IO pin connected to the servo pin.

#define L_SIDE_IR			2		// non usato, dichiarato come ingresso ma libero
#define R_SIDE_IR			3		// sensore IR dx
#define GIRO_DX_PIN			21		// sensore su rotazione albero motore
#define GIRO_SX_PIN			20		// sensore su rotazione albero motore

#define R_SIDE_FRONT		26	// seleziona sensore frontale
#define L_SIDE_FRONT		28	// sensore posteriore

	

#define ledPin 				13
#define startBtn 			A5
#define laserPin 			8
#define tensionePin 		A4
#define	BT_CONNECTION_PIN	51

#define  TEST_MOTORE    	0
#define  TEST_STERZO    	1
#define  TEST_SENSORS   	2
#define  TEST_CONTROLLO 	3
#define  TEST_GIRO_SENSOR 	4


int sideIRstate 	= 0;
int sideIRcnt 		= 0;
int frontIRstate 	= 0;
int measureAvailable = 0;
unsigned long totTimeAtOne;
unsigned long measuringTime;
int startMeasureIRSide  = 0;

long odometroCnt, odometroDxCnt, odometroSxCnt;
char BTstate;	

float 	percentoUno;
float 	x;
int 	i;

char firstRun;
float lastPosition;

// tempo del controllo sterzo, posizione etc in ms
#define TEMPO_CONTROLLO 100

/*	V = w*r	-> w=V/raggio di curvatura
	a larghezza tra due ruote
	V1 = V*(r+a/2)/r = V(1 + a/2r)
	
*/
#define LAGHEZZA_A_MEZZI	0.09	// mezza carregiata (larghezza delle due ruote)
#define S_NEUTRO_FWD		-0.05	// con questo valore va crica diritto
#define S_NEUTRO_REV		-0.02	// con questo valore va crica diritto


// aggiorno per encoder da 35 ppr prima erano 20 
#define GIRO_RUOTA  		1.5   //  2.625	// 5.25	// mm per impulso*0.5 = sviluppo ruota[mm]/ppr (pulsi per rivoluzione)
#define GIRO_RUOTA_DX   	1.5   //	2.625	// 5.25	
#define GIRO_RUOTA_SX  		1.494  //	2.615 	// 5.25	
#define MIN_TIME_TRA_PULSE 	5   	//9	// 18	// tempo minimo tra impulsi encoder per evitare errate letture

/*
#define GIRO_RUOTA  		2.625	// 5.25	// mm per impulso*0.5 = sviluppo ruota[mm]/ppr (pulsi per rivoluzione)
#define GIRO_RUOTA_DX  		2.625	// 5.25	
#define GIRO_RUOTA_SX  		2.615 	// 5.25	
#define MIN_TIME_TRA_PULSE 	9		// 18	// tempo minimo tra impulsi encoder per evitare errate letture
*/

#define E_POSIZIONAMENTO  	10
#define E_APPROCCIO			300
#define BASELINE 			150.0	// carreggiata

/*
	MODERATA 		è il pwm di movimento normale
	APPROCCIO 		durante l'avvicinamento alla poszione
	ACCELERAZIONE 	nella fase del partenza
*/
#define FERMO			0	
#define MODERATA		190
#define APPROCCIO		150
#define ACCELERAZIONE	160
#define AVANTI			0
#define INDIETRO 		1

#define KP_DEF			4

// parametri lettura e/o scrittura
int 	statoRun 		= 0;
float 	odometro 		= 0;
float 	distanza 		= 0;
float 	raggiorSterzo	= 0.0;
float 	errore;
int		viaLibera 		= 0;
int 	motorSpeed 		= 0;
int 	motorSpeedValue = MODERATA;
int 	panAngle 		= 90;
int 	tiltAngle 		= 90;
char 	laser 			= 0;
int 	misura;
float	kp 				= KP_DEF;		// k proporzionale
float	kpTeta			= 2.0;			// k proporzionale per teta
float 	MAX_S			= 0.2;			// max_s = LAGHEZZA_A_MEZZI/Raggio massimo 
float 	teta 			= 0.0;					// teta attuale
float 	xpos, ypos 		= 0.0;
float 	tetaRef			= 0.0;
// temporaneo

float VA, VB;

int mode = TEST_STERZO;

char debugBT, monitorDati = 0;

 
  
void _sendBlueToothCommand(char command[])
{
    Serial1.print(command);
}
/*
 * setup BT connection
 */ 
void setupBlueToothConnection(){

String str;

    Serial1.begin(38400);                           // Set BluetoothBee BaudRate to default baud rate 38400
    Serial1.print("\r\n+STWMOD=0\r\n");             // set the bluetooth work in slave mode
    Serial1.print("\r\n+STNA=SeeedBTLamp\r\n");    	// set the bluetooth name as "SeeedBTSlave"
    Serial1.print("\r\n+STOAUT=1\r\n");             // Permit Paired device to connect me
    Serial1.print("\r\n+STAUTO=0\r\n");             // Auto-connection should be forbidden here
    delay(2000);                                            // This delay is required.
    Serial1.print("\r\n+INQ=1\r\n");                // make the slave bluetooth inquirable
    Serial.println("The slave bluetooth is inquirable!");
    delay(2000);                                            // This delay is required.
	
	Serial1.setTimeout(2000);
    Serial1.flush();
	
	Serial.println("init BT done!");
}	 
	 
void setup() {
    // initialize serial:
    Serial.begin ( 9600);
    Serial1.begin(38400);
    ESPserial.begin ( 9600);  // ID_002
    
    Serial.println("Init: ari2DC_esp.ino da PI 03mag17");
	
	pinMode(ledPin, 	OUTPUT);
	pinMode(laserPin, 	OUTPUT);
	pinMode(startBtn, 	INPUT_PULLUP);

	pinMode(BT_CONNECTION_PIN,  INPUT_PULLUP);
	
	pinMode(ECHO_PIN, 	INPUT);
	pinMode(TRIGGER_PIN,OUTPUT);

	// setup digital inputs for IR sensors
	pinMode(R_SIDE_FRONT, 		OUTPUT);
	pinMode(L_SIDE_FRONT, 		OUTPUT);
	pinMode(L_SIDE_IR, 			INPUT);
	pinMode(R_SIDE_IR, 			INPUT);
	pinMode(GIRO_DX_PIN,  		INPUT);
	pinMode(GIRO_SX_PIN,  		INPUT);

	
	digitalWrite(laserPin, LOW);

	// attaches the servo on pin .. to the servo object
	// sterzo.attach	 (SERVO_PIN);  		
	servoPan.attach  (SERVO_PAN_PIN);
	servoTilt.attach (SERVO_TILT_PIN);
	
	servoPan.write ( 90);
	servoTilt.write( 90);

	
	// Attach motors to the input pins:
	driver.attachMotorA(MTR_A_P1, MTR_A_P2);
	driver.attachMotorB(MTR_B_P1, MTR_B_P2);

	differenziale(0);
	
	
/* tolto t2_ovf in it	
// timer1 usato per servo	
//	Timer1.initialize(5000);         	// initialize timer1, and set a 5ms period
//	Timer1.attachInterrupt(IRSensors);  // attaches callback() as a timer overflow interrupt

// https://oscarliang.com/arduino-timer-and-interrupt-tutorial/
#define BIT(x) (0x01 << (x))

 	noInterrupts();           // disable all interrupts

	// Use Timer2 with 1 ms interrupts
	// OC2A & OC2B disconnected, mode 2 (CTC, Clear Timer on Compare match)
	TCCR2A = BIT(WGM21);
	// Don't force output compare (FOCA & FOCB), mode 2, Clock Select clk/128 (CS = 5)
	TCCR2B = BIT(CS22) | BIT(CS20);
	//  TCCR2A = BIT(WGM21);
	//  TCCR2B = 0;
	//  TCNT2  = 0;

	OCR2A = 125-1;            // compare match register 16MHz/256/2Hz
	//  TCCR2B |= (1 << WGM12);   // CTC mode
	//  TCCR2B |= (1 << CS12);    // 256 prescaler 
	TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
	
	interrupts();             // enable all interrupts
tolto t2 ovf end */ 
	
	attachInterrupt(digitalPinToInterrupt(R_SIDE_IR), cntSideSensor, CHANGE);
        // pin 3 interrupt 1 atMega
//	attachInterrupt(2, cntSideSensor, CHANGE);

	// odometro 
	// pin 21 (atMega) int 2	
	attachInterrupt(digitalPinToInterrupt(GIRO_DX_PIN), odometroDxMisuraHW, CHANGE);
	// pin 20 (atMega) int 3	
	attachInterrupt(digitalPinToInterrupt(GIRO_SX_PIN), odometroSxMisuraHW, CHANGE);

	/*
	il BT intasa la seriale !!!
	Status instruction port PIO1: low-disconnected, high-connected
	*/
//	if (BTstate != digitalRead(BT_CONNECTION_PIN)){


    delay(1000);

    setupBlueToothConnection();


	// ID_002 gestire BT e Wifi
 
	while (!digitalRead(BT_CONNECTION_PIN)){
		;
	}
 
	BTstate = 1;
		Serial.print("BT connection: ");
		Serial.println(digitalRead(BT_CONNECTION_PIN));
	delay (2000);
//	BTstate = digitalRead(BT_CONNECTION_PIN);


  
	Serial.flush();
	Serial1.flush();
	ESPserial.flush();  // ID_002
	
	firstRun = 1;
    Serial.println("setup done!");
    Serial1.println("setup done!");

    
}

unsigned long timeFrozen;
unsigned long lastTime, lastTimeFast;
char cambia;
float	teta_;

void loop() {

	mode = TEST_CONTROLLO;
	
	BTstate = digitalRead(BT_CONNECTION_PIN);
	
	// ID_002
	getCmd(); // ID_003
	
	getCmd2();
	rSeriale();
	

//    odometroMisura();			
	servoPan.write ( panAngle);
	servoTilt.write(tiltAngle);

	
    if (mode == TEST_CONTROLLO){

		if (firstRun){
			Serial.println("start TEST_CONTROLLO mode");
			Serial.println("controlloAttivo");
			Serial1.println("controlloAttivo");
			raggiorSterzo = 0.0;
			servoPan.write(panAngle);
			servoTilt.write(tiltAngle);
			kp = KP_DEF;
			digitalWrite( R_SIDE_FRONT,  LOW);
			digitalWrite( L_SIDE_FRONT, HIGH);
			firstRun = 0;
		}
	
		/* statoRun
		
			0: fermo
			1: controllo sterzo da sensore dx e distanza automatico
			1: controllo sterzo da sensore dx e distanza automatico
			2: controllo distanza, strzo da parametro
			4: controllo distanza, sterzo da parametro da tetaRef vs teta odometria
		*/
			
			//debounce 
			if (( digitalRead(startBtn) == 0) && (timeFrozen == 0)){
				timeFrozen = millis();
			}
			if (( digitalRead(startBtn) == 0) && (millis() - (timeFrozen) > 200)){
				if (cambia) statoRun = 0; //!statoRun;
				cambia = 0;
			}	
			if (  digitalRead(startBtn) == 1){
				timeFrozen = 0; 
				cambia     = 1;
			}

			// parte a 100 ms
			if ((millis()-lastTime) > TEMPO_CONTROLLO){
//				Serial.println(millis()-lastTime);

				lastTime = millis();

				noInterrupts();
					measureAvailable = 0;
					errore 		= 0.5 - misuraSideIR(0); // 1=libero  0=vicino
				interrupts();
				
				
				// controllo sterzo da sensore laterale dx
				if ((statoRun == 1)||(statoRun == 3)){
					// a seconda dello stato attivo sensore DX o SX
					
					if (statoRun == 1) {
						digitalWrite( R_SIDE_FRONT,  LOW);
						digitalWrite( L_SIDE_FRONT, HIGH);
						raggiorSterzo =   kp*errore; // aggiunto in differenziale + S_NEUTRO; 
					}
					else{
						digitalWrite( R_SIDE_FRONT, HIGH);
						digitalWrite( L_SIDE_FRONT,  LOW);
						raggiorSterzo =  -kp*errore; // aggiunto in differenziale + S_NEUTRO; 
					}
					// errore +/- 0.5

					
					if (raggiorSterzo < -MAX_S) raggiorSterzo = -MAX_S;
					if (raggiorSterzo >  MAX_S) raggiorSterzo =  MAX_S;
				}
				
//				if (statoRun != 0){
				if (statoRun == 2){
/*					if ( viaLibera < 0 ){
						motorSpeedRef = FERMO;
						direzione 	  = AVANTI;
					}
					else{*/
					motorSpeedRef = motorSpeedValue;
					direzione 	  = AVANTI;
					
				}

				// controllo sterzo da teta odometro
				/* stato 11 e 13 fanno si se si incontra  il sensore laterale
					il controllo passa a questo
				*/
				if ((statoRun == 4)||(statoRun == 11)||(statoRun == 13)){
					// Kp 1 ok
					// if teta > 3.14 -> teta -= 6.28
					// 0 - (6.27 - 6.28) = 0 - (-0.1) = atteso 0.1
					// 0 - 0.1  = - 0.1
					// 3.14 - 3.13 = 0.1
					// 3.14 - (3.15 - 6.28) = 6.27
					// if errore > 3.14 errore -= 6.28
					/* ID_001
					if (teta > 3.14) teta_ = teta - DUE_PI;
					else			 teta_ = teta;
					
					errore = tetaRef - teta_;
					if (errore > 3.14) errore -= DUE_PI;
					*/

					if (statoRun == 11) {
						digitalWrite( R_SIDE_FRONT,  LOW);
						digitalWrite( L_SIDE_FRONT, HIGH);
						
						if (errore >= 0.0){
							statoRun = 1; // 1=libero  0=vicino
						}
					}
					if (statoRun == 13){
						digitalWrite( R_SIDE_FRONT, HIGH);
						digitalWrite( L_SIDE_FRONT,  LOW);
						
						if (errore >= 0.0){
							statoRun = 3; // 1=libero  0=vicino
						}

					}
					
					errore = tetaRef - teta;	// ID_001
					raggiorSterzo =   kpTeta*errore; // aggiunto in differenziale + S_NEUTRO; 
					// errore +/- 0.5
					
					if (raggiorSterzo < -1.0) raggiorSterzo = -1.0;
					if (raggiorSterzo >  1.0) raggiorSterzo =  1.0;
				}
				
				/* gestione raggiungimento target
				
						all'approccio si diminuisce la velocità
						alla partenza metto pwm alto per tot spazio per accertarmi di partire
						
						viaggio è lo spazio percorso dall'ultimo avvio
						l'avvio è quando cambia il riferimento di posizione
						nel primo tratto metto pwm alto
						passo a pwm moderato
						nell'approccio (E_APPROCCIO) metto pwm approccio
						se sono sotto E_posizionamento mi fermo
				
				*/
				// direzione

				updatePosition();
				
				if ( distanza > odometro )	direzione = AVANTI;
				else						direzione = INDIETRO;
				// gestione velocità
				// se viaggio 
/*				if (abs(odometro - lastPosition) < 0.2){ // ??? m ???
					motorSpeedRef = ACCELERAZIONE;
				}
				else*/
					if ( abs(distanza - odometro) > E_APPROCCIO){
						motorSpeedRef = motorSpeedValue;
					}
					else
						if ( abs(distanza - odometro) > E_POSIZIONAMENTO){
//						if ( ( ((distanza - odometro) > E_POSIZIONAMENTO) &&  direzione) ||
//						     ( ((distanza - odometro) < E_POSIZIONAMENTO) && !direzione) ) {
							motorSpeedRef = APPROCCIO;
						}
						else{
							motorSpeedRef = FERMO;
							lastPosition  = odometro;
							statoRun      = 0;
						}
			}// fine parte temporizzata TEMPO_CONTROLLO ms

			if ((millis()-lastTimeFast) > 10){
				lastTimeFast = millis();

				if (statoRun == 0) motorSpeedRef = 0;
				
				// rampa sulla velocita'
				if (motorSpeedRef > motorSpeed)	motorSpeed += 2;
				if (motorSpeedRef < motorSpeed)	motorSpeed -= 15;

				if (motorSpeed > 250) motorSpeed = 250;
				if (motorSpeed <   1) motorSpeed = 0;
			}// fine temporizzata veloce
			
			
			differenziale(motorSpeed);
			
  
	}
	
    if (mode == TEST_MOTORE){
        Serial.println("start TEST_MOTORE mode, enter motorSpeed, direzione");
		while (1){
			while (Serial.available() > 0) {

				// format x, y
				// look for the next valid integer in the incoming serial stream:
				motorSpeedRef = Serial.parseInt();
				// do it again:
				direzione = Serial.parseInt();
				
				//PDURATION = Serial.parseInt();
				
				// look for the newline. That's the end of your
				// sentence:
				if (Serial.read() == '\n') {
					// constrain the values to -50 - 50
			//                    x = constrain(x, -150, 150);
			//                    y = constrain(y, -200, 200);

					Serial.print(motorSpeedRef);
					Serial.print(',');
					Serial.println(direzione);

				}    
			}
	/*		
			if (motorSpeedRef > motorSpeed)	motorSpeed++;
			if (motorSpeedRef < motorSpeed)	motorSpeed--;

			if (motorSpeed > 250) motorSpeed = 250;
			if (motorSpeed <   1) motorSpeed = 0;

			if (direzione)  driver.motorAForward(motorSpeed);
			else            driver.motorAReverse(motorSpeed);

		*/	
			
			
			
		}
	}
	
	
	
	
	
    if (mode == TEST_STERZO){

		if (firstRun){
		    Serial.println("start SERVO mode, enter servo position in degrees [0-180]");
			firstRun = 0;
		}
		//sterzo.write(raggiorSterzo); // sets the servo position according to the scaled value 
    }	
    
	


    if (mode == TEST_SENSORS){
	
		if (firstRun){
			Serial.println("start TEST_SENSORS mode");
			firstRun = 0;
		}
		misuraSideIR(1);	// 1 modalità debug
    }    

    if (mode == TEST_GIRO_SENSOR){
        Serial.println("start TEST_GIRO_SENSOR mode");
        while(1){
		// tbd
	    delay(10);
        }    
    }    

}

//------------------- fine main --------------------------------

/*
	pin 21 (atMega) int 2
	sotto interrupt incremmento contatore interi
	solo se R != 0
*/
void odometroDxMisuraHW(void){
unsigned long pulseTime;

	if ((millis() - pulseTime) < MIN_TIME_TRA_PULSE) return;
	pulseTime = millis();

	if (statoRun == 0) return;
	//Serial.println("dx");

	digitalWrite(ledPin, !digitalRead(ledPin));
	if (direzione == AVANTI)  odometroDxCnt ++;
	else                      odometroDxCnt --;
    
}

void odometroSxMisuraHW(void){
unsigned long pulseTime;

	if ((millis() - pulseTime) < MIN_TIME_TRA_PULSE) return;
	pulseTime = millis();

	if (statoRun == 0) return;
	//Serial.println("Sx");

	digitalWrite(ledPin, !digitalRead(ledPin));
	if (direzione == AVANTI)  odometroSxCnt ++;
	else                      odometroSxCnt --;

}


/*	
	un interrupt chiamato al cambio livello (minimo 50 us) integra il tempo a uno
	e il tempo di misura.
	conta inoltre il numero delle transizioni.

	 
*/
float misuraSideIR(int debug){

static float percentoUno;
static unsigned long 	copia_measuringTime;
static unsigned long 	copia_totTimeAtOne;
static int 				copia_frontIRstate 	= 0;
static int 				copia_sideIRstate 	= 0;
static int 				copia_sideIRcnt 	= 0;

	noInterrupts();
		copia_sideIRcnt     = sideIRstate;
		copia_totTimeAtOne  = totTimeAtOne;
		copia_frontIRstate  = frontIRstate;
		copia_measuringTime = measuringTime;

		totTimeAtOne        = 0;
		startMeasureIRSide 	= 1;		// riparte integrazione tempi
		cntSideSensor();
	interrupts();

	
	// se ho transizioni sufficienti leggo percentuale del tempo

	if (copia_sideIRcnt > 2)
		percentoUno = float(copia_totTimeAtOne) / float(copia_measuringTime);
		//if (percentoUno > 1.0) percentoUno = 1.0;
		
	else{
		if (digitalRead(R_SIDE_IR))	percentoUno =  1.0;
		else						percentoUno =  0.0;
	}	
		
	if (debug){
//				Serial.print(sideIRstate);
		Serial.print(copia_sideIRcnt);
		Serial.print(" ,");
		Serial.print(percentoUno);
		Serial.print(" ,");
		Serial.print(copia_totTimeAtOne);
		Serial.print(" ,");
		Serial.println(copia_measuringTime);
	}
		
	return(percentoUno);
	
}

/*
	il sensore IR ha tre distanze notevoli:
		sopra una certa d1 riporta 0 fisso
		sotto una certa d2 riporta 1 fisso
		tra d1 e d2 restituisce 0 e 1 in maniera instabile. 
	
	la routine è chiamata ad ogni cambio di livello
	l'idea è di misurare il tempo che il sensore sta a uno e 
	rapportarlo al tempo totale
	prendo il tempo 
*/

void cntSideSensor(){
static unsigned long time, firstTime, lastTime;
	//Serial.println("interrupt");
	
	sideIRstate ++;

	if (digitalRead(R_SIDE_IR)){
		// quando va a 1 prendo il tempo
		time = micros();
	}
	else{
		// quando va a zero calcolo il tempo che è stato a 1
		lastTime     = micros();
		totTimeAtOne+= lastTime - time;			// integro tempo a uno
		// tempo totale di luttura
		// measuringTime è
		measuringTime = lastTime - firstTime;
	}	

	if (startMeasureIRSide) {
		// questo è un nuovo ciclo di letture
		// il tempo totale inizia dall'ultima trasizione a zero
		startMeasureIRSide = 0;
		firstTime = lastTime;
		sideIRstate         = 0;
	}

}


/*
	pinMode(ECHO_PIN, INPUT);
	pinMode(TRIGGER_PIN, OUTPUT);
*/
float sonarMisura(int numeroMisure){
float distance = 0;
float max = -100000.0;
float min =  100000.0;
float somma = 0;


	for (int i = 0; i < numeroMisure; i++){

		noInterrupts();
		
		digitalWrite(TRIGGER_PIN, LOW);             // Set the trigger pin to low for 2uS
		delayMicroseconds(2);
		digitalWrite(TRIGGER_PIN, HIGH);            // Send a 10uS high to trigger ranging
		delayMicroseconds(10);
		digitalWrite(TRIGGER_PIN, LOW);             // Send pin low again

		interrupts();
		
		distance = pulseIn(ECHO_PIN, HIGH);        // Read in times pulse
		

		somma += distance;
		if (distance > max) 	max = distance;
		if (distance < min) 	min = distance;
		
		delay(50);      
	}
	somma -= (max + min);
	distance = somma/(58.0*(numeroMisure-2));
	return(distance);
}

/* sterzo con differenziale

raggiorSterzo indica lo scorrimento che applichiamo alle ruote

	VA = motorSpeed*(1-s)
	VB = motorSpeed*(1+s)
	
	S_NEUTRO è il valore per andare diritto
	si ottiene per taratura (prova)
	
*/

void differenziale(float motorSpeed){
static float rs;
static float Vlimite;

	if (direzione)	rs = raggiorSterzo + S_NEUTRO_FWD;
	else			rs = raggiorSterzo - S_NEUTRO_REV;
	
	/* quando viene fatta una curva con una ruota bloccata l'altra ruota va a 255.
	   partendo da fermo ci può essere slittamento.
	   In questo caso si limita la velocità della ruota che marcia.
	*/
	Vlimite = 255;
	if (rs >  1.0){
		rs =  1.0;
		Vlimite = motorSpeed;
	}
	if (rs < -1.0){
		rs = -1.0;
		Vlimite = motorSpeed;
	}
	
	VA = motorSpeed*(1.0+rs);
	VB = motorSpeed*(1.0-rs);
	
	if (VA > Vlimite) VA = Vlimite;
	if (VA <   0) VA =   0;
	
	if (VB > Vlimite) VB = Vlimite;
	if (VB <   0) VB =   0;
	
	if (direzione){
		driver.motorAReverse(VA);
		driver.motorBReverse(VB);
	}
	else{
		driver.motorAForward(VA);
		driver.motorBForward(VB);
	}
}

void updatePosition(void){

static long SxCnt_k_1 = 0;	// valore cnt a k-1
static long DxCnt_k_1 = 0;	// valore cnt a k-1
static long dDxCnt, dSxCnt;		// delta cnt
static long letturaDx;			// congelo cnt
static long letturaSx;			// congelo cnt
static float deltaC;			// delta cnt

	// valore complessivo: usato temporaneamente
	odometro = (odometroDxCnt + odometroSxCnt)*GIRO_RUOTA;

	// calcolo evoluzione nel periodo 
	
	// congelo le letture per lavorare su valori coerenti
	
	noInterrupts();
		letturaDx= odometroDxCnt;
		letturaSx= odometroSxCnt;
	interrupts();
	
	dDxCnt   = letturaDx - DxCnt_k_1;				// delta sx e dx in count
	dSxCnt   = letturaSx - SxCnt_k_1;
	
	deltaC   = (dDxCnt + dSxCnt)*GIRO_RUOTA;// avanzamento del centro nel periodo in mm
	
	DxCnt_k_1= letturaDx;							// memoria per prossimo ciclo
	SxCnt_k_1= letturaSx;
	
	// integro teta
	teta 	+= ((float)dDxCnt*GIRO_RUOTA_DX - (float)dSxCnt*GIRO_RUOTA_SX)*2.0/BASELINE;
	
	// constrain _theta to the range 0 to 2 pi
	
	/* ID_001
	if (teta > DUE_PI) teta -= DUE_PI;
	if (teta <    0.0) teta += DUE_PI;
	*/
	// integro posizioni
	xpos    +=  deltaC*cos(teta);
	ypos    +=  deltaC*sin(teta);
	
	// monitor dati
	if (monitorDati){
		Serial1.print(dDxCnt);
		Serial1.print(", ");
		Serial1.print(dSxCnt);
		Serial1.print(", ");
		Serial1.print(deltaC);
		Serial1.print(", ");
		Serial1.print(teta);
		Serial1.print(", ");
		Serial1.print(xpos);
		Serial1.print(", ");
		Serial1.print(ypos);
		Serial1.print(", ");
		Serial1.print(errore);
		Serial1.println();
	}

}


	void sendAnswer(char port, String risposta){

		if (debugBT){
			risposta += '_';
			Serial.print("cm: ");
			Serial.println(risposta);
		}
		if (port)	Serial1.println(risposta);
		else		 Serial.println(risposta);
	}
/*
	verifica se arrivano caratteri da seriale
	considero un pacchetto con la forma
	
	char 1:	parametro modificato, 
			S Scrivo sterzo e suo angolo						[gradi] 90 diritto 
																		180 dx
																		0   sx
			A tetaRef														
			B kpTeta														
			D scrivo distanza relativa da percorrere e valore 	[mm]
			d lettura distanza assoluta (odometro)				[mm]
			V scrivo setpoint velocita' con valore motorSpeedValue
			v leggo velocità (motorSpeedRef)
			e errore
			l libero (viaLibera) stato sensore anteriore
			r statoRun	
			R statoRun (scrivi) 0: fermo, 1: controllo con sensore dx, 2: sterzo esterno
			  fermata al superamento della D attiva con ritorno di statoRun a 0
			P angolo pan
			T angolo tilt
			L laser (0=off, else On)
			a xpos
			b ypos
			c teta
			m misura con sonar [cm]
			t legge tensione batteria
			w
			z
			s	valore sterzo (lettura)
			C	raggio di curva
			K	kp guadagno proporzionale
			
			1	scrivi valore debug
			2   leggi  valore debug
			x   BTstate
			Z	MAX_S
*/

void getCmd(void){
static float x, y;
static char smComandi = 0;
static char inCmd;
static unsigned long time, cmdTime;
static char portCmd, inChar;
//static char inChar;
static String risposta;

static int inByte;

/* test BT
	if (Serial.available() > 0) {
		inByte = Serial.read();
		Serial1.write(inByte);
	}	

	if (Serial1.available() > 0) {
		inByte = Serial1.read();
		Serial.write(inByte);
	}	
	return;
*/

	if (!digitalRead(BT_CONNECTION_PIN)){
		Serial1.flush();
	}

	// se il pacchetto non arriva completo in un secondo 
	// resetto ricezione
	if ((smComandi != 0) && ((millis() - cmdTime) > 3000 )) {
		if (portCmd == 0)   Serial.println("Timeout");
		if (portCmd == 1)  Serial1.println("Timeout");
		if (portCmd == 2)  Serial2.println("Timeout");
		
		if (debugBT){
			Serial.println("timeOut");
		}
		
		Serial.flush();
		Serial1.flush();
		//Serial2.flush();
	
		smComandi = 0;
	}

 
	BTstate = 1;

  
	// se BT connesso gestisco caratteri da BT
	if ( ( Serial.available() > 0)				|| 
	    ((Serial1.available() > 0) && BTstate))  {


		switch (smComandi) {
			case 0: // attesa 1rst valore

				// ID_002
				if ( Serial.available () > 0) portCmd = 0;     // seriale usb
				if (Serial1.available () > 0) portCmd = 1;     // BT
				// if (Serial2.available () > 0) port = 2;     // esp		ID_003

				if (portCmd == 0) inCmd =  Serial.read();
				if (portCmd == 1) inCmd = Serial1.read();
				if (portCmd == 2) inCmd = Serial2.read();
				cmdTime = millis();

				debugBT = 1;
						if (debugBT){
							Serial.print("inCmd: ");
							Serial.println(inCmd);
						}
				
				switch (inCmd) {
					case '+':
						// comando da BT
							smComandi = 10;
							break;
					// scrivo valore
					case 'A': 
					case 'B': 
					case 'C': 
					case 'K': 
					case 'D': 
					case 'L': 
					case 'M': 
					case 'P': 
					case 'R': 
					case 'S': 
					case 'T': 
					case 'V': 
					case 'Z': 
					case '1': 
							smComandi = 1;		// stato a seconda del numero di
												// parametri da ricevere
						break;
		
					case 'a': 
					case 'b': 
					case 'c': 
					case 'd': 
					case 'e': 
					case 'l': 
					case 'm': 
					case 'r': 
					case 's': 
					case 't': 
					case 'v': 
					case 'x': 
					case 'w': 
					case 'z': 
					case '2': 
					case '3': 
							smComandi = 2;		// stato a seconda del numero di
												// parametri da ricevere
												// questa è lettura quindi segue subito risposta
						break;
				}
				break;
				
			case 1: // attende 1 valore
				// look for the next valid float in the incoming serial stream:
				
				if (portCmd == 0)  x =  Serial.parseFloat();
				if (portCmd == 1)  x = Serial1.parseFloat();
				if (portCmd == 2)  x = Serial2.parseFloat();

				smComandi = 2; 	// mette in esecuzione valore

				
				if (debugBT){
					Serial.print("x: ");
					Serial.println(x);
				}


				
				break;	
				
			case 2: // attesa terminatore

				if (portCmd == 0)  inChar =  Serial.read();
				if (portCmd == 1)  inChar = Serial1.read();
				if (portCmd == 2)  inChar = Serial2.read();

				if (debugBT){
					Serial.print("eol: ");
					Serial.println(inChar);
				}
				if ((inChar  != '\n') && (inChar  != '\r'))  break;

				switch (inCmd) {
	
					case 'A': 
							tetaRef  = x*3.14/180.0;
							smComandi = 0;		
							risposta = "A: " + String( tetaRef, 3);
						break;
		
					case 'B': 
							kpTeta = x;
							smComandi = 0;		
							risposta = "B: " + String(kpTeta, 3);
						break;

					case 'D': 
							distanza += x;
							smComandi = 0;		
							risposta = "D: " + String( distanza, 3);
						break;
		
					case 'C': 
							if (abs(x) < 1000) 
								raggiorSterzo = LAGHEZZA_A_MEZZI/x;
							else
								raggiorSterzo = 0.0;
							smComandi = 0;		
							risposta = "S: " + String(raggiorSterzo, 3);
						break;
		
					case 'K': 
							kp = x;
							smComandi = 0;		
							risposta = "K: " + String(kp, 3);
						break;

					case 'L': 
							laser = x;
							smComandi = 0;		
							risposta = "L: ";// + String(laser);
							if (laser==0){
								digitalWrite(laserPin, LOW);
								risposta += "0";
								}
							else{
								digitalWrite(laserPin, HIGH);
								risposta += "1";
							}
						break;

					case 'M': 
							monitorDati = x;
							smComandi = 0;		
							risposta = "M: " + String(monitorDati);
						break;

					case 'P': 
							panAngle = x;
							smComandi = 0;		
							risposta = "P: " + String(panAngle);
						break;
		
					case 'R': 
							statoRun = x;
							smComandi = 0;		
							risposta = "R: " + String(statoRun);
						break;

					case 'S': 
							raggiorSterzo = x;
							smComandi = 0;		
							risposta = "S: " + String( raggiorSterzo, 3);
						break;

					case 'T': 
							tiltAngle = 180 - x;		// servomotore girato
							smComandi = 0;		
							risposta = "T: " + String(x);
						break;
		
					case 'V': 
							motorSpeedValue = x;
							smComandi = 0;		
							risposta = "V: " + String(motorSpeedValue);
						break;
		
					case 'Z': 
							MAX_S = x;
							smComandi = 0;		
							risposta = "Z: " + String(MAX_S, 3);
						break;

					case '1': 
							VA = x;
							smComandi = 0;		
							risposta = "1: " + String(x, 3);
						break;
		
					case 'a': 
							smComandi = 0;		
							risposta = "a: " + String(xpos);
						break;
		
					case 'b': 
							smComandi = 0;		
							risposta = "b: " + String(ypos);
						break;
		
					case 'c': 
							smComandi = 0;		
							risposta = "c: " + String(teta);
						break;
		

					case 'd': 
							smComandi = 0;		
							risposta = "d: " + String(odometro);
						break;
		
					case 'e': 
							smComandi = 0;		
							risposta = "e: " + String(errore);
						break;
		
					case 'l': 
							smComandi = 0;		
							risposta = "l: " + String(viaLibera);
						break;
						
					case 'm': 
							smComandi = 0;		
							misura = sonarMisura(5);
							risposta = "m: " + String(misura);
						break;

					case 'r': 
							smComandi = 0;		
							risposta = "r: " + String(statoRun);
						break;

					case 's': 
							smComandi = 0;		
							risposta = "s: " + String(raggiorSterzo, 3);
						break;
						
					case 't': 
							smComandi = 0;		
							risposta = "b: " + String(analogRead(tensionePin));
						break;

					case 'v': 
							smComandi = 0;		
							risposta = "v: " + String(motorSpeed);
						break;

					case 'w': 
							smComandi = 0;		
							risposta = "w: " + String(VB);
						break;
						
					case 'z': 
							smComandi = 0;		
							risposta = "z: " + String(VA);
						break;

					case 'x': 
							smComandi = 0;		
							if (BTstate) 	risposta = "x: 1";
							else 			risposta = "x: 0";
							
						break;

					case '2': 
							smComandi = 0;		
							risposta = "Sx: " + String(odometroSxCnt);
						break;

					case '3': 
							smComandi = 0;		
							risposta = "Dx: " + String(odometroDxCnt);
						break;
				}
				
				sendAnswer(portCmd, risposta);
				
				break;
			case 10: // attesa terminatore BT
     
					if (portCmd == 0)  inChar =  Serial.read();
					if (portCmd == 1)  inChar = Serial1.read();
					if (portCmd == 2)  inChar = Serial2.read();
					
					if ((inCmd == '\n') || (inCmd == '\r')) smComandi = 0;
					
					if (debugBT){
						Serial.print("eoldBT: ");
						Serial.println(inCmd);
					}
				break;
			default:
			
				smComandi = 0;
				break;
		}
		
	}// in serialLine.available
}


//ID_002 *********************ale

  void sendAnswer2(char port){
    //*********ale
    ESPserial.println("!"+risposta+"?");
    
    Serial.println("!"+risposta+"?");
    //*********ale
    risposta="";
  }

void richieste()
{       
static float x, y;

        switch (char(inputString[1])) {
					case 'a': 
							risposta = "a: " + String(xpos);
						break;
		
					case 'b': 
							risposta = "b: " + String(ypos);
						break;
		
					case 'c': 
							risposta = "c: " + String(teta);
						break;
		

					case 'd': 
							risposta = "d: " + String(odometro);
						break;
		
					case 'e': 
							risposta = "e: " + String(errore);
						break;
		
					case 'l': 
							risposta = "l: " + String(viaLibera);
						break;
						
					case 'm': 
							misura = sonarMisura(5);
							risposta = "m: " + String(misura);
						break;

					case 'r': 
							risposta = "r: " + String(statoRun);
						break;

					case 's': 
							risposta = "s: " + String(raggiorSterzo, 3);
						break;
						
					case 't': 
							risposta = "b: " + String(analogRead(tensionePin));
						break;

					case 'v': 
							risposta = "v: " + String(motorSpeed);
						break;

					case 'w': 
							risposta = "w: " + String(VB);
						break;
						
					case 'z': 
							risposta = "z: " + String(VA);
						break;

					case 'x': 
							if (BTstate) 	risposta = "x: 1";
							else 			risposta = "x: 0";
							
						break;

					case '2': 
							risposta = "Sx: " + String(odometroSxCnt);
						break;

					case '3': 
							risposta = "Dx: " + String(odometroDxCnt);
						break;
        }
		
		sendAnswer2(port);

  
  }

void comandi()
{         
static float x, y;

//          x==(inputString.substring(2)).toInt();
		//with only one parameter looks for a given substring from the position given to the end of the string. 
		x = (inputString.substring(2)).toFloat();

		Serial.print  ("comandi- ch[1]: ");
		Serial.print  (inputString[1]);
		Serial.print  (",xstr: ");
		Serial.print	(inputString.substring(2));
		Serial.print  (",x: ");
		Serial.println(x);

		switch (char(inputString[1])) {
			case 'A': 
					tetaRef  = x*3.14/180.0;
					risposta = "A: " + String( tetaRef, 3);
				break;

			case 'B': 
					kpTeta = x;
					risposta = "B: " + String(kpTeta, 3);
				break;

			case 'D': 
					distanza += x;
					risposta = "D: " + String( distanza, 3);
				break;

			case 'C': 
					if (abs(x) < 1000) 
						raggiorSterzo = LAGHEZZA_A_MEZZI/x;
					else
						raggiorSterzo = 0.0;
					risposta = "S: " + String(raggiorSterzo, 3);
				break;

			case 'K': 
					kp = x;
					risposta = "K: " + String(kp, 3);
				break;

			case 'L': 
					Serial.println(x);
					if (x == 0.0){
						Serial.println("L0");
						digitalWrite(laserPin, LOW);
						laser = 0;
					}
					else{
						Serial.println("L1");
						digitalWrite(laserPin, HIGH);
						laser = 1;
					}
					risposta = "L: " + String(x);
				break;

			case 'M': 
					monitorDati = x;
					risposta = "M: " + String(monitorDati);
				break;

			case 'P': 
					panAngle = x;
					risposta = "P: " + String(panAngle);
				break;

			case 'R': 
					statoRun = x;
					risposta = "R: " + String(statoRun);
				break;

			case 'S': 
					raggiorSterzo = x;
					risposta = "S: " + String( raggiorSterzo, 3);
				break;

			case 'T': 
					tiltAngle = 180 - x;		// servomotore girato
					risposta = "T: " + String(x);
				break;

			case 'V': 
					motorSpeedValue = x;
					risposta = "V: " + String(motorSpeedValue);
				break;

			case 'Z': 
					MAX_S = x;
					risposta = "Z: " + String(MAX_S, 3);
				break;

			case '1': 
					VA = x;
					risposta = "1: " + String(x, 3);
				break;
		
        default:
            risposta="mica capito";
        }
        sendAnswer2(port);
  }


void getCmd2(void){
static float x, y;
static int inByte;

	if (inputString!="")
	{
	  //qui tutti i comandi da eseguire 1 richieste 3 comandi
	  // char[0] 1 o 3 per richieste o comandi 
	  // char[1] indica cosa
	  // teminatore ?

		Serial.print("getCmd2: ");
		Serial.println(inputString);
//		Serial.println(char(inputString[0]));
		
		for (int i=0; i < inputString.length(); i++){
			Serial.print(i);
			Serial.print(": ");
			Serial.println(inputString.charAt(i));
		}
//		switch (char(inputString[0])) {
		switch (inputString.charAt(0)) {
		  case '1':
			richieste();
			break;
		  case '3':
			comandi();
			break;
		  default:
			risposta="mica capito";
			sendAnswer2(port);		// l'argomento dovrebbe essere la porta su cui rispondere
		
		}
	  
	  // fine comandi 
	  inputString="";
	}

}


/*	ID_002
	leggo stringa da seriale
	il terminatore della stringa è ?
	! flush
	? terminatore
*/
void rSeriale(void)	
{
static unsigned long time, cmdTime;
static char c;
static int numero = 0;
	port = 0;
	if (ESPserial.available() ==0) return;
	
	c = (char)ESPserial.read(); 
	// arrivano e non so da dove
    if (c == '\n' || c == '\r')		return;
	
	
	Serial.print(numero);
	Serial.print(": ");
	Serial.println(c);
	numero ++;
	
    if (c=='!')		// flush stringa
    {
  		inputStringTmp = "";
  		ESPserial.flush();
  		okcomm=1;
  		numero = 0;
  		Serial.println("flush");
    }
    if (c != '?' && c != '!' && okcomm==1)
    {
      cmdTime = millis();
  		inputStringTmp += c;
	  	Serial.println(inputStringTmp);
    }
    if (c == '?') {
		  inputString=inputStringTmp;
		  Serial.println(inputString);
		  okcomm=0;
		  Serial.println("c==?");
    }
}
