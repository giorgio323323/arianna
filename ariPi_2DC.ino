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
			
			
 * Simple test for the DRV8833 library.
 * The DRV8833 is a dual motor driver carrier made by Pololu.
 * You can find it here: https://www.pololu.com/product/2130

 */

#include <SPI.h>
#include <SD.h>
#include <Servo.h> 
#include <TimerOne.h>
// #include <NewPing.h> usa interrupt timer2

//Servo sterzo;          	// create servo object to control a servo 
Servo servoPan;      	// create servo object to control a servo 
Servo servoTilt;      	// create servo object to control a servo 

#define TRIGGER_PIN	12
#define ECHO_PIN	11		// FILO BIANCO BLU

  
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

#define L_SIDE_IR			2
#define R_SIDE_IR			3		// sensore IR dx
#define GIRO_PIN			21		// sensore su rotazione albero motore

#define R_SIDE_FRONT		26	// seleziona sensore frontale
#define R_SIDE_REAR			28	// sensore posteriore

	

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

long odometroCnt;
char BTstate;	

float 	percentoUno;
float 	x;
int 	i;

char firstRun;
float lastPosition;

// tempo del controllo sterzo, posizione etc in ms
#define TEMPO_CONTROLLO 50

/*	V = w*r	-> w=V/raggio di curvatura
	a larghezza tra due ruote
	V1 = V*(r+a/2)/r = V(1 + a/2r)
	
*/
#define LAGHEZZA_A_MEZZI	0.09	// mezza carregiata (larghezza delle due ruote)
#define S_NEUTRO_FWD		-0.05	// con questo valore va crica diritto
#define S_NEUTRO_REV		-0.02	// con questo valore va crica diritto

// mm per impulso = sviluppo ruota[mm]/ppr (pulsi per rivoluzione)
#define GIRO_RUOTA  		10.5
#define E_POSIZIONAMENTO  	10
#define E_APPROCCIO			300

/*
	MODERATA 		è il pwm di movimento normale
	APPROCCIO 		durante l'avvicinamento alla poszione
	ACCELERAZIONE 	nella fase del partenza
*/
#define FERMO			0	
#define MODERATA		190
#define APPROCCIO		135
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
float 	MAX_S			= 0.2;			// max_s = LAGHEZZA_A_MEZZI/Raggio massimo 

// temporaneo

float VA, VB;

int mode = TEST_STERZO;

char debugBT = 0;

 
  
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
    
    Serial.println("Init: ari2DC.ino da PI 23gen17");
	
	pinMode(ledPin, 	OUTPUT);
	pinMode(laserPin, 	OUTPUT);
	pinMode(startBtn, 	INPUT_PULLUP);

	pinMode(BT_CONNECTION_PIN,  INPUT_PULLUP);
	
	pinMode(ECHO_PIN, 	INPUT);
	pinMode(TRIGGER_PIN,OUTPUT);

	// setup digital inputs for IR sensors
	pinMode(R_SIDE_FRONT, 		OUTPUT);
	pinMode(R_SIDE_REAR, 		OUTPUT);
	pinMode(L_SIDE_IR, 			INPUT);
	pinMode(R_SIDE_IR, 			INPUT);
	pinMode(GIRO_PIN,  			INPUT);

	
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
        // pin 3 interrupt 2 ID_000
//	attachInterrupt(2, cntSideSensor, CHANGE);

	// odometro dx
	// pin 21 (atMega) int 2	
	attachInterrupt(digitalPinToInterrupt(GIRO_PIN), odometroMisuraHW, RISING);

	/*
	il BT intasa la seriale !!!
	Status instruction port PIO1: low-disconnected, high-connected
	*/
//	if (BTstate != digitalRead(BT_CONNECTION_PIN)){
    delay(1000);
    setupBlueToothConnection();
	
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
	
	firstRun = 1;
    Serial.println("setup done!");
    Serial1.println("setup done!");

    
}

unsigned long timeFrozen;
unsigned long lastTime, lastTimeFast;
char cambia;


void loop() {

	mode = TEST_CONTROLLO;
	
	BTstate = digitalRead(BT_CONNECTION_PIN);
	
	getCmd();

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
			firstRun = 0;
		}
	
		/* statoRun
		
			0: fermo
			1: controllo sterzo e distanza automatico
			2: controllo distanza, strzo da parametro
		
		
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
			if ((millis()-lastTime) > 100){
//				Serial.println(millis()-lastTime);

				lastTime = millis();



				noInterrupts();
					measureAvailable = 0;
					errore 		= 0.5 - misuraSideIR(0); // 1=libero  0=vicino
					viaLibera 	= frontIRstate;
				interrupts();

				
				// controllo sterzo da sensore laterale dx
				if (statoRun == 1){
					// errore +/- 0.5
					raggiorSterzo =  kp*errore; // aggiunto in differenziale + S_NEUTRO; 

					
					if (raggiorSterzo < -MAX_S) raggiorSterzo = -MAX_S;
					if (raggiorSterzo >  MAX_S) raggiorSterzo =  MAX_S;
				}
				
//				if (statoRun != 0){
				if (statoRun == 2){
					if ( viaLibera < 0 ){
						motorSpeedRef = FERMO;
						direzione 	  = AVANTI;
					}
					else{
						motorSpeedRef = motorSpeedValue;
						direzione 	  = AVANTI;
					}
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
				
				odometro = odometroCnt*GIRO_RUOTA;
				
				if ( distanza > odometro )	direzione = AVANTI;
				else						direzione = INDIETRO;
				// gestione velocità
				// se viaggio 
				if (abs(odometro - lastPosition) < 0.2){
					motorSpeedRef = ACCELERAZIONE;
				}
				else
					if ( abs(distanza - odometro) > E_APPROCCIO){
						motorSpeedRef = motorSpeedValue;
					}
					else
//						if ( abs(distanza - odometro) > E_POSIZIONAMENTO){
						if ( ( ((distanza - odometro) > E_POSIZIONAMENTO) &&  direzione) ||
						     ( ((distanza - odometro) < E_POSIZIONAMENTO) && !direzione) ) {
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
				if (motorSpeedRef > motorSpeed)	motorSpeed += 5;
				if (motorSpeedRef < motorSpeed)	motorSpeed -= 15;

				if (motorSpeed > 250) motorSpeed = 250;
				if (motorSpeed <   1) motorSpeed = 0;
			}// fine temporizzata veloce
			
			
			// a seconda della direzione si attiva sensore frontale 
			// o posteriore
			// la guida all'indietro con sensore frontale è instabile
			if (direzione) {
				digitalWrite( R_SIDE_FRONT, HIGH);
				digitalWrite( R_SIDE_REAR ,  LOW);
			}
			else{
				digitalWrite( R_SIDE_FRONT,  LOW);
				digitalWrite( R_SIDE_REAR , HIGH);
			}
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
void odometroMisuraHW(void){

  if (statoRun == 0) return;
  
  //digitalWrite(ledPin, !digitalRead(ledPin));
  if (direzione == AVANTI)  odometroCnt ++;
  else                      odometroCnt --;
    
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

	if (direzione)	rs = raggiorSterzo + S_NEUTRO_FWD;
	else			rs = raggiorSterzo - S_NEUTRO_REV;
	
	
	if (rs >  1.0) rs =  1.0;
	if (rs < -1.0) rs = -1.0;
	
	
	VA = motorSpeed*(1.0+rs);
	VB = motorSpeed*(1.0-rs);
	
	if (VA > 255) VA = 255;
	if (VA <   0) VA =   0;
	
	if (VB > 255) VB = 255;
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

	void sendAnswer(char port, String risposta){
	
		if (debugBT){
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
			m misura con sonar [cm]
			b legge tensione batteria
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
static int inCmd;
static unsigned long time, cmdTime;
static char port, inChar;
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
		if (port)	Serial1.println("Timeout");
		else		 Serial.println("Timeout");
		
		if (debugBT){
			Serial.println("tiemOut");
		}
		
		Serial.flush();
		Serial1.flush();
	
		smComandi = 0;
	}

	
	// se BT connesso gestisco caratteri da BT
	if ( (Serial.available () > 0)				|| 
	    ((Serial1.available() > 0) && BTstate))  {


		switch (smComandi) {
			case 0: // attesa 1rst valore

				if (Serial.available () > 0) port = 0; 
				else						 port = 1;

				if (port)	inCmd = Serial1.read();
				else		inCmd = Serial.read();
				cmdTime = millis();
				
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
					case 'S': 
					case 'D': 
					case 'R': 
					case 'V': 
					case 'P': 
					case 'T': 
					case 'L': 
					case 'C': 
					case 'K': 
					case 'Z': 
					case '1': 
							smComandi = 1;		// stato a seconda del numero di
												// parametri da ricevere
						break;
		
					case 'd': 
					case 'e': 
					case 'v': 
					case 'l': 
					case 'r': 
					case 'm': 
					case 'b': 
					case 's': 
					case 'w': 
					case 'z': 
					case 'x': 
					case '2': 
							smComandi = 2;		// stato a seconda del numero di
												// parametri da ricevere
												// questa è lettura quindi segue subito risposta
						break;
				}
				break;
			case 1: // attende 1 valore
				// look for the next valid integer in the incoming serial stream:
				
				if (port)	x = Serial1.parseFloat();
				else		x =  Serial.parseFloat();
				smComandi = 2; 	// mette in esecuzione valore

				
				if (debugBT){
					Serial.print("x: ");
					Serial.println(x);
				}


				
				break;	
				
			case 2: // attesa terminatore


				if (port){
					inChar =Serial1.read();
				}
				else{
					inChar =Serial.read();
				}

				if (debugBT){
					Serial.print("eol: ");
					Serial.println(inChar);
				}
				if (inChar  != '\n') break;

				switch (inCmd) {
					case 'S': 
							raggiorSterzo = x;
							smComandi = 0;		
							risposta = "S: " + String( raggiorSterzo, 3);
						break;
		
					case 'D': 
							distanza += x;
							smComandi = 0;		
							risposta = "D: " + String( distanza, 3);
						break;
		
					case 'R': 
							statoRun = x;
							smComandi = 0;		
							risposta = "R: " + String(statoRun);
						break;

					case 'V': 
							motorSpeedValue = x;
							smComandi = 0;		
							risposta = "V: " + String(motorSpeedValue);
						break;
		
					case 'P': 
							panAngle = x;
							smComandi = 0;		
							risposta = "P: " + String(panAngle);
						break;
		
					case 'T': 
							tiltAngle = 180 - x;		// servomotore girato
							smComandi = 0;		
							risposta = "T: " + String(x);
						break;
		
					case 'L': 
							laser = x;
							smComandi = 0;		
							risposta = "L: " + String(laser);
							if (laser==0) 	digitalWrite(laserPin, LOW);
							else			digitalWrite(laserPin, HIGH);
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
		
					case 'd': 
							smComandi = 0;		
							risposta = "d: " + String(odometro);
						break;
		
					case 'e': 
							smComandi = 0;		
							risposta = "e: " + String(errore);
						break;
		
					case 'v': 
							smComandi = 0;		
							risposta = "v: " + String(motorSpeed);
						break;

					case 'l': 
							smComandi = 0;		
							risposta = "l: " + String(viaLibera);
						break;
						
					case 'r': 
							smComandi = 0;		
							risposta = "r: " + String(statoRun);
						break;

					case 'm': 
							smComandi = 0;		
							misura = sonarMisura(5);
							risposta = "m: " + String(misura);
						break;

					case 'b': 
							smComandi = 0;		
							risposta = "b: " + String(analogRead(tensionePin));
						break;

					case 's': 
							smComandi = 0;		
							risposta = "s: " + String(raggiorSterzo, 3);
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
							risposta = "2: " + String(odometroCnt);
						break;
				}
				
				sendAnswer(port, risposta);
				
				break;
			case 10: // attesa terminatore BT
					if (port)	inCmd = Serial1.read();
					else		inCmd = Serial.read();
					
					if (inCmd == '\n') smComandi = 0;
					
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
