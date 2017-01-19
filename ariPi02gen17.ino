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
 
 * Simple test for the DRV8833 library.
 * The DRV8833 is a dual motor driver carrier made by Pololu.
 * You can find it here: https://www.pololu.com/product/2130

 */

#include <SPI.h>
#include <SD.h>
#include <Servo.h> 
#include <TimerOne.h>
// #include <NewPing.h> usa interrupt timer2

Servo sterzo;          	// create servo object to control a servo 
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
const int inputA1 	= 5;
const int inputA2 	= 6;


int motorSpeedRef 	= 0;
int state 		= 0;
int direzione 		= 1;




#define SERVO_PIN   		9    	// Digital IO pin connected to the servo pin.
#define SERVO_PAN_PIN  		10    	// Digital IO pin connected to the servo pin.
#define SERVO_TILT_PIN  	3    	// Digital IO pin connected to the servo pin.

#define L_SIDE_IR	2
#define  R_SIDE_IR	4		// sensore IR dx
#define GIRO_PIN	7		// sensore su rotazione albero motore

#define ledPin 		13
#define startBtn 	A5
#define laserPin 	8
#define tensionePin A4


#define  TEST_MOTORE    0
#define  TEST_STERZO    1
#define  TEST_SENSORS   2
#define  TEST_CONTROLLO 3
#define  TEST_GIRO_SENSOR 4


int sideIRstate 	= 0;
int sideIRcnt 		= 0;
int frontIRstate 	= 0;
int measureAvailable = 0;
unsigned long totTimeAtOne;
unsigned long measuringTime;
int startMeasureIRSide  = 0;


float 	percentoUno;
float 	x;
int 	i;
int 	statoRun = 0;

char firstRun;
float lastPosition;


#define DX_POCO 130
#define SX_POCO	 60
#define DIRITTO  80

// sviluppo ruota[mm]/ppr (pulsi per rivoluzione)
#define GIRO_RUOTA  		94.2
#define E_POSIZIONAMENTO  	100
#define E_APPROCCIO			300

/*
	MODERATA 		è il pwm di movimento normale
	APPROCCIO 		durante l'avvicinamento alla poszione
	ACCELERAZIONE 	nella fase del partenza
*/
#define FERMO		0	
#define MODERATA		190
#define APPROCCIO		135
#define ACCELERAZIONE	230
#define AVANTI		0
#define INDIETRO 	1

// parametri lettura e/o scrittura
float odometro 		= 0;
float distanza 		= 0;
float angoloSterzo;
float 	errore;
int		viaLibera 	= 0;
int motorSpeed 		= 0;
int motorSpeedValue = MODERATA;
int panAngle = 90;
int tiltAngle = 90;
char laser = 0;
int misura;


//




int mode = TEST_STERZO;
 

     
void setup() {
    // initialize serial:
    Serial.begin(9600);
    
    Serial.println("Init: ariDC.ino da PI 30dic16");
	
	pinMode(ledPin, OUTPUT);
	pinMode(laserPin, OUTPUT);
	pinMode(startBtn, INPUT_PULLUP);
	
	pinMode(ECHO_PIN, INPUT);
	pinMode(TRIGGER_PIN, OUTPUT);
	
	digitalWrite(laserPin, LOW);
	
	/*  For Phase-correct PWM of 31.250 kHz (prescale factor of 1)
	Use these two lines in the setup function:
	Setting   Divisor   Frequency
	0x01    1     62500
	0x02      8     7812.5
	0x03      64    976.5625   <--DEFAULT
	0x04    256     244.140625
	0x05    1024    61.03515625

	TCCR0B = (TCCR0B & 0b11111000) | <setting>;

	For Fast PWM of 62.500 kHz (prescale factor of 1)
	Use these two lines in the setup function:


	And modify the line in the wiring.c function in the Arduino program files
	hardware\arduino\cores\arduino\wiring.c :
	#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(1 * 256))
	*/

	TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
	TCCR0B = _BV(CS00); 

	// Start the serial port:
	Serial.begin(9600);


	// Attach a motor to the input pins:
	driver.attachMotorA(inputA1, inputA2);
	Serial.println("Ready!");
	
	// attaches the servo on pin .. to the servo object
	sterzo.attach	 (SERVO_PIN);  		
	servoPan.attach  (SERVO_PAN_PIN);
	servoTilt.attach (SERVO_TILT_PIN);
	
	
	
	// setup digital inputs for IR sensors
	pinMode(L_SIDE_IR, INPUT);
	pinMode(R_SIDE_IR, INPUT);
	pinMode(GIRO_PIN, INPUT);
	
	
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
	
//	attachInterrupt(digitalPinToInterrupt(R_SIDE_IR), cntSideSensor, CHANGE);
        // pin 3 interrupt 2 ID_000
	attachInterrupt(2, cntSideSensor, CHANGE);
	
	interrupts();             // enable all interrupts

	firstRun = 1;
    Serial.println("done!");

    
}



void loop() {
int timeFrozen;
char cambia;

	mode = TEST_CONTROLLO;
	
	getCmd();

    odometroMisura();			
	servoPan.write (panAngle);
	servoTilt.write(tiltAngle);

	
    if (mode == TEST_CONTROLLO){

		if (firstRun){
		Serial.println("start TEST_CONTROLLO mode");
			angoloSterzo = DIRITTO;
			sterzo.write(angoloSterzo); 
			servoPan.write(panAngle);
			firstRun = 0;
		}
	
		/* statoRun
		
			0: fermo
			1: controllo sterzo e distanza automatico
			2: controllo distanza, strzo da parametro
		
		
		*/
		
		
		/*	i sensori riportano 
		-99 (livello zero quando impegnati)
		 99 (livello uno  quando liberi)
		 ci sono dei valori intermedi
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


			if (measureAvailable){
				noInterrupts();
					measureAvailable = 0;
					errore 		= 0.5 - misuraSideIR(0); // 1=libero  0=vicino
					viaLibera 	= frontIRstate;
				interrupts();

				// controllo sterzo da sensore laterale dx
				if (statoRun == 1){
					// errore +/- 0.5
					angoloSterzo = 90 - errore*100.0; 
					if (angoloSterzo <   0) angoloSterzo =0;
					if (angoloSterzo > 180) angoloSterzo = 180;
				}
				
				sterzo.write(angoloSterzo);                  // sets the servo position according to the scaled value 
				
				
				if ( viaLibera < 0 ){
					motorSpeedRef = FERMO;
					direzione 	  = AVANTI;
				}
				else{
					motorSpeedRef = motorSpeedValue;
					direzione 	  = AVANTI;
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
				if ( distanza > odometro )	direzione = AVANTI;
				else						direzione = INDIETRO;
				// gestione velocità
				// se viaggio 
				if (abs(odometro - lastPosition) < 0.3){
					motorSpeedRef = ACCELERAZIONE;
				}
				else
					if ( abs(distanza - odometro) > E_APPROCCIO){
						motorSpeedRef = motorSpeedValue;
					}
					else
						if ( abs(distanza - odometro) > E_POSIZIONAMENTO){
							motorSpeedRef = APPROCCIO;
						}
						else{
							motorSpeedRef = FERMO;
							lastPosition  = odometro;
							statoRun      = 0;
						}
			
				/*	
				Serial.print(angoloSterzo);
				Serial.print(" ,");
				Serial.print(errore);
				Serial.print(" ,");
                                Serial.print(odometro);
				Serial.print(" ,");
				Serial.println(motorSpeedRef);
				*/
			}

			if (statoRun == 0) motorSpeedRef = 0;
			
			
			if (motorSpeedRef > motorSpeed)	motorSpeed++;
			if (motorSpeedRef < motorSpeed)	motorSpeed -= 75;

			if (motorSpeed > 250) motorSpeed = 250;
			if (motorSpeed <   1) motorSpeed = 0;

			if (direzione)  driver.motorAForward(motorSpeed);
			else            driver.motorAReverse(motorSpeed);

            delay (1);
  
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
			
			if (motorSpeedRef > motorSpeed)	motorSpeed++;
			if (motorSpeedRef < motorSpeed)	motorSpeed--;

			if (motorSpeed > 250) motorSpeed = 250;
			if (motorSpeed <   1) motorSpeed = 0;

			if (direzione)  driver.motorAForward(motorSpeed);
			else            driver.motorAReverse(motorSpeed);

			
			
			
			
		}
	}
	
	
	
	
	
    if (mode == TEST_STERZO){

		if (firstRun){
		    Serial.println("start SERVO mode, enter servo position in degrees [0-180]");
			firstRun = 0;
		}
		sterzo.write(angoloSterzo); // sets the servo position according to the scaled value 
    }	
    
	


    if (mode == TEST_SENSORS){
        Serial.println("start TEST_SENSORS mode");
        while(1){
		
			if (measureAvailable){
				misuraSideIR(1);
				measureAvailable = 0;
			}
			delay(1);
			
        }    
    }    

    if (mode == TEST_GIRO_SENSOR){
        Serial.println("start TEST_GIRO_SENSOR mode");
        while(1){
            odometroMisura();
	    delay(10);
        }    
    }    
}

//------------------- fine main --------------------------------




void odometroMisura(void){
static char statoOdometro = 0;
static unsigned long timerOdometro;

    switch (statoOdometro){
      case 0: 
        if (digitalRead(GIRO_PIN)){
          timerOdometro = millis();
          if (direzione == AVANTI)  odometro += GIRO_RUOTA;
          else                      odometro -= GIRO_RUOTA;
          statoOdometro = 1;
          digitalWrite(ledPin, HIGH);   // toggle LED pin
//          Serial.println (odometro);
        }
        break;
    
      case 1:
        if (millis() - timerOdometro > 100){
          if (!digitalRead(GIRO_PIN)){
            statoOdometro = 0;
            digitalWrite(ledPin, LOW);   // toggle LED pin
          }
        }
        break;
    }
    
    
}

/*	riportano 
	-99 (livello zero quando impegnati)
	 99 (livello uno  quando liberi)
	 ci sono dei valori intermedi

	un interrupt chiamato al cambio livello (minimo 50 us) integra il tempo a uno
	e il tempo di misura
	una routine richiamata a tempo conta quante volte è contato a zero e quante a uno
	se il numero di transizioni è elevato si guarda il rapporto Ton/Tmisuta
	else si considera lo stato più volte misurato
	 
*/
float misuraSideIR(int debug){
static float percentoUno;
static unsigned long copia_measuringTime;
static unsigned long copia_totTimeAtOne;
static int copia_frontIRstate 	= 0;
static int copia_sideIRstate 	= 0;
static int copia_sideIRcnt 	= 0;

		noInterrupts();
			copia_sideIRcnt     = sideIRstate;
			copia_totTimeAtOne  = totTimeAtOne;
			copia_frontIRstate  = frontIRstate;
			copia_measuringTime = measuringTime;
			totTimeAtOne        = 0;
			startMeasureIRSide 	= 1;		// riparte integrazione tempi
		interrupts();

		
		// se ho transizioni sufficienti leggo percentuale del tempo
		// -99 vicino	 0.00
		//  99 lontano   1.00
		if ((copia_sideIRcnt < 70) && (copia_sideIRcnt > -70))
			percentoUno = float(copia_totTimeAtOne) / float(copia_measuringTime);
		else{
			if (copia_sideIRcnt >  75)	percentoUno =  1.0;
			if (copia_sideIRcnt < -75)	percentoUno =  0.0;
		}	
			
		if (debug){
//				Serial.print(sideIRstate);
			Serial.print(copia_sideIRcnt);
			Serial.print(" ,");
			Serial.print(percentoUno);
			Serial.print(" ,");
			Serial.println(copia_frontIRstate);
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

void cntSideSensor(void){
static unsigned long time, firstTime, lastTime;

	if (digitalRead(R_SIDE_IR)){
		// quando va a 1 prendo il tempo
		time = micros();
	}
	else{
		// quando va a zero calcolo il tempo che è stato a 1
		lastTime     = micros();
		totTimeAtOne+= lastTime - time;			// integro tempo a uno
		if (startMeasureIRSide){
			// questo è un nuovo ciclo di letture
			// il tempo totale inizia dall'ultima trasizione a zero
			startMeasureIRSide = 0;
			firstTime = lastTime;
		}
		// tempo totale di luttura
		// measuringTime è
		measuringTime = lastTime - firstTime;
	}	

}

/*
	routine richiamata ogni x ms
	ogni 100 lettura setta flag dati disponibili e parte
	nuova serie di misure
*/
ISR(TIMER2_COMPA_vect){          // timer compare interrupt service routine
//void task1ms(void){
static int cntMisura = 0;
static int frontIRcnt= 0;
static long lastExecutionTime;

/*	if ((micros() - lastExecutionTime) < 1000) 	return;
	else 										lastExecutionTime= micros();
*/	
	//Serial.println("interrupt");
	if (cntMisura < 100){
		if (digitalRead(L_SIDE_IR))	frontIRcnt++;
		else						frontIRcnt--;

		if (digitalRead(R_SIDE_IR))	sideIRcnt++;
		else						sideIRcnt--;

	}
	else{
		sideIRstate = sideIRcnt;
		frontIRstate= frontIRcnt;
		cntMisura 	= 0;
		sideIRcnt 	= 0;
		frontIRcnt  = 0;
		measureAvailable = 1;
		//digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin
	}
	cntMisura++;
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
*/

void getCmd(void){
static float x, y;
static char smComandi = 0;
static char inCmd;
static unsigned long time, cmdTime;

	// se il pacchetto non arriva completo in un secondo 
	// resetto ricezione
	if ((smComandi != 0) && ((millis() - cmdTime) > 1000 )) {
		Serial.println('Timeout');
		smComandi = 0;
	}


	if (Serial.available() > 0) {


		switch (smComandi) {
			case 0: // attesa 1rst valore

				inCmd = Serial.read();
				cmdTime = millis();
				
				switch (inCmd) {
		
					// scrivo valore
					case 'S': 
					case 'D': 
					case 'R': 
					case 'V': 
					case 'P': 
					case 'T': 
					case 'L': 
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
							smComandi = 2;		// stato a seconda del numero di
												// parametri da ricevere
												// questa è lettura quindi segue subito risposta
						break;
				}
				break;
			case 1: // attende 1 valore
				// look for the next valid integer in the incoming serial stream:
				x = Serial.parseInt();
				smComandi = 2; 	// mette in esecuzione valore
				break;	
				
			case 2: // attesa terminatore

				if (Serial.read() != '\n') break;
				
				switch (inCmd) {
					case 'S': 
							angoloSterzo = x;
							smComandi = 0;		
							Serial.print("S: ");
							Serial.println(angoloSterzo);
						break;
		
					case 'D': 
							distanza += x;
							smComandi = 0;		
							Serial.print("D: ");
							Serial.println(distanza);
						break;
		
					case 'R': 
							statoRun = x;
							smComandi = 0;		
							Serial.print("R: ");
							Serial.println(statoRun);
						break;

					case 'V': 
							motorSpeedValue = x;
							smComandi = 0;		
							Serial.print("V: ");
							Serial.println(motorSpeedValue);
						break;
		
					case 'P': 
							panAngle = x;
							smComandi = 0;		
							Serial.print("P: ");
							Serial.println(panAngle);
						break;
		
					case 'T': 
							tiltAngle = 180 - x;		// servomotore girato
							smComandi = 0;		
							Serial.print("T: ");
							Serial.println(x);
						break;
		
					case 'L': 
							laser = x;
							smComandi = 0;		
							Serial.print("L: ");
							Serial.println(laser);
							if (laser==0) 	digitalWrite(laserPin, LOW);
							else			digitalWrite(laserPin, HIGH);
						break;
		
					case 'd': 
							smComandi = 0;		
							Serial.print("d: ");
							Serial.println(odometro);
						break;
		
					case 'e': 
							smComandi = 0;		
							Serial.print("e: ");
							Serial.println(errore);
						break;
		
					case 'v': 
							smComandi = 0;		
							Serial.print("v: ");
							Serial.println(motorSpeed);
						break;

					case 'l': 
							smComandi = 0;		
							Serial.print("l: ");
							Serial.println(viaLibera);

					case 'r': 
							smComandi = 0;		
							Serial.print("r: ");
							Serial.println(statoRun);
						break;

					case 'm': 
							smComandi = 0;		
							misura = sonarMisura(5);
							Serial.print("m: ");
							Serial.println(misura);
						break;

					case 'b': 
							smComandi = 0;		
							Serial.print("m: ");
							Serial.println(analogRead(tensionePin));
						break;
				}
				
				break;
			default:
				smComandi = 0;
				break;
		}
		
	}// in serialLine.available
}
