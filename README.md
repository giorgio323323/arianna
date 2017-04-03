# arianna
arianna: il robot biblitecario


via seriale è possibile:
-eseguire delle funzioni base
-settare dei parametri
-leggere lo stato di variabili

elenco comandi (primitive disponibili):
verifica se arrivano caratteri da seriale
considero un pacchetto con la forma
a scopo mnemonico i comandi di scrittura hanno caratteri maiuscoli, le letture minuscoli.
  
  scrittura invio: ch, valore, terminatore(\n)  risposta: chInviato, ':', valore inviato, , terminatore(\n)
  lettura   ch, terminatore(\n)                 risposta: chInviato, ':', valore inviato, , terminatore(\n)
  
	S Scrivo S (scorrimento tar ruote)
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
	Z MAX_S
	a xpos
	b ypos
	c teta
	m misura con sonar [cm]
	t legge tensione batteria
	w
	z
	s valore sterzo (lettura)
	C raggio di curva
	K kp guadagno proporzionale

	1 scrivi valore debug
	2 leggi  valore debug
	x BTstate
	M 1 attivo monitor variabili, 0 fermo monitor
