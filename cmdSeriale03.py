#!/usr/bin/env python
# -*- coding: utf-8 -*-

# device monitoring per testare seriale
'''
#        line = sio.readline()   con questa va in timeout
        line = ser.readline()    con questa va
'''

import sys, getopt
import serial
import io
import time
import matplotlib.pyplot as plt
import math
import sys

debug 		= True
waitReset 	= False
    
# invio comando e relativo valore
# esito = sendCmd('L', 1)  L1
def sendCmd(cmd, value):
    
    ser.flushInput()
    msg2send = bytes(str(cmd),"ascii") + bytes(str(value),"ascii") + b'\n'
    ser.write(msg2send)
    sio.flush() # it is buffering. required to get the data out *now*
    if debug:
        print('msg2send: %s' % (msg2send))
    try:
#        line = sio.readline()   # read a '\n' terminated line
        line = ser.readline()

    except serial.SerialException as e:
        if debug:
            print ("no answer")
        return -1
    except TypeError as e:
        if debug:
            print ("typeError")
        ser.close()
        return -1
    else:
        if debug:
            print ('send return: %s' %(line))
        return 0


# invio richiesta di valore 
# esito, misura = sendReq('m')
def sendReq(req):
    
    ser.flushInput()
    msg2send = bytes(str(req),"ascii") + b'\n'
    ser.write(msg2send)
    sio.flush() # it is buffering. required to get the data out *now*

    try:
        #line = sio.readline()   # read a '\n' terminated line
        line = ser.readline()

    except serial.SerialException as e:
        #there is no data from serial line
        if debug:
            print ("no answer")
        return -1, 0
    except TypeError as e:
        # disconnect usb
        if debug:
            print ("typeError")
        ser.close()
        return -1, 0
    else:
        if debug:
            print (line)
        a=line[0:len(line)-1]
#        misura = float(a.split(':')[1])
        try:
            misura = float(a.split()[1])
        except:
            return -1, 0
        else:
            return 0, misura


# aspetto il termine del comando di run

#	
def waitEndRun():

	running = 1
	oldD    = 0
	
	while (running):
		e, v = sendReq('r')
	
		# verifico se runCmd finito 
		if (e == 0):
			running = v
		else:	
			# se non terminato controllo che si muova
			# odometro cambia
			e, v = sendReq('d')
			if (e == 0):
				if (d == oldD):
					timeOutCnt += 1
				else:
					timeOutCnt = 0
					oldD = d
				
			if (timeOutCnt > 4):
				sendCmd ('R', 0)	# fermo
				endFor = -2
				return endFor
				
		if (running == 0):
			endFor = 0
			return endFor

		time.sleep(0.1)
	
	
	
	
def main(argv):

	debug = False

	try:
		opts, args = getopt.getopt(argv,"hdnw",["file","console"])
	except getopt.GetoptError:
		print("modo help    cmdSeriale.py --h")
		print("modo debug   cmdSeriale.py --d")
		print("modo nodebug cmdSeriale.py --n")
		print("wait serial port init      --w")
		sys.exit(2)

	for opt, arg in opts:
		if opt == '-h':
			print("modo help    cmdSeriale.py --h")
			print("modo debug   cmdSeriale.py --d")
			print("modo nodebug cmdSeriale.py --n")
			print("wait serial port init      --w")
			sys.exit()
		elif opt in ("-d", "--file"):
			debug = True
			print (" debug mode On")
		elif opt in ("-w"):
			waitReset = True
			
#----------------------------------------------------------

if __name__ == "__main__":
    main(sys.argv[1:])

    print('start: cmdSeriale03.py')

    distanza = 0;

    waitReset = True
    debug     = True


    # con timeout = 0 diventa non bloccante
    # ritorna subito e esegue quello che arriva quando arriva??
#    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1, parity=serial.PARITY_NONE)
    ser = serial.Serial('COM12', 38400, timeout=5, parity=serial.PARITY_NONE)
#    ser = serial.Serial('COM11', 9600, timeout=5, parity=serial.PARITY_NONE)
    sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser), newline = None)

    print ('seriale aperta')
    # attende che il micro si resetti e faccia cose
    time.sleep(0.1)

    if (waitReset):
        time.sleep(0.1)
        ser.flush()
        # attendo che mi il micro invii la fine del srtup
        # wait for micro start
        # sembra non ripartire non so per il F1 tolto
        # oppure altro
        '''
        linea = b"b"
        while (not(b"controlloAttivo" in linea)):
            try:
                linea = ser.readline()   # read a '\n' terminated line
            except:
                print ("errore: %s" %(str(e)))
            else:
                print (linea)
        '''
        e, v = sendReq('x')
        while ( v == 0):
            print ('wait BT: %s' % (v))
            e, v = sendReq('x')
        
    time.sleep(0.1)
    
    # init
#        sendCmd('S', 90)
#        print('sent S')

    sendCmd('P', 90)
    sendCmd('T', 90)

    sendCmd('L', 1)
    print('sent L')
    time.sleep(0.2)
    sendCmd('L', 0)
    print('sent L')
    time.sleep(0.2)
    sendCmd('K', 12)     # guadagno in modo sensore
    sendCmd('B', 2)     # guadagno in modo odometro


    distanza = 4000
    e, dTarget = sendReq('d')
    dTarget += distanza

    # passo 1
    sendCmd('D', distanza)
    print('sent D')
    sendCmd('R', 3)
    print('sent R')

    waitEndRun()
    
    sendCmd('A', -90)   # 1 ruota ferma
    print('sent A')

   
    sendCmd('D', 1500)
    print('sent R')
    sendCmd('R', 4)
    print('sent R')

    waitEndRun()
    

    ''' misura distanza dal muro e calcola angolo del T

    altezzaLaser = 15 cm
    dMuro = misura in cm

    H altezza da puntare in cm
    

    '''
    sendCmd('P', 90)
    e = 1
    while( e ):
        e, dMuro = sendReq('m')
    H = 80
    altezzaLaser = 15
    angoloTilt = math.atan((H - altezzaLaser)/dMuro)
    angoloTilt = math.floor(math.degrees(angoloTilt) + 90)
    

    sendCmd('T', angoloTilt)

    sendCmd('L', 1)
    print('sent L')
    time.sleep(5)
    sendCmd('L', 0)
    print('sent L')
    time.sleep(0.2)

    sendCmd('P', 100)
    time.sleep(0.5)
    sendCmd('P', 80)
    time.sleep(0.5)
    sendCmd('P', 100)
    time.sleep(0.5)
    sendCmd('P', 80)
    time.sleep(0.5)

    # riotrno -------------------------

    sendCmd('D', 1300)
    sendCmd('A', 90)
    print('sent R')
    sendCmd('R', 4)
    print('sent R')

    waitEndRun()

    sendCmd('D', 4000)
    sendCmd('A', 170)
    print('sent R')
    sendCmd('R', 11)
    print('sent R')

    waitEndRun()

    ser.close()             # close port

    
'''

    # passo 1
    sendCmd('D', 1000)
    print('sent D')
    sendCmd('R', 1)
    print('sent R')

    waitEndRun()

    # arretra sterzando dx
    sendCmd('D', -700)
    sendCmd('S', 180)
    sendCmd('R', 2)
    waitEndRun()

    # avanva sterzando sx
    sendCmd('D', 700)
    sendCmd('S', 0)
    sendCmd('R', 2)
    waitEndRun()


    sendCmd('S', 90)
    sendCmd('L', 0)

    e, v = sendReq('d')
    print('posizione: %s, %s'% (v, e))
   
    ser.close()             # close port
    


    sendCmd('P', 100)
    sendCmd('T', 100)

    e, m = sendReq('m')
    if (e==0):
        print('e: %s, misura: %s' % (e, m))
    else:
        print('misura fallita')
        
    time.sleep(1)
    sendCmd('P', 80)
    sendCmd('T', 80)
    sendCmd('L', 0)


    min = 20
    max = 160

    misure = []
    angoli = []

    for a in range (80, 110, 5):
        angolo = a
        sendCmd('P', angolo)
        e, misura = sendReq('m')
        misure.append(misura)
        angoli.append(angolo)
        print ('Misura ad angolo %s: %s' %(angolo, misura))

    sendCmd('P', 90)
    ser.close()             # close port

    plt.plot(angoli, misure)
    plt.ylabel('some numbers')
    plt.show()

'''
