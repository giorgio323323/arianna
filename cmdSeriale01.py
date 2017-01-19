#!/usr/bin/env python
# -*- coding: utf-8 -*-

# device monitoring per testare seriale
import sys, getopt
import serial
import io
import time
import matplotlib.pyplot as plt

debug 		= False
waitReset 	= False
portFree        = False
    
# invio comando e relativo valore
# esito = sendCmd('L', 1)  L1
def sendCmd(cmd, value):
    
    global portFree
    portFree = False

    ser.flushInput()
    msg2send = bytes(str(cmd),"ascii") + bytes(str(value),"ascii") + b'\n'
    ser.write(msg2send)
    sio.flush() # it is buffering. required to get the data out *now*
    if debug:
        print('msg2send: %s' % (msg2send))
    try:
        line = sio.readline()   # read a '\n' terminated line
    except serial.SerialException as e:
        #there is no data from serial line
        portFree = True
        if debug:
            print ("no answer")
        return -1
    except TypeError as e:
        # disconnect usb
        portFree = True
        if debug:
            print ("typeError")
        ser.close()
        return -1
    else:
        portFree = True
        if debug:
            print (line)
        return 0


# invio richiesta di valore 
# esito, misura = sendReq('m')
def sendReq(req):

    global portFree
    
    ser.flushInput()
    msg2send = bytes(str(req),"ascii") + b'\n'
    ser.write(msg2send)
    sio.flush() # it is buffering. required to get the data out *now*

    try:
        line = sio.readline()   # read a '\n' terminated line
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
        misura = float(a.split(':')[1])
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

    waitReset = False
    debug     = True
    if (debug):
        print('start: cmdSeriale01.py')


    # con timeout = 0 diventa non bloccante
    # ritorna subito e esegue quello che arriva quando arriva??
#    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1, parity=serial.PARITY_NONE)
    ser = serial.Serial('COM17', 9600, timeout=1, parity=serial.PARITY_NONE)
    sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

    print ('seriale aperta')
    # attende che il micro si resetti e faccia cose
    time.sleep(0.1)

    if (waitReset):
        time.sleep(3)
        ser.flush()
        # attendo che mi il micro invii la fine del srtup
        # wait for micro start
        # sembra non ripartire non so per il F1 tolto
        # oppure altro
        line =''
        while (not('start' in line)):
            try:
                line = sio.readline()   # read a '\n' terminated line
            except:
                print ("errore: %s" %(str(e)))
            else:
                print ('start received')

    time.sleep(5)
    
    # init
    if(portFree):
        sendCmd('S', 90)
        print('sent S')

    if (portFree):
        sendCmd('L', 1)
        print('sent L')

    # passo 1
    if (portFree):
            sendCmd('D', 1000)
            print('sent D')
    if (portFree):
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
    
''' 
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
