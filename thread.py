#!/usr/bin/python3

# device monitoring per testare seriale

#cd D:\Program Files (x86)\python351
#cd C:\Users\giorgio.rancilio\Dropbox\MakersPgo\progettoAlfredo\gestioneArianna




#        line = sio.readline()   con questa va in timeout
#        line = ser.readline()    con questa va


import sys, getopt
import serial
import io
import time

import math
import sys
import threading
import msvcrt
import matplotlib.pyplot as plt
import numpy as np


exitFlag = 0
ser = 0
debug = True





'''
        0       #
	1	Serial1.print(dDxCnt);
	2	Serial1.print(dSxCnt);
	3	Serial1.print(deltaC);
	4	Serial1.print(teta);
	5	Serial1.print(xpos);
	6       Serial1.print(ypos);
	7	Serial1.print(errore);
'''
def plotData():

   # prima scrivo il file poi apro figura
   # la funzione non procede con la figura aperta
   file = open("C:testfile.txt","w") 

   headerLine = "indice; dDxCnt; dSxCnt; deltaC; teta; xpos; ypos; errore \n"
   file.write(headerLine) 
   indice = 0;
   for line in dataArray[1]:
      linea = str(indice)+"; "
      for j in range(1, 8):
         linea += str(dataArray[j][indice])+"; "
      linea += "\n"
      file.write(linea)
      indice +=1
 
    
   file.close() 

   #print (dataArray[6])
   #print (dataArray[7])
   plt.figure(1)
   plt.subplot(211)
   plt.plot(dataArray[5], dataArray[6], 'ro--')
   plt.grid(True)
   plt.xlabel('x')
   plt.ylabel('y')
   plt.subplot(212)
   plt.plot(dataArray[5], dataArray[4], 'ro--')
   plt.grid(True)
   plt.xlabel('x')
   plt.ylabel('teta')
   
   #plt.axis([-100, 15000, -2000, 2000])
   plt.show()
   #dataArrayReset()


   




#
# txThread
# attende dati da testiera e .....
#

class txThread (threading.Thread):
   # costruisce una instanza di myThreadcon il suo spazio e le sue variabili
   def __init__(self, threadID, name, counter):
      threading.Thread.__init__(self)
      
      self.threadID = threadID
      self.name     = name
      self.counter  = counter
      
      print("costuttore di: %s" % (name))

   def run(self):
      print ("Starting " + self.name)
      print ()
      #funzione del corpo
      keyboardMonitor(self.name, self.counter)
      print ("Exiting " + self.name)


#attende comandi e li invia alla seriale
def keyboardMonitor(threadName, counter):
   global ser, debug
   global runState

   num = 0
   done = False
 

   stringaIn = ''
   while runState:
      if exitFlag:
         threadName.exit()

      #print (num)
      num += 1
         
      if msvcrt.kbhit():
         char = msvcrt.getch()
         #char = char.decode("ascii")
         if (char == b'\r'):
            # stringa arrivata
            print (stringaIn )
            print ("enter quit to end program")
            stringaArrivata = stringaIn
            stringaIn = ''
            if (stringaArrivata == "quit"):
               print("go to quit")
               runState = False
            else:
               msg2send = bytes(stringaArrivata, "ascii") + b'\n'
               ser.write(msg2send)
               sio.flush() # it is buffering. required to get the data out *now*

         else:
            stringaIn += char.decode("ascii")
            print (char )


         
         #print ("you pressed" + charIn.decode("ascii")  + "so now i will quit")
         #done = True


#-----------------------------------------------------------
def readSerial(ser_):
   global runState
   
   eol=b'\n'
   leneol = len(eol)
   
   line = bytearray()
   
   while runState:
      if ser_.inWaiting():
         c = ser_.read(ser_.inWaiting())
         #print(c)
         if (eol in c):
            c = c.replace(eol, b"?")
            ll = c.split(b"?")
            line += ll[0]
            #print(line)
            ser_.flushInput()
            return (True, bytes(line))
         else:
            line += c
   

   return (False, bytes(line))   #uscita per runState false



#
# rxThread
#
#
class rxThread (threading.Thread):
   # costruisce Una instanza di myThreadcon il suo spazio e le sue variabili
   def __init__(self, threadID, name, ser_):
      threading.Thread.__init__(self)
      
      self.threadID = threadID
      self.name     = name
      self.ser_  = ser_


      print("costuttore di: %s" % (name))

   def run(self):
      print ("Starting " + self.name)
      print ()
      self.ser_.flushInput()
      rxData(self.ser_)

      print ("Exiting " + self.name)


def rxData(ser_):
   global debug
   global runState, running
   global newPoint, dataArray

   print("attendo dati")
   stringaIn = ''
   x = 0
   oldLine = 0
   monitor = False
   firstRun = True
   counter = 0
   
   while runState:

#      if ser_.inWaiting():  # Or: while ser.inWaiting():#
#         line = ser_.readline()#.strip()
      arrivato, line = readSerial(ser_)
      if arrivato:
         
         line = line.decode("ascii")
         #print (line)

         if (line == "quit"):
            print("go to quit")
            runState = False
         #print(line[0])

         ok = len(line) 
         if (ok > 0):
            monitor = (line[0] == "#")
         if (monitor):

            if (firstRun):
               oldLine = line
               firstRun = False
               
            if (line != oldLine):
               oldLine = line
               counter = 0
               running = True
               
               line1 = line.replace(" ", "")  # rimuove spazi
               data  = line1.split(',')  # separa con "," come separatore

               # ricevitore non conosce contenuto dati. riempe array per plot
               i=0
               for item in data:
                  if (i>0):
                     dataArray[i].append(float(item))
                  else:
                     dataArray[i].append(item)  # tutti numeri tranne il #
                  i +=1
               print('# in: %s' % (len(dataArray[0])))
            else:
               counter += 1
               if counter == 10:
                  running = False

         else:
            print("answ: %s" % (line))

#------------------------------------------------
def dataArrayReset():
   global dataArray
   dataArray= [[],[],[],[],[],[],[],[],[],[],[],[],[],[]]
   print("array cleared")

if __name__ == "__main__":

#   global debug
   waitReset = True
   debug     = True
   runState = True
   newPoint = [False, 0, 0, 0, 0,0,0,0,0,0,0,0,0]
   
   dataArrayReset()

   # con timeout = 0 diventa non bloccante
   # ritorna subito e esegue quello che arriva quando arriva??
#    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1, parity=serial.PARITY_NONE)
   # BT a 38400
   ser = serial.Serial('COM12', 38400, timeout=5, parity=serial.PARITY_NONE)
#   ser = serial.Serial('COM11', 9600, timeout=5, parity=serial.PARITY_NONE)
   sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser), newline = None)

   print ('seriale aperta')
   # attende che il micro si resetti e faccia cose
   time.sleep(3)

   #ser.flushInput()
   #msg2send = bytes(str("ciao mi sono collegato, fino a qui tutto bene"),"ascii") + b'\n'
   #ser.write(msg2send)
   #sio.flush() # it is buffering. required to get the data out *now*



   #plt.ion()

   #fig, ax = plt.subplots()

   #plot = ax.scatter([], [])
   #ax.set_xlim(-500, 2000)
   #ax.set_ylim(-500, 2000)
   
   # start
   print("start: threading .....")
   print()
   # Create new threads
   thread1 = txThread(1, "Thread-tx", 1)
   thread2 = rxThread(2, "Thread-rx", ser)

   # Start new Threads
   thread1.start()
   thread2.start()
   runningOld = running = False

   while runState:
      if  ((runningOld == True) and (running == False)):
         plotData()
         #print('dato aggiunto')
         time.sleep(0.1) # tempo libero per gestire il resto
      runningOld = running

   thread1.join()
   thread2.join()
   ser.close()
   print ("Exiting Main Thread")
