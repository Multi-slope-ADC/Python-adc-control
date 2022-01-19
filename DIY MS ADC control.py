#!/usr/bin/python3

import time
from datetime import datetime, timezone
import serial
from math import *
from threading import Thread
from sshkeyboard import listen_keyboard
from statistics import median

class ADC:
    # this belongs into __init__
    serial_port = 'COM4'	# e.g. /dev/ttyUSB0 on Linux, COM1 on Windows
    m = 10					# output to screen every m results

    # constants depending on ADC board (HW and software)
    # TODO UPPERCASE in python 
    adcclock = 12000000		# clock on ADC board
    scale = 6951.926		# Ref voltage in mV 7106.8384
    xd = 12*3				# extra  delay 3x xdel in ASM code !!! has to match FW !!!
    k1 =  1.0 / 20.9637		# measured ref ratios from adjustment
    k2 =  4.0 / 121.66		# fine step / adc LSB

    def fwrite(string):
        ADC.f.write(string)
    # __init__ until here


    # Lenght of active run-up phase for different RU mode P,Q,R,S,T,U,V,W, has to match AVR software
    k0 = [51+xd-16, 102+2*xd-24, 102+2*xd-16, 102+2*xd-16, 102+2*xd-16, 102+2*xd-24, 102+2*xd-36, 204+4*xd-24]

    k1_puls = 25			# lentgh of long K1 pulse, needed for skale factor measurement, should be 25
						    # may need to be smaller if integrator capacity is small
    sf0 =  (2-k1) * scale  / (adcclock * 0.02)
						    # crude skaling units -> uV  ,depends on HW (clock + ref)
						    # run mode C for some  time to get accurate scale factor

    m_sf = 200				# averaging for scale factor usually 100...500

    #Var
    ser = 0
    setsfstate = -1
    sfvalues = [0 for i in range(m_sf+1)]
    u1old = 0
    k1values = list()
    k2values = list()

    k,c = 0, 0     #k,c : char;
    n,outcnt,ruv = 0, 0, 0  #n,outcnt,ruv : word;
    rawind = 0  #rawind   : word;
    f, fkom = 0, 0  #f, fkom  : text;
    u,u1,u2,u3,u4,du,u2old = 0, 0, 0, 0, 0, 0, 0    #u,u1,u2,u3,u4,du,u2old : double;   { voltages }
    ru = 0  #ru : word;     { runup }
    adc1, adc2, adcalt,adcdiff = 0, 0, 0, 0     #adc1, adc2, adcalt,adcdiff : longint;
    raw = [0 for i in range(60+1)] #raw : array [0..60] of longint;
    sum, sum5,sum25 = 0, 0, 0   #sum, sum5,sum25 : longint;
    n5,n25 = 0, 0   #n5,n25 : word;
    sumA, sumB = 0, 0   #sumA, sumB : longint;
    sumdu,avdu,sumq,rms,sf = 0, 0 ,0 ,0, 0  #sumdu,avdu,sumq,rms,sf :  double;
    sumu1,sumu2,sumu3,u1m,u2m,u3m = 0, 0, 0, 0, 0, 0    #sumu1,sumu2,sumu3,u1m,u2m,u3m : double;
    sumk1,sumk2, sumsf = 0, 0, 0    #sumk1,sumk2, sumsf : double;
    countk,countsf = 0, 0   #countk,countsf : integer;

    ruv = 1                   # default RU version
    sf = sf0                  # crude estimate for scale factor as a start


    def checkscreen():          # sum up and check if ready for output
    #var avduold :double;
        #global outcnt, sumdu, sumq, sumu1, sumu2, sumu3, avdu, rms, u1m, u2m, u3m
        ADC.outcnt = ADC.outcnt + 1;
        ADC.sumdu = ADC.sumdu+ADC.du;
        ADC.sumq = ADC.sumq + (ADC.du-ADC.avdu) ** 2;
        ADC.sumu1 = ADC.sumu1 + ADC.u1;
        ADC.sumu2 = ADC.sumu2 + ADC.u2;
        ADC.sumu3 = ADC.sumu3 + ADC.u3;
        if ADC.outcnt == ADC.m:
            ADC.outcnt = 0
            checkscreen = True
            avduold = ADC.avdu
            ADC.avdu = ADC.sumdu/ADC.m
            ADC.rms = sqrt((ADC.sumq - (ADC.avdu-avduold) ** 2) / (ADC.m-1))
            ADC.u1m = ADC.sumu1/ADC.m
            ADC.u2m = ADC.sumu2/ADC.m
            ADC.u3m = ADC.sumu3/ADC.m
            ADC.sumu1 = 0
            ADC.sumu2 = 0
            ADC.sumu3 = 0
            ADC.sumdu = 0
            ADC.sumq = 0
        else:
            checkscreen = False
        return checkscreen


    def writeStatus():
        print('Device: ' + ADC.ser.name + ' Status: ' + str(ADC.ser.is_open) + '\n')

    def readbytes(n):                # read value and remember
        #global raw, rawind
        bytesread = bytearray(n)
        count = ADC.ser.readinto(bytesread)
        if count != n: print('ERROR: Wanted to read {} bytes, but read {} bytes instead'.format(n, count))  #TODO raise exception
        k = int.from_bytes(bytesread, byteorder='little', signed=False)
        ADC.raw[ADC.rawind] = k         # remember raw values as words
        ADC.rawind += 1
        return k

    def readADC(k_0):           # read one ADC result
                                # k_0 is lenght of runup steps in cycle
                                # result is in cycles of small (negative) reference
    #var  small, large, ru0 :longint
        #global ru, adc1, adc2, adcdiff, adcalt
        ADC.ru = ADC.readbytes(2)             # run-up counts
        small = ADC.readbytes(2)          # cycles of weaker ref (negative)
        large = ADC.readbytes(2)          # cycles of stronger ref (pos)
        ADC.adc1 = ADC.readbytes(2)           # aux ADC , average integrator voltage, for info only
        ADC.adc1 = ADC.readbytes(2)           # residual charge after conversion
        ADC.adc2 = ADC.readbytes(2)           # residual charge before conversion
        ADC.adcdiff = ADC.adc2-ADC.adc1
        ru0 = round(ADC.adcclock *0.02 / (k_0+16) / 2)  # approximate zero : length K_o+16 is not accurate
        ADC.adcalt= ADC.adc2            # remember old res. charge (for version with only 1 reading)
        return (k_0 *(ADC.ru-ru0)) *(2+ADC.k1) + (small - large*(1+ADC.k1) + (ADC.adcdiff)*ADC.k1*ADC.k2)

    def writetime(f):           # write date and time to file + info on konstants
        time = datetime.now(timezone.utc).astimezone(tz=None).replace(tzinfo=None)
        f.write(str(datetime.now().astimezone().replace(microsecond=0)))
        f.write(f' with k0={ADC.k0[1]} k1={1/ADC.k1:10.4f} k2={4/ADC.k2:8.2f} scale={ADC.scale}\n'.format())

    def writeLogK():
    #VAR  fkom : text;
        #global sumk1, sumk2, countK
        with open('log_kom.txt', 'a') as fkom:
            fkom.write('# K1,K2,SF= {:14.6f} {:10.2f} {:15.7f}  '.format(ADC.sumk1, ADC.sumk2, ADC.sf*1000))
            ADC.writetime(fkom)
            ADC.sumk1 = 0
            ADC.sumk2 = 0
            ADC.countK = 0

    def writeraw():             # write raw data to file (end of line)
    # Var  i   :  word;
        #global rawind
        for i in range (0, ADC.rawind, 1):   # i  := 0 to rawind-1 do
            ADC.fwrite('\t{}'.format(ADC.raw[i]))
        ADC.fwrite('\n')
        ADC.rawind = 0

    def skalefactor1():         # result from slow slope measurement
    #var  pulseL, adcl : integer;
        #global sum, sum5, n5, sum25, n25
        ADC.sum = ADC.readbytes(3)          # sum of slow slope lenght
        pulseL = ADC.readbytes(1)        # pulse length
        if pulseL == 5:         # sum up data for short
           ADC.sum5 = ADC.sum5 + ADC.sum;
           ADC.n5 = ADC.n5 + 1;
        if pulseL == ADC.k1_puls:   # long case
            ADC.sum25 = ADC.sum25 + ADC.sum;
            ADC.n25 = ADC.n25 + 1;
        adcl = ADC.readbytes(2)           # dummy read not yet used, not needed
        adcl = ADC.readbytes(2)

    def skalefactor2():         # result of ADC scale factor measurement
    #var m1,m2  : integer;
        #global adc1, adc2, sum5, sum25, n5, n25, sumk1, sumk2, countk, sumsf, countsf, u, u2, u3, sf, k1, k2, k1values, k2values
        m1 = ADC.readbytes(1)            # number of steps up
        m2 = ADC.readbytes(1)            # number of steps down
        ADC.sumA = ADC.readbytes(2)
        ADC.sumB = ADC.readbytes(2)
        ADC.adc1 = ADC.readbytes(2)         # for debug only, but need to read !
        ADC.adc2 = ADC.readbytes(2)
        ADC.writeraw()
        if ADC.n5 > 2:              # enough data from adjustK1 ( slow slope)
            ADC.u = (ADC.sum25 / ADC.n25 - ADC.sum5 / ADC.n5)   # difference of average
            ADC.u3 = ADC.u / (3*(ADC.k1_puls-5)*128)    # calculate slope ratio, include fixed constants
            ADC.fwrite('k1=\t{:14.6f}'.format(ADC.u3))
            print('## k1 = {:14.6f}'.format(ADC.u3), end = '')
            ADC.sum5 = 0            # reset sums after use
            ADC.sum25 = 0
            ADC.n5 = 0
            ADC.n25 = 0
        if (m1 > 2) and (m2 > 2):           # useful result from ADC scale
            ADC.u = ADC.sumA
            ADC.u = ADC.u/m1
            ADC.u2 = 65536-ADC.sumB
            ADC.u2 = ADC.u2/m2          # U2 is negative
            ADC.sumk1 = ADC.sumk1 + ADC.u3
            ADC.sumk2 = ADC.sumk2 + (ADC.u + ADC.u2) / 2
            ADC.countk = ADC.countk + 1
            
            ADC.k1values.append(ADC.u3)
            ADC.k2values.append((ADC.u + ADC.u2) / 2)
            if len(ADC.k1values) >= ADC.m_sf:
                print('\nOld k1 & k2: {:.6f} {:.3f}'.format(1.0 / ADC.k1, 4.0 / ADC.k2))
                ADC.k1 =  1.0 / median(ADC.k1values)
                ADC.k2 =  4.0 / median(ADC.k2values)

                print('New k1 & k2: {:.6f} {:.3f}'.format(1.0 / ADC.k1, 4.0 / ADC.k2))
                ADC.k1values = list()
                ADC.k2values = list()

            if ADC.countsf == ADC.m_sf:
                print(' update scalefactor from ref reading old: {:13.10f}'.format(ADC.sf), end = '')
                ADC.sf = ADC.sumsf / ADC.m_sf
                print(' new: {:13.10f}'.format(ADC.sf), end = '')
                ADC.sumsf = 0       # reset sum to allow update
                ADC.countsf = 0
            if ADC.countk > 5:  ADC.writeLogK()
            ADC.fwrite('\tk2=\t{:13.5f}\t{:11.3f}\t{:11.3f}\tSF=\t{:13.5f}\n'.format((ADC.u + ADC.u2) / 2, ADC.u, ADC.u2, ADC.sf*1000))
            print(' ## k2 : {:11.3f}{:11.3f}{:3.0f}{:3.0f}{:11.3f}'.format(ADC.u, ADC.u2, m1, m2, (ADC.u + ADC.u2) / 2))
        else:
            print(' Problem with ADC data: {:10.0f}{:10.0f}{:10.0f}{:10.0f}\n'.format(m1, m2, ADC.sumA, ADC.sumB))    # invalid data

    def setscalefactor(n):                  # acal for modes with 2 readings, updates sf
        #global setsfstate, sfvalues, sf
        if n == 254:            # n = action - setsf only for Mode A & E
            # get m_sf readings of 7V & GNDS
            if ADC.setsfstate == 0:
                ADC.ser.write('6'.encode("utf-8"))
            if ADC.setsfstate >= 1 and ADC.setsfstate <= ADC.m_sf + 1:
                ADC.sfvalues[ADC.setsfstate-1] = ADC.u1 - ADC.u2

            ADC.setsfstate += 1

            if ADC.setsfstate > ADC.m_sf + 1:                     # setsf finished
                ADC.setsfstate = -1
                print('update scalefactor from ref reading old: {:13.10f}'.format(ADC.sf), end = '')
                ADC.sf = ADC.scale/median(ADC.sfvalues)
                print(' new: {:13.10f}'.format(ADC.sf))

        else:
            ADC.setsfstate = -1
            print('Error: update scalefactor only awailable in mode A & E!')

    def read2 (n):              # 254, 251: 2 readings (modes A, B, E)
        #global u1, u2, u1old, u2old, f, du
        ADC.u1 = ADC.readADC(ADC.k0[ADC.ruv])       # result of 1. conversion
        if n == 254:
            ADC.u2 = ADC.readADC(ADC.k0[ADC.ruv])   # result of 2. conversion
            ADC.du = ADC.u1 - ADC.u2
        else:
            ADC.u2 = ADC.readADC(ADC.k0[1])     # result of 2. conversion, mode B for INL test
            ADC.du = (3*(ADC.u2old-ADC.u1)-ADC.u1old+ADC.u2)/4   # interpolate both values

        ADC.fwrite('{:11.3f}\t{:11.3f}\t{:13.4f}\t{:6.0f}'.format(ADC.u1, ADC.u2, ADC.du*ADC.sf, ADC.adcdiff))
        ADC.writeraw()
        if ADC.checkscreen():
            print('{} {:11.3f} {:11.3f} {:11.3f} {:13.4f} {:6.0f}{:13.5f}'.format( ADC.ru, ADC.u1m, ADC.u2m, ADC.du, ADC.sf*ADC.avdu, ADC.adc1, ADC.rms*ADC.sf))
        ADC.u1old = ADC.u1
        ADC.u2old = ADC.u2
        return [ADC.u1, ADC.u2, ADC.du, ADC.sf, ADC.adcdiff]

    def read2AE():
        return ADC.read2(254)

    def read2B():
        return ADC.read2(251)

    def read3():                # 250: 3 readings (mode C)
        #global u1, u2, u3, u2old, f, du, countsf, sumsf
        ADC.u1 = ADC.readADC(ADC.k0[ADC.ruv])       # result of 1. conversion
        ADC.u2 = ADC.readADC(ADC.k0[ADC.ruv])       # result of 2. conversion
        ADC.u3 = ADC.readADC(ADC.k0[ADC.ruv])       # result of 3. conversion
        if abs(ADC.u3-ADC.u2) > 1000 :
            ADC.du = (ADC.u1-ADC.u2)/(ADC.u3-ADC.u2)
            if ADC.countsf < ADC.m_sf:
                ADC.sumsf = ADC.sumsf + ADC.scale/(ADC.u3-ADC.u2)
                ADC.countsf = ADC.countsf + 1   # sum up the first m_sf scale factor readings
        else:
            ADC.du = (ADC.u1-ADC.u2) *ADC.sf/ 1000  # approx scale if no valid 7 V
        ADC.fwrite('{:11.3f}\t{:11.3f}\t{:11.3f}\t{:13.4f}\t{:6.0f}'.format(ADC.u1, ADC.u2, ADC.u3, ADC.du*ADC.scale, ADC.adc1-ADC.adc2))
        ADC.writeraw()
        if ADC.checkscreen():
            print('{} {:11.3f} {:11.3f} {:11.3f} {:13.4f}{:6.0f} {:13.5f}'.format(ADC.ru, ADC.u1m, ADC.u2m, ADC.u3m, ADC.avdu*ADC.scale, ADC.adc1, ADC.rms*ADC.scale))
        ADC.u2old = ADC.u2
        return [ADC.u1, ADC.u2, ADC.u3, ADC.du, ADC.scale, ADC.adc1-ADC.adc2]

    def read4():                # 248, 247: 4 readings (mode D)
        #global u1, u2, u3, u4, u2old, f, du
        ADC.u1 = ADC.readADC(ADC.k0[ADC.ruv])      # result of 1. conversion
        ADC.u2 = ADC.readADC(ADC.k0[ADC.ruv])      # result of 2. conversion
        ADC.u3 = ADC.readADC(ADC.k0[ADC.ruv])      # result of 3. conversion
        ADC.u4 = ADC.readADC(ADC.k0[ADC.ruv])      # result of 4. conversion
        if (abs(ADC.u4-ADC.u3) > 1000): ADC.du = (ADC.u2-ADC.u3)/(ADC.u4-ADC.u3)
        else: ADC.du = (ADC.u2-ADC.u3) * ADC.sf/1000.0  # approx scale if no valid 7 V
        ADC.fwrite('{:11.3f}\t{:11.3f}\t{:11.3f}\t{:11.3f}\t{:13.4f}\t{:6.0f}'.format(ADC.u1, ADC.u2, ADC.u3, ADC.u4, ADC.du*ADC.scale, ADC.adc1-ADC.adc2))
        ADC.writeraw()
        if ADC.checkscreen():
            print('{} {:11.3f} {:11.3f} {:11.3f} {:11.3f} {:13.4f}{:6.0f} {:13.5f}'.format(ADC.ru, ADC.u1m, ADC.u2m, ADC.u3m, ADC.u4, ADC.avdu*ADC.scale, ADC.adc1, ADC.rms*ADC.scale))
        ADC.u2old = ADC.u2

    def readda():               # TODO 241: DA-Test (mode G)
    # Charge (40.5 µs), wait (uart send nx17 cycles), run-down (uart send 2x2bytes), read ADC wait, read ADC x7, wait ~520µs, read ADC, wait ~780µs, read ADC, wait ~26 ms, read ADC, run-down (uart send 2x2bytes), read ADC wait, wait ~13ms, read ADC, wait ~13ms, read ADC
        #global f
        da = [0 for i in range(34)]
        print('DA result: ', end = '')
        das = '0x'
        for n in range (0, 34, 1):
            #da[n] = ord(ser.read(1))
            da[n] = ADC.readbytes(2)
            print('{:4d} '.format(da[n]), end = '')
            das = das + hex(da[n])
        print()
        ADC.fwrite(das)
        ADC.writeraw()

    def keypress(key):
        #global k, ser, ruv
        ADC.k = (key).upper()
        if len(ADC.k) == 1 and (((ADC.k >= '0') and (ADC.k <= '7')) or ((ADC.k >= 'A') and (ADC.k <= 'W'))):
            ADC.ser.write(ADC.k.encode("utf-8"))
            print('Sent: ' + ADC.k)
            if (ADC.k >= 'P') and (ADC.k <= 'W'): ADC.ruv = ord(ADC.k)-80      # update runup version
        
    def read():
        #global n
        rawind = 0          # new package, reset counter for raw data
        while ord(ADC.ser.read(1)) != 255: pass     # wait for sync
        ADC.n = ord(ADC.ser.read(1))       # get tag
        return ADC.action.get(ADC.n, lambda: 'Invalid')()         # action depending on tag

    action = {              # action depending on tag
        254: read2AE,       # 2 readings (modes A, E)
        251: read2B,        # 2 readings (mode B)
        250: read3,         # 3 readings (mode C)
        253: skalefactor1,  # slope ratio measurement (mode ?, included in mode L)
        252: skalefactor2,  # ADC skale factor and ouput of slope ratio (mode L)
        248: read4,         # 4 readings (mode D?)
        247: read4,         # 4 readings (mode D?)
        241: readda         # DA-Test 26 chars (mode G)
    }


def main():                     # main program
    #VAR     fn,kom : string;
    #		fkom_ : Longint;
    #global k, ser, f, fkom, setsfstate

    #try:

    print('Opening file...')
    
    fn = input('Filename ')
    if fn == '': fn = 'test.txt'
    kom = input('Kommentar ')
        
    with open(fn, 'w') as ADC.f:
        ADC.fwrite('# ')
        ADC.writetime(ADC.f)
        ADC.fwrite('# {}\n'.format(fn))
        ADC.fwrite('# {}\n'.format(kom))
        
        if fn != 't':
            with open('log_kom.txt', 'a') as ADC.fkom:  # extra log file
                ADC.fkom.write('{} {}\n'.format(fn, kom))
                ADC.writetime(ADC.fkom)
        print('Opening COM-Port...')

        ADC.ser = serial.Serial(port=ADC.serial_port,
                                baudrate=9600,
                                bytesize=serial.EIGHTBITS,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                #xonxoff=True,
                                # rtscts=True,
                                # dsrdtr=True,
                                #exclusive=False,
                                timeout=120)

        time.sleep(10E-3)
        ADC.writeStatus()
        
        print('COM-Port opened!\n')
        
        ADC.keypress('C')           # call for mode C = AZ mode with ref

        # handle keypresses
        on_press=ADC.keypress
        t = Thread(target=listen_keyboard, args=(on_press,))
        t.daemon = True
        t.start()
        
        while (ADC.k != 'X'):       # loop to receive & send data
            ADC.read()
            if ADC.k == 'Z':        # set scalefactor for 2 reading mode
                ADC.setsfstate = 0
                ADC.k = ''
                print('starting update scalefactor...')
            if ADC.setsfstate >= 0: ADC.setscalefactor(ADC.n)

    #except Exception as e:
    #    print('Error: ' + str(e))
        
if __name__ == "__main__":
	main()