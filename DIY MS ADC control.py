#!/usr/bin/python3

import time
from datetime import datetime, timedelta, timezone
import serial
from decimal import *
from math import *
from threading import Thread
from sshkeyboard import listen_keyboard

serial_port = 'COM4'	# e.g. /dev/ttyUSB0 on Linux, COM1 on Windows
m = 10					# output to screen every m results
out_mode = 3			# TODO output format (not yet implemented)
						# 0 -> only average of result
						# 1 -> only average
						# 2 -> u1,u2,...
						# 3 -> raw

# constants depending on ADC board (HW and software)
# TODO UPPERCASE in python 
adcclock = 12000000		# clock on ADC board
scale = 6951.926		# Ref voltage in mV 7106.8384
xd = 12*3				# extra  delay 3x xdel in ASM code !!! has to match FW !!!
k1 =  1.0 / 20.962		# measured ref ratios from adjustment
k2 =  4.0 / 121.5		# fine step / adc LSB

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
#outcnt = 0
#sumdu =0
#sumq =0
#avdu =0
#u2old=0
#rawind = 0
#sumk1 = 0
#sumk2 = 0
#sumsf = 0
#countk = 0
#countsf = 0
sf = sf0                  # crude estimate for scale factor as a start


def checkscreen():          # sum up and check if ready for output
#var avduold :double;
    global outcnt, sumdu, sumq, sumu1, sumu2, sumu3, avdu, rms, u1m, u2m, u3m
    outcnt = outcnt + 1;
    sumdu = sumdu+du;
    sumq = sumq + (du-avdu) ** 2;
    sumu1 = sumu1 + u1;
    sumu2 = sumu2 + u2;
    sumu3 = sumu3 + u3;
    if outcnt == m:
        outcnt = 0
        checkscreen = True
        avduold = avdu
        avdu = sumdu/m
        rms = sqrt((sumq - (avdu-avduold) ** 2) / (m-1))
        u1m = sumu1/m
        u2m = sumu2/m
        u3m = sumu3/m
        sumu1 = 0
        sumu2 = 0
        sumu3 = 0
        sumdu = 0
        sumq = 0
    else:
        checkscreen = False
    return checkscreen


def writeStatus():
    #print('Device: {} Windows Status: {}\n').format(ser.name, str(ser.is_open))
    print('Device: ' + ser.name + ' Windows Status: ' + str(ser.is_open) + '\n')

def readcom():
    #print('*')             # for debug
    return ser.read(1)

def read8():                # read value and remember
#VAR  k : word;
    global raw, rawind
    k = ord(readcom())
    raw[rawind] = k         # remember raw values as words
    rawind = rawind +1
    return k

def read16():
#Var       k,k2  : word;
    global raw, rawind
    k = ord(readcom())        # low byte
    k2 = ord(readcom())
    k = 256 * k2 + k
    raw[rawind] = k         # remember raw values as word
    rawind = rawind +1
    return k

def read24():
#Var       k,k1,k2  :longint;
    global raw, rawind
    k =ord(readcom())         # low byte
    k1=ord(readcom())
    k2=ord(readcom())
    k = k + 256 * (k1 + 256 * k2)
    raw[rawind] = k         # remember raw values
    rawind = rawind +1
    return k

def readADC(k_0):           # read one ADC result
                            # k_0 is lenght of runup steps in cycle
                            # result is in cycles of small (negative) reference
#var  small, large, ru0 :longint
    global ru, adc1, adc2, adcdiff, adcalt
    ru = read16()             # run-up counts
    small = read16()          # cycles of weaker ref (negative)
    large = read16()          # cycles of stronger ref (pos)
    adc1 = read16()           # aux ADC , average integrator voltage, for info only
    adc1 = read16()           # residual charge after conversion
    adc2 = read16()           # residual charge before conversion
    adcdiff = adc2-adc1
    ru0 = round(adcclock *0.02 / (k_0+16) / 2)  # approximate zero : length K_o+16 is not accurate
    adcalt= adc2            # remember old res. charge (for version with only 1 reading)
    return (k_0 *(ru-ru0)) *(2+k1) + (small - large*(1+k1) + (adcdiff)*k1*k2)

def writetime(f):           # write date an time to file + info on konstants
    time = datetime.now(timezone.utc).astimezone(tz=None).replace(tzinfo=None)
    f.write('{:2d}/{:2d} {:2d}:{:2d}'.format(time.month, time.day, time.hour, time.minute))
    f.write(' with k0={}'.format(k0[1]))
    f.write(' k1={:10.4f}'.format(1/k1))
    f.write(' k2={:8.2f}\n'.format(4/k2))


def writeLogK():
#VAR  fkom : text;
    global sumk1, sumk2, countK
    with open('log_kom.txt', 'a') as fkom:
        fkom.write('# K1,K2,SF= {:14.6f} {:10.2f} {:15.7f}  '.format(sumk1, sumk2, sf*1000))
        writetime(fkom)
        sumk1 = 0
        sumk2 = 0
        countK = 0

def writeraw():             # write raw data to file (end of line)
# Var  i   :  word;
    global rawind
    for i in range (0, rawind, 1):   # i  := 0 to rawind-1 do
        f.write('\t{}'.format(raw[i]))
    f.write('\n')
    rawind = 0

def skalefactor1():         # result from slow slope measurement
#var  pulseL, adcl : integer;  # sums are global
    global sum, sum5, n5, sum25, n25
    sum = read24()          # sum of slow slope lenght
    pulseL = read8()        # pulse length
    if pulseL == 5:         # sum up data for short
       sum5 = sum5 + sum;
       n5 = n5 + 1;
    if pulseL == k1_puls:   # long case
        sum25 = sum25 + sum;
        n25 = n25 + 1;
    adcl = read16()           # dummy read not yet used, not needed
    adcl = read16()
    # write(f,n:4);     # type of data
    # writeraw;         # raw data

def skalefactor2():         # result of ADC scale factor measurement
#var m1,m2  : integer;
    global adc1, adc2, sum5, sum25, n5, n25, sumk1, sumk2, countk, sumsf, countsf, u, u2, u3, sf
    m1 = read8()            # number of steps up
    m2 = read8()            # number of steps down
    sumA = read16()
    sumB = read16()
    adc1 = read16()         # for debug only, but need to read !
    adc2 = read16()
    writeraw()
    if n5 > 2:              # enough data from adjustK1 ( slow slope)
        u = (sum25 / n25 - sum5 / n5)   # difference of average
        u3 = u / (3*(k1_puls-5)*128)    # calculate slope ratio, include fixed constants
        f.write('k1=\t{:14.6f}'.format(u3))
        print('## k1 = {:14.6f}'.format(u3), end = '')
        sum5 = 0            # reset sums after use
        sum25 = 0
        n5 = 0
        n25 = 0
    if (m1 > 2) and (m2 > 2):           # useful result from ADC scale
        u = sumA
        u = u/m1
        u2 = 65536-sumB
        u2 = u2/m2          # U2 is negative
        sumk1 = sumk1 + u3
        sumk2 = sumk2 + (u + u2) / 2
        countk = countk + 1
        if countsf == m_sf:
            print(' update scalefactor from ref reading old: {:13.5f}'.format(sf), end = '')
            sf = sumsf / m_sf
            print(' new: {:13.5f}'.format(sf), end = '')
            sumsf = 0       # reset sum to allow update
            countsf = 0
        if countk > 5:  writeLogK()
        f.write('\tk2=\t{:13.5f}\t{:11.3f}\t{:11.3f}\tSF=\t{:13.5f}\n'.format((u + u2) / 2, u, u2, sf*1000))
        print(' ## k2 : {:11.3f}{:11.3f}{:3.0f}{:3.0f}{:11.3f}'.format(u, u2, m1, m2, (u + u2) / 2))
    else:
        print(' Problem with ADC data: {:10.0f}{:10.0f}{:10.0f}{:10.0f}\n'.format(m1, m2, sumA, sumB))    # invalid data

def read2 (n):              # 254, 251: 2 readings (modes A, B, E)
    global u1, u2, u2old, f, du
    u1 = readADC(k0[ruv])       # result of 1 st conversion
    if n == 254:
        u2 = readADC(k0[ruv])   # result of 2. conversion
    else:
        u2 = readADC(k0[1])     # result of 2. conversion, mode B for INL test
    du = u1-0.5*(u2+u2old)
    f.write('{:11.3f}\t{:11.3f}\t{:13.4f}\t{:6.0f}'.format(u1, u2, du*sf, adcdiff))
    writeraw()
    if checkscreen():
        print('{} {:11.3f} {:11.3f} {:11.3f} {:13.4f} {:6.0f}{:13.5f}'.format( ru, u1m, u2m, du, sf*avdu, adc1, rms*sf))
    #switch (out_mode) {
    #    case 3:                 # full data with raw
    #        write(f,' ',u1:8:3,' ',u2:8:3,' ',du:8:3,' ', adcdiff:6)
    #        writeraw
    #        if checkscreen:
    #            writeln( ru,' ',u1m:8:3,' ',u2m:8:3,' ',du:8:3, ' ', sf*avdu:9:4,' ',adc1:6, rms*sf:8:5)
    #    case 2:                 # full data without raw
    #        writeln(f,' ',u1:8:3,' ',u2:8:3,' ',du:8:3,' ', adcdiff:6)
    #        if checkscreen:
    #            writeln( ru,' ',u1m:8:3,' ',u2m:8:3,' ',du:8:3, ' ', sf*avdu:9:4,' ',adc1:6, rms*sf:8:5)
    #    case 1:                 # only average
    #        if checkscreen:
    #            writeln( ru,' ',u1m:8:3,' ',u2m:8:3,' ',du:8:3, ' ', sf*avdu:9:4,' ',adc1:6, rms*sf:8:5)
    #            writeln( f,u1m:8:3,' ',u2m:8:3,' ',du:8:4, ' ', sf*avdu:9:5,' ', rms*sf:8:5)
    #}
    u2old = u2

def read2AE():
    read2(254)

def read2B():
    read2(251)

def read3():                # 250: 3 readings (mode C)
    global u1, u2, u3, u2old, f, du, countsf, sumsf
    u1 = readADC(k0[ruv])       # result of 1 st conversion
    u2 = readADC(k0[ruv])       # result of 2. conversion
    u3 = readADC(k0[ruv])       # result of 3. conversion
    if abs(u3-u2) > 1000 :
        du = (u1-u2)/(u3-u2)
        if countsf < m_sf:
            sumsf = sumsf + scale/(u3-u2)
            countsf = countsf + 1   # sum up the first m_sf scale factor readings
    else:
        du = (u1-u2) *sf/ 1000  # approx scale if no valid 7 V
    f.write('{:11.3f}\t{:11.3f}\t{:11.3f}\t{:13.4f}\t{:6.0f}'.format(u1, u2, u3, du*scale, adc1-adc2))
    writeraw()
    if checkscreen():
        print('{} {:11.3f} {:11.3f} {:11.3f} {:13.4f}{:6.0f} {:13.5f}'.format(ru, u1m, u2m, u3m, avdu*scale, adc1, rms*scale))
    u2old = u2

def read4():                # 248, 247: 4 readings (mode D)
    global u1, u2, u3, u4, u2old, f, du
    u1 = readADC(k0[ruv])      # result of 1 st conversion
    u2 = readADC(k0[ruv])      # result of 2. conversion
    u3 = readADC(k0[ruv])      # result of 3. conversion
    u4 = readADC(k0[ruv])      # result of 4. conversion
    if (abs(u4-u3) > 1000): du = (u2-u3)/(u4-u3)
    else: du = (u2-u3) * sf/1000.0  # approx scale if no valid 7 V
    f.write('{:11.3f}\t{:11.3f}\t{:11.3f}\t{:11.3f}\t{:13.4f}\t{:6.0f}'.format(u1, u2, u3, u4, du*scale, adc1-adc2))
    writeraw()
    if checkscreen():
        print('{} {:11.3f} {:11.3f} {:11.3f} {:11.3f} {:13.4f}{:6.0f} {:13.5f}'.format(ru, u1m, u2m, u3m, u4, avdu*scale, adc1, rms*scale))
    u2old = u2

def readda():               # 241: DA-Test 26 chars (mode G)
    global f
    da = [0 for i in range(25+1)]
    print('DA result: ', end = '')
    das = '0x'
    for n in range (0, 25+1, 1):
        da[n] = ord(readcom())
        print('{:3d} '.format(da[n]), end = '')
        das = das + hex(da[n])
    print()
    f.write(das + '\n')

def keypress(key):
    global k, ser, ruv
    k = (key).upper()
    if ((k >= '0') and (k <= '7')) or ((k >= 'A') and (k <= 'W')):
        ser.write(k.encode("utf-8"))
        print('Sent: ' + k)
        if (k >= 'P') and (k <= 'W'): ruv = ord(k)-80      # update runup version

def main():                     # main program
    global k, ser, f, fkom

    try:

        print('Opening file...')
    
    #VAR     fn,kom : string;
    #		fkom_ : Longint;
        fn = input('Filename ')
        if fn == '': fn = 'test.txt'
        kom = input('Kommentar ')
        
        with open(fn, 'w') as f:
        #with open(fn, 'a') as f:        # TODO debug only 'x'
            f.write('# ')
            writetime(f)
            f.write('# {}\n'.format(fn))
            f.write('# {}\n'.format(kom))
        
            if fn != 't':
                with open('log_kom.txt', 'a') as fkom:  # extra log file
                    fkom.write('{} {}\n'.format(fn, kom))
                    writetime(fkom)
            print('Opening COM-Port...')

            ser = serial.Serial(port=serial_port,
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
            writeStatus()
        
            print('COM-Port opened!\n')
        
            ser.write('C'.encode("utf-8"))
            print('Sent: C')  # call for mode C = AZ mode with ref
            k = ' '                 # key - space as not send
            time.sleep(10E-3)
            writeStatus()
        
            action = {              # action depending on tag
                254: read2AE,       # 2 readings (modes A, E)
                251: read2B,        # 2 readings (mode B)
                250: read3,         # 3 readings (mode C)
                253: skalefactor1,  # slope ratio measurement (mode K?)
                252: skalefactor2,  # ADC skale factor and ouput of slope ratio (mode L)
                248: read4,         # 4 readings (mode D?)
                247: read4,         # 4 readings (mode D?)
                241: readda         # DA-Test 26 chars (mode G)
            }

            # handle keypresses
            on_press=keypress
            t = Thread(target=listen_keyboard, args=(on_press,))
            t.daemon = True
            t.start()

            while (k != 'X'):       # loop to receive & send data
                rawind = 0          # new package, reset counter for raw data
                while ord(readcom()) != 255: pass     # wait for sync
                c = readcom()       # get tag
                n = ord(c)          # for debuging:   write(n,'  ');
                action.get(n, lambda: 'Invalid')()         # action depending on tag

    except Exception as e:
        # TODO
        print('Error: ' + str(e))
        
if __name__ == "__main__":
	main()