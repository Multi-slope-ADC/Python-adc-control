#!/usr/bin/python3

import time
from datetime import datetime, timezone
import serial
from math import *
from threading import Thread
from sshkeyboard import listen_keyboard
from statistics import median

serial_port = 'COM4'	# e.g. /dev/ttyUSB0 on Linux, COM1 on Windows
m = 10					# output to screen every m results

# constants depending on ADC board (HW and software)
# TODO UPPERCASE in python 
adcclock = 12000000		# clock on ADC board
scale = 6951.926		# Ref voltage in mV 7106.8384
xd = 12*3				# extra  delay 3x xdel in ASM code !!! has to match FW !!!
k1 =  1.0 / 20.9637		# measured ref ratios from adjustment
k2 =  4.0 / 121.66		# fine step / adc LSB

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
    print('Device: ' + ser.name + ' Status: ' + str(ser.is_open) + '\n')

def readbytes(n):                # read value and remember
    global raw, rawind
    bytesread = bytearray(n)
    count = ser.readinto(bytesread)
    if count != n: print('ERROR: Wanted to read {} bytes, but read {} bytes instead'.format(n, count))  #TODO raise exception
    k = int.from_bytes(bytesread, byteorder='little', signed=False)
    raw[rawind] = k         # remember raw values as words
    rawind += 1
    return k

def readADC(k_0):           # read one ADC result
                            # k_0 is lenght of runup steps in cycle
                            # result is in cycles of small (negative) reference
#var  small, large, ru0 :longint
    global ru, adc1, adc2, adcdiff, adcalt
    ru = readbytes(2)             # run-up counts
    small = readbytes(2)          # cycles of weaker ref (negative)
    large = readbytes(2)          # cycles of stronger ref (pos)
    adc1 = readbytes(2)           # aux ADC , average integrator voltage, for info only
    adc1 = readbytes(2)           # residual charge after conversion
    adc2 = readbytes(2)           # residual charge before conversion
    adcdiff = adc2-adc1
    ru0 = round(adcclock *0.02 / (k_0+16) / 2)  # approximate zero : length K_o+16 is not accurate
    adcalt= adc2            # remember old res. charge (for version with only 1 reading)
    return (k_0 *(ru-ru0)) *(2+k1) + (small - large*(1+k1) + (adcdiff)*k1*k2)

def writetime(f):           # write date and time to file + info on konstants
    time = datetime.now(timezone.utc).astimezone(tz=None).replace(tzinfo=None)
    f.write(str(datetime.now().astimezone().replace(microsecond=0)))
    f.write(f' with k0={k0[1]} k1={1/k1:10.4f} k2={4/k2:8.2f} scale={scale}\n'.format())

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
    #for i in range (0, rawind, 1):   # i  := 0 to rawind-1 do
    #    #f.write(' {}'.format(raw[i]))
    #    f.write('\t{}'.format(raw[i]))
    #f.write('\n')
    rawind = 0

def skalefactor1():         # result from slow slope measurement
#var  pulseL, adcl : integer;  # sums are global
    global sum, sum5, n5, sum25, n25
    sum = readbytes(3)          # sum of slow slope lenght
    pulseL = readbytes(1)        # pulse length
    if pulseL == 5:         # sum up data for short
       sum5 = sum5 + sum;
       n5 = n5 + 1;
    if pulseL == k1_puls:   # long case
        sum25 = sum25 + sum;
        n25 = n25 + 1;
    adcl = readbytes(2)           # dummy read not yet used, not needed
    adcl = readbytes(2)

def skalefactor2():         # result of ADC scale factor measurement
#var m1,m2  : integer;
    global adc1, adc2, sum5, sum25, n5, n25, sumk1, sumk2, countk, sumsf, countsf, u, u2, u3, sf, k1, k2, k1values, k2values
    m1 = readbytes(1)            # number of steps up
    m2 = readbytes(1)            # number of steps down
    sumA = readbytes(2)
    sumB = readbytes(2)
    adc1 = readbytes(2)         # for debug only, but need to read !
    adc2 = readbytes(2)
    writeraw()
    if n5 > 2:              # enough data from adjustK1 ( slow slope)
        u = (sum25 / n25 - sum5 / n5)   # difference of average
        u3 = u / (3*(k1_puls-5)*128)    # calculate slope ratio, include fixed constants
        #f.write('k1=\t{:14.6f}'.format(u3))
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

        k1values.append(u3)
        k2values.append((u + u2) / 2)
        if len(k1values) >= m_sf:
            print('\nOld k1 & k2: {:.6f} {:.3f}'.format(1.0 / k1, 4.0 / k2))
            k1 =  1.0 / median(k1values)
            k2 =  4.0 / median(k2values)

            print('New k1 & k2: {:.6f} {:.3f}'.format(1.0 / k1, 4.0 / k2))
            k1values = list()
            k2values = list()

        if countsf == m_sf:
            print(' update scalefactor from ref reading old: {:13.10f}'.format(sf), end = '')
            sf = sumsf / m_sf
            print(' new: {:13.10f}'.format(sf), end = '')
            sumsf = 0       # reset sum to allow update
            countsf = 0
        if countk > 5:  writeLogK()
        #f.write('\tk2=\t{:13.5f}\t{:11.3f}\t{:11.3f}\tSF=\t{:13.5f}\n'.format((u + u2) / 2, u, u2, sf*1000))
        print(' ## k2 : {:11.3f}{:11.3f}{:3.0f}{:3.0f}{:11.3f}'.format(u, u2, m1, m2, (u + u2) / 2))
    else:
        print(' Problem with ADC data: {:10.0f}{:10.0f}{:10.0f}{:10.0f}\n'.format(m1, m2, sumA, sumB))    # invalid data

def setscalefactor(n):                  # acal for modes with 2 readings, updates sf
    global setsfstate, sfvalues, sf
    if n == 254:            # n = action - setsf only for Mode A & E
        # get m_sf readings of 7V & GNDS
        if setsfstate == 0:
            ser.write('6'.encode("utf-8"))
        if setsfstate >= 1 and setsfstate <= m_sf + 1:
            sfvalues[setsfstate-1] = u1 - u2

        setsfstate += 1

        if setsfstate > m_sf + 1:                     # setsf finished
            setsfstate = -1
            print('update scalefactor from ref reading old: {:13.10f}'.format(sf), end = '')
            sf = scale/median(sfvalues)
            print(' new: {:13.10f}'.format(sf))

    else:
        setsfstate = -1
        print('Error: update scalefactor only awailable in mode A & E!')

def read2 (n):              # 254, 251: 2 readings (modes A, B, E)
    global u1, u2, u1old, u2old, f, du
    u1 = readADC(k0[ruv])       # result of 1. conversion
    if n == 254:
        u2 = readADC(k0[ruv])   # result of 2. conversion
        du = u1 - u2
    else:
        u2 = readADC(k0[1])     # result of 2. conversion, mode B for INL test
        du = (3*(u2old-u1)-u1old+u2)/4   # interpolate both values

    #f.write('{:11.3f}\t{:11.3f}\t{:13.4f}\t{:6.0f}'.format(u1, u2, du*sf, adcdiff))
    writeraw()
    if checkscreen():
        print('{} {:11.3f} {:11.3f} {:11.3f} {:13.4f} {:6.0f}{:13.5f}'.format( ru, u1m, u2m, du, sf*avdu, adc1, rms*sf))
    u1old = u1
    u2old = u2
    return [u1, u2, du, sf, adcdiff]

def read2AE():
    return read2(254)

def read2B():
    return read2(251)

def read3():                # 250: 3 readings (mode C)
    global u1, u2, u3, u2old, f, du, countsf, sumsf
    u1 = readADC(k0[ruv])       # result of 1. conversion
    u2 = readADC(k0[ruv])       # result of 2. conversion
    u3 = readADC(k0[ruv])       # result of 3. conversion
    if abs(u3-u2) > 1000 :
        du = (u1-u2)/(u3-u2)
        if countsf < m_sf:
            sumsf = sumsf + scale/(u3-u2)
            countsf = countsf + 1   # sum up the first m_sf scale factor readings
    else:
        du = (u1-u2) *sf/ 1000  # approx scale if no valid 7 V
    #f.write('{:11.3f}\t{:11.3f}\t{:11.3f}\t{:13.4f}\t{:6.0f}'.format(u1, u2, u3, du*scale, adc1-adc2))
    writeraw()
    if checkscreen():
        print('{} {:11.3f} {:11.3f} {:11.3f} {:13.4f}{:6.0f} {:13.5f}'.format(ru, u1m, u2m, u3m, avdu*scale, adc1, rms*scale))
    u2old = u2
    return [u1, u2, u3, du, scale, adc1-adc2]

def read4():                # 248, 247: 4 readings (mode D)
    global u1, u2, u3, u4, u2old, f, du
    u1 = readADC(k0[ruv])      # result of 1. conversion
    u2 = readADC(k0[ruv])      # result of 2. conversion
    u3 = readADC(k0[ruv])      # result of 3. conversion
    u4 = readADC(k0[ruv])      # result of 4. conversion
    if (abs(u4-u3) > 1000): du = (u2-u3)/(u4-u3)
    else: du = (u2-u3) * sf/1000.0  # approx scale if no valid 7 V
    #f.write('{:11.3f}\t{:11.3f}\t{:11.3f}\t{:11.3f}\t{:13.4f}\t{:6.0f}'.format(u1, u2, u3, u4, du*scale, adc1-adc2))
    writeraw()
    if checkscreen():
        print('{} {:11.3f} {:11.3f} {:11.3f} {:11.3f} {:13.4f}{:6.0f} {:13.5f}'.format(ru, u1m, u2m, u3m, u4, avdu*scale, adc1, rms*scale))
    u2old = u2

def readda():               # TODO 241: DA-Test (mode G)
# Charge (40.5 µs), wait (uart send nx17 cycles), run-down (uart send 2x2bytes), read ADC wait, read ADC x7, wait ~520µs, read ADC, wait ~780µs, read ADC, wait ~26 ms, read ADC, run-down (uart send 2x2bytes), read ADC wait, wait ~13ms, read ADC, wait ~13ms, read ADC
    global f
    da = [0 for i in range(34)]
    print('DA result: ', end = '')
    das = '0x'
    for n in range (0, 34, 1):
        #da[n] = ord(ser.read(1))
        da[n] = readbytes(2)
        print('{:4d} '.format(da[n]), end = '')
        das = das + hex(da[n])
    print()
    #f.write(das)
    writeraw()

def keypress(key):
    global k, ser, ruv
    k = (key).upper()
    if len(k) == 1 and (((k >= '0') and (k <= '7')) or ((k >= 'A') and (k <= 'W'))):
        ser.write(k.encode("utf-8"))
        print('Sent: ' + k)
        if (k >= 'P') and (k <= 'W'): ruv = ord(k)-80      # update runup version
        
def read():
    global n
    rawind = 0          # new package, reset counter for raw data
    while ord(ser.read(1)) != 255: pass     # wait for sync
    n = ord(ser.read(1))       # get tag
    return action.get(n, lambda: 'Invalid')()         # action depending on tag

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
    global k, ser, f, fkom, setsfstate

    #try:

    print('Opening file...')
    
    #fn = input('Filename ')
    #if fn == '': fn = 'test.txt'
    #kom = input('Kommentar ')

    # TODO new directory for each run
 
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

    writeStatus()
        
    print('COM-Port opened!\n')
        
    desc = 'delayed effect 0V & -7Vext'    # Datafile
    runups = ['Q', 'W', 'P']            # run-up versions as char
    inputs = ['7', '0']                 # inputs as char
    fw = ['delayed mux', 'B same ru']                 # Firmware version details
    kom = ''                # Description

    # handle keypresses
    on_press=keypress
    t = Thread(target=listen_keyboard, args=(on_press,))
    t.daemon = True
    t.start()

    for runup in runups:
        fn = f"{desc} - mode A, ru {runup}, inputs {', '.join(inputs)} - FW {', '.join(fw)}.csv" #{datetime.now().astimezone().replace(microsecond=0).replace(':', '').replace('-', '')}

        with open('log_kom.txt', 'a') as fkom:  # extra log file
            fkom.write('{} {}\n'.format(fn, kom))
            writetime(fkom)

        with open(fn, 'x') as f:
            f.write('# ')
            writetime(f)
            f.write('# {}\n'.format(fn))
            f.write('# {}\n'.format(kom))
         
            keypress(inputs[0])     # call for input#
            time.sleep(100E-3)
            keypress('B')           # call for mode B
            time.sleep(100E-3)
            keypress(runup)           # call for run-up mode

            k = ''                  # reset key

            time.sleep(200E-3)

            values = list()
        
            def readntimes(n, key):
                for l2 in range (0, n):
                    if l2 == n-1: keypress(key)
                    actionresult = read()
                    #print('Key: ' + k)
                    #if l1 >= start:
                    if actionresult != 'Invalid':
                        values.append([k, 2*l2+1, actionresult[0]])
                        values.append([k, 2*l2+2, actionresult[1]])
                    else: print('Invalid')

    # get typical values
            count = 400
            key = inputs[1]                   # 

            ser.flushInput()              # discard serial buffer .read_all
            readntimes(count, key)
            median0V = median([t[2] for t in values])

            values = list()         #reset values list
            #key = '7'
            readntimes(count, key)
            median7V = median([t[2] for t in values])

            properscaleµV = (scale*1000)/(median7V - median0V) * ((median7V > 0) - (median7V < 0))  # (median7V > 0) - (median7V < 0) gives sign of median7V
            print(str(properscaleµV))
        
            values = list()         #reset values list
            k = ''                  # reset key

            # mode A run-up (2 readings AZ-mode)
            keypress(inputs[1])           # call for input
            time.sleep(100E-3)
            keypress('A')           # call for mode A (AZ mode)
            time.sleep(100E-3)
            keypress(runup)           # call for run-up mode

            k = ''                  # reset key
            #time.sleep(1)           # wait to settle

            for l1 in range (1000):
                actionresult = read()
                if actionresult != 'Invalid':
                    values.append(['0', 1, actionresult[0]])
                    values.append(['7', 1, actionresult[1]])
                else: print('Invalid')
        
            # write values to file in convenient way
            prevvalue = values[0]
            count = 0
            prevkey = ''
            prevprevkey = ''
            key = ''
            valuescorrect = list()
            for val in values:

                if abs(prevvalue[2] - val[2]) > 10:          # new key
                    count = 1
                    key = prevprevkey
                if key != '':
                    valuescorrect.append([key, count, val[2]])
                    f.write('{}\t{:3d}\t{:11.3f}\n'.format(key, count, (val[2]-median0V)*properscaleµV))
                    count += 1
                prevprevkey = prevkey
                prevkey = val[0]
                prevvalue = val


        
    for runup in runups:
        fn = f"{desc} - mode B, ru {runup}, inputs {', '.join(inputs)} - FW {', '.join(fw)}.csv" #{datetime.now().astimezone().replace(microsecond=0).replace(':', '').replace('-', '')}

        with open('log_kom.txt', 'a') as fkom:  # extra log file
            fkom.write('{} {}\n'.format(fn, kom))
            writetime(fkom)

        with open(fn, 'x') as f:
            f.write('# ')
            writetime(f)
            f.write('# {}\n'.format(fn))
            f.write('# {}\n'.format(kom))
         
            keypress(inputs[0])     # call for input#
            time.sleep(100E-3)
            keypress('B')           # call for mode B
            time.sleep(100E-3)
            keypress(runup)           # call for run-up mode

            k = ''                  # reset key

            time.sleep(200E-3)

            values = list()
        
            def readntimes(n, key):
                for l2 in range (0, n):
                    if l2 == n-1: keypress(key)
                    actionresult = read()
                    #print('Key: ' + k)
                    #if l1 >= start:
                    if actionresult != 'Invalid':
                        values.append([k, 2*l2+1, actionresult[0]])
                        values.append([k, 2*l2+2, actionresult[1]])
                    else: print('Invalid')

    # get typical values
            count = 400
            key = inputs[1]                   # 

            ser.flushInput()              # discard serial buffer .read_all
            readntimes(count, key)
            median0V = median([t[2] for t in values])

            values = list()         #reset values list
            #key = '7'
            readntimes(count, key)
            median7V = median([t[2] for t in values])

            properscaleµV = (scale*1000)/(median7V - median0V) * ((median7V > 0) - (median7V < 0))  # (median7V > 0) - (median7V < 0) gives sign of median7V
            print(str(properscaleµV))
        
            values = list()         #reset values list
            k = ''                  # reset key

            # delayed/settling effects - mode B
            ser.flushInput()              # discard serial buffer .read_all
            for l1 in range (100):
                count = 10          # count for other key 10 to settle = 20 readings
                start = 1
                key = inputs[1]           # key for other reading
                readntimes(count, key)

                count = 10          # count for other key
                key = inputs[0]           # key for other reading
                readntimes(count, key)
        
            # write values to file in convenient way
            prevvalue = values[0]
            count = 0
            prevkey = ''
            prevprevkey = ''
            key = ''
            valuescorrect = list()
            for val in values:

                if abs(prevvalue[2] - val[2]) > 10:          # new key
                    count = 1
                    key = prevprevkey
                if key != '':
                    valuescorrect.append([key, count, val[2]])
                    f.write('{}\t{:3d}\t{:11.3f}\n'.format(key, count, (val[2]-median0V)*properscaleµV))
                    count += 1
                prevprevkey = prevkey
                prevkey = val[0]
                prevvalue = val



        #median7 = median([t[2] for t in values if t[0] == '7'])
        #median6 = median([t[2] for t in values if t[0] == '6'])
        
        #diff = abs(median7 - median6)

        keypress('7')           # always end with 0V
        
        #print ('Median 7: ' + str(median7) + ' Median 6: ' + str(median6))
        #while (k != 'X'):       # loop to receive & send data
        #    rawind = 0          # new package, reset counter for raw data
        #    while ord(readcom()) != 255: pass     # wait for sync
        #    c = readcom()       # get tag
        #    n = ord(c)          # for debuging:   write(n,'  ');
        #    actionresult = action.get(n, lambda: 'Invalid')()         # action depending on tag

        #    if actionresult != 'Invalid': f.write('{:11.3f}\n{:11.3f}\n'.format(actionresult[0], actionresult[1]))

            #while (k != 'X'):       # loop to receive & send data
            #    rawind = 0          # new package, reset counter for raw data
            #    while ord(ser.read(1)) != 255: pass     # wait for sync
            #    n = ord(ser.read(1))       # get tag
            #    action.get(n, lambda: 'Invalid')()         # action depending on tag
            #    if k == 'Z':        # set scalefactor for 2 reading mode
            #        setsfstate = 0
            #        k = ''
            #        print('starting update scalefactor...')
            #    if setsfstate >= 0: setscalefactor(n)

    #except Exception as e:
    #    print('Error: ' + str(e))
        
if __name__ == "__main__":
	main()