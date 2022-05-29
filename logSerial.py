#!/home/pi/p310/bin/python
#!/cygdrive/c/Python37/Python.exe

from time import sleep, time, localtime
from threading import Thread, Lock
from requests import get
import serial
import datetime
import signal
import sys
from platform import system
from sys import argv
from pytz import timezone


sysType = system().lower()

if 'linux' in sysType:
    termDev = '/dev/ttyUSB0'
    ping = True
elif 'windows' in sysType:
    termDev = 'COM9'
    ping = False
elif 'cygwin' in sysType:
    termDev = '/dev/ttyS4'
    ping = False
termSpeed = 9600
#logDir = "/home/pi/water/"
logDir = ""
#pingURL = "http://test.ericnystrom.com/alert/check?id=rpi"
pingURL = None

tz = timezone("America/New_York")

if len(argv) >= 2:
    termDev = argv[1]

def getFileDate():
    return('{:%Y%m%d}'.format(datetime.datetime.now(tz=tz)))

def signalHandlerInt(signal, frame):
    global f
    global lineNum
    global done
    # print 'exit int'
    done = True
    try:
        if lineNum != 0:
            f.flush()
        f.close()
    except:
        pass
    sys.exit(0)

def signalHandlerTerm(signal, frame):
    global f
    global lineNum
    global done
    # print 'exit term'
    done = True
    try:
        if lineNum != 0:
            f.flush()
        f.close()
    except:
        pass
    sys.exit(0)

def timeStamp(f):
    ts = '{:%Y-%m-%d %H:%M:%S}'.format(datetime.datetime.now(tz=tz))
    fileLock.acquire(True)
    f.write((ts + '\n').encode('utf-8'))
    fileLock.release()

def timeString():
    return '{:%H:%M:%S} '.format(datetime.datetime.now(tz=tz))
            
class FlushThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        # print 'flushThread init'
        self.start()

    def run(self):
        # print 'flushThread run'
        global done
        global f
        global lastTime
        global lineNum
        global fileLock
        global lastDay
        global ping
        lastStamp = time()
        lastPing = lastStamp
        timeStamp(f)
        while not done:
            sleep(1)
            curTime = time()
            deltaTime = curTime - lastTime
            if deltaTime > (30 * 60):
                lastTime = curTime
                if lineNum != 0:
                    lineNum = 0
                    # print "flush"
                    fileLock.acquire(True)
                    f.flush()
                    fileLock.release()
            deltaTime = curTime - lastStamp
            if deltaTime > (60 * 60):
                lastStamp = curTime
                timeStamp(f)
            deltaTime = curTime - lastPing
            if ping and (deltaTime > (1 * 60)):
                lastPing = curTime
                try:
                    if pingURL is not None:
                        r = get(pingURL)
                    # fileLock.acquire(True)
                    # f.write("%s ping\n" % (timeString()))
                    # fileLock.release()
                except:
                    fileLock.acquire(True)
                    f.write(("%s ping failure\n" % (timeString())).encode('utf8'))
                    fileLock.release()
            day = localtime().tm_mday
            if day != lastDay:
                lastDay = day
                fileLock.acquire(True)
                f.close()
                f = open(logDir + "log" + getFileDate() + ".dat", 'ab')
                fileLock.release()
        # print 'thread done'

signal.signal(signal.SIGINT, signalHandlerInt)
signal.signal(signal.SIGTERM, signalHandlerTerm)

ser = serial.Serial(termDev, termSpeed) #, timeout = 1)

lastDay = localtime().tm_mday
f = open(logDir + "log" + getFileDate() + ".dat", 'ab')
done = False
lineNum = 0
lastTime = time()
fileLock = Lock()
flushThread = FlushThread()
blank = False
while True:
    try:
        line = ser.readline()
    except:
        # print 'exception'
        break
    # print line
    line = line.strip()
    if len(line) == 0:
        if blank:
            continue
        blank = True
    else:
        blank = False
    fileLock.acquire(True)
    f.write(timeString().encode('utf-8'))
    f.write(line)
    f.write('\n'.encode('utf-8'))
    fileLock.release()
    lineNum += 1
    # if lineNum > 128:
    if lineNum > 16:
        lineNum = 0
        lastTime = time()
        # print "flush"
        fileLock.acquire(True)
        f.flush()
        fileLock.release()
