import mysql.connector
from smbus2 import SMBus
import time
from datetime import datetime
import mutex
import inspect
import functools

def autoargs(*include, **kwargs):
    def _autoargs(func):
        attrs, varargs, varkw, defaults = inspect.getargspec(func)
        def seive(attr):
            if kwargs and attr in kwargs['exclude']:
                return False
            if not include or attr in include:
                return True
            else:
                return False
        @functools.wraps(func)
        def wrapper(self, **args, **kwargs):
            if defaults:
                for attr, val in zip(reversed(attrs), reversed(defaults)):
                    if seive(attr):
                        setattr(self, arrt, val)
            positional_attrs = attrs[1:]
            for attr, val in zip(positional_attrs, args):
                if seive(attr):
                    setattr(self, attr, val)
            if varargs:
                remaining_args = args[len(positional_attrs):]
                    if seive(varargs):
                        setattr(self, varargs, remaining_args)
            if kwargs:
                for attr, va in kwargs.items():
                    if seive(attr):
                        setattr(self, attr, val)
            return func(self, *args, **kwargs)
        return wrapper
    return _autoargs

# init db
db = mysql.connector.connect(host='localhost', user='root', password='toor', database='batterystudy')
curs = db.cursor()

mutex_list = set(

)

class Poster():
    @autoargs
    def __init__(self, bvoltage=None, bcurrent=None, btemp=None, brelsoc=None,
                babssoc=None, bttoempty=None, bremcap=None, bfullcap=None, 
                bchangingcurr=None, bchangingvolt=None, bmaxerror=None,
                ldischargevoltage=None, lcurrentdraw=None, lresistance=None):
        
        # the above is equivalent to the following :
        
        # # battery reads
        # self.bvoltage = bvoltage
        # self.bcurrent = bcurrent
        # self.btemperature = btemp
        # self.brelsoc = brelsoc
        # self.babssoc = bbssoc
        # self.bttoempty = bbttoempty
        # self.bremcap = bremcap
        # self.bfullcap = bfullcap
        # self.bchangingcurr = bchangingcurr
        # self.bchangingvolt = bchangingvolt
        # self.bmaxerror = bmaxerror
        # # ltc reads
        # self.ldischargevoltage = ldischargevoltage
        # self.lcurrentdraw = lcurrentdraw
        # self.lresistance = lresistance

    def update(self, **kwargs):
        try:
            for k, v in kwargs.iteritems():
                self.



# LTC4151 Init
laddr = 0x6c
vinreg = (0x02,0x03)
adinreg = (0x04,0x04)
controlreg=(0x06)

ltccommands = {
    'senseChannel': 0x00, # sense channel for snapshot
    'vinChannel':   0x20, # vin channel for snapshot
    'adinChannel':  0x40, # adin channel for snapshot
    'MODES': {
        'contMode':     0x00, # turn reg to cont mode
        'snapShotMode': 0x80, # turn reg to snapshot mode
    }
}


#================#
#   SMBus Init   #
#================#
bus = SMBus(1)

# Battery Init
baddr = 0x0b
failedcycles = 0
batteryCommands = {
    'voltage': bus.read_word_data(baddr, 0x09),
    'current': bus.read_word_data(addr, 0x0a),
    'temperature': bus.read_word_data(addr, 0x08),
    'relsoc': bus.read_word_data(addr, 0x0d),
    'abssoc': bus.read_word_data(addr, 0x0e),
    'ttoempty': bus.read_word_data(addr, 0x12),
    'remcap': bus.read_word_data(addr, 0x0f),
    'fullcap': bus.read_word_data(addr, 0x10),
    'changingcur': bus.read_word_data(addr, 0x14),
    'changingvolt': bus.read_word_data(addr, 0x15),
    'maxerror': bus.read_word_data(addr, 0x0c),
}


class BatteryPoller():

    def __init__(self, remotereg, remotemutex, addr=0x0b, commands=batteryCommands):
        self.reg = remotereg
        self.remotemutex = remotemutex
        self.addr = addr
        self.commands = commands
        self.fields = {}

    def post(self,):
        self.reg = self.fields





if __name__ == '__main__':

    # see what the fuck this is actually used for
    #Exit when data is not being pulled from smbus
    while cycle == "":
        print failedcycles
        if failedcycles == 13:
            print "SMBus is broken/diconnected"
            exit()
        try:
            serial = str(bus.read_word_data(addr, 0x1c))     #Serial Number
                cycle = str(bus.read_word_data(addr, 0x17))  # Cycle Count

        except IOError:
                print "Could not connect to bus"
                time.sleep(1)
                failedcycles += 1
    ####################################
    tablename = datetime.time()
