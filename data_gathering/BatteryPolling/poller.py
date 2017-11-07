from smbus2 import SMBus
import mysql.connector
import time


class LtcPoll:
    def __init__(self, smbus, debug=True):
        self.debug = debug
        self.bus = smbus
        self.addr = 0x6c
        self.vinReg = (0x02, 0x03)
        self.adinReg = (0x04, 0x05)
        self.version = 'vin'
        self.table = 'LtcPoll'+time.strftime("%H:%M:%S %d/%m/%Y")
        db = mysql.connector.connect(host="localhost", user="root", password="toor", database="batterystudy")
        self.curs = db.cursor()
        createString = '''DROP TABLE IF EXISTS {table}; CREATE TABLE {table} (id INTEGER NOT NULL AUTO_INCREMENT, time DECIMAL(20,2), vin INTEGER, adin INTEGER, resistance DECIMAL, current INTEGER);'''.format(table=self.table)
        self.execute(createString)


    def poll(self, reg):

        if self.version == 'vin':
            val = self.bus.read_word_data(self.addr, self.vinReg[0])
            val = val << 4
            val = val | self.bus.read_word_data(self.addr, self.vinReg[1])
            self.post([('vin', val)])

        elif self.version == 'adin':
            val = self.bus.read_word_data(self.addr, self.vinReg[0])
            val = val << 4
            val = val | self.bus.read_word_data(self.addr, self.vinReg[1])
            self.post([('adin', val)])
    
        else:
            vinVal = self.bus.read_word_data(self.addr, self.vinReg[0])
            vinVal = vinVal << 4
            vinVal = vinVal | self.bus.read_word_data(self.addr, self.vinReg[1])
            adinVal = self.bus.read_word_data(self.addr, self.adinReg[0])
            adinVal = adinVal << 4
            adinVal = adinVal | self.bus.read_word_data(self.addr, self.adinReg[1])
            self.post([('vin', vinVal), ('adin', adinVal)])
    

    def post(self, vals):
        if self.debug: print('[+] Recv post vals of:', vals)
        cols = vals[0][0]
        values = vals[0][1]
        if len(vals) > 1:
            for val in vals[1:]:
                cols += ', ' + val[0]
                values += ', ' + str(val[1])
        insertString = 'INSERT INTO {table} ({columns}) VALUES ({vals})'.format(columns=cols, table=self.table, vals=values)
        self.execute(insertString)

    def execute(self, string):
        if self.debug: print('[+] Executing:',string)
        self.curs.execute(string)

if __name__ == '__main__':
    bus = SMBus(1)
    poller = LtcPoll.__init__(bus)
    poller.start()