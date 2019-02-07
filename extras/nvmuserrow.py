#USERROW=0xFFFFFC5DD8E0C7FF
USERROW=0xFFFFFC5DD8E0C7FA
USERROW_BIN = bin(USERROW).replace('0b','')
assert(len(USERROW_BIN) == 64)
class UR(object):

    def __init__(self, name, bits):
        self.name = name
        self.bits = bits

        self.slice = USERROW_BIN[63-self.bits[0]:63-self.bits[1]+1]
        self.value = int(self.slice, 2)

    def __str__(self):
        return "%10s : %20s %d" % (self.name, self.slice, self.value)

rows = [
    # page 31 for NVM User Row Mapping
    UR('bootprot',  (2,0)),
    UR('res1',      (3,3)),
    UR('eeprom',    (6,4)),
    UR('res2',      (7,7)),
    UR('bod33lev',  (13,8)),
    UR('bod33en',   (14,14)),
    UR('bod33act',  (16,15)),
    UR('res3',      (24,17)),
    UR('wdtenable', (25,25)),
    UR('wdtalways', (26,26)),
    UR('wdtperiod', (30,27)),
    UR('wdtwindow', (34,31)),
    UR('wdtoffset', (38,35)),
    UR('wdtwen',    (39,39)),
    UR('bod33hyst', (40,40)),
    UR('res4',      (41,41)),
    UR('res5',      (47,42)),
    UR('lock',      (63,48)),
    ]
print("%x" % USERROW)
print(USERROW_BIN)
for row in rows:
    print(row)
#print(userrow)
