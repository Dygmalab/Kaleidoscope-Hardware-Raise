DEFAULT_USERROW=0xFFFFFC5DD8E0C7FF
USERROW_BIN = bin(DEFAULT_USERROW).replace('0b','')
assert(len(USERROW_BIN) == 64)

class UR(object):

    def __init__(self, name, bits, default):
        self.name = name
        self.bits = bits

        self.slice = USERROW_BIN[63-self.bits[0]:63-self.bits[1]+1]
        self.value = default #int(self.slice, 2)
        self.bin = "{:0{width}b}".format(self.value, width=self.bits[0]-self.bits[1]+1)
   
    def __str__(self):
        return "%10s : %20s %d" % (self.name, self.bin, self.value)

rows = [
    # page 31 for NVM User Row Mapping
    UR('bootprot',  (2,0), 2),          # page 361
    UR('res1',      (3,3), 1),
    UR('eeprom',    (6,4), 0),          # page 361
    UR('res2',      (7,7), 1),
    UR('bod33lev',  (13,8), 7),
    UR('bod33en',   (14,14), 1),
    UR('bod33act',  (16,15), 1),
    UR('res3',      (24,17), 0x70),
    UR('wdtenable', (25,25), 0),
    UR('wdtalways', (26,26), 0),
    UR('wdtperiod', (30,27), 0xB),
    UR('wdtwindow', (34,31), 0xB),
    UR('wdtoffset', (38,35), 0xB),
    UR('wdtwen',    (39,39), 0),
    UR('bod33hyst', (40,40), 0),
    UR('res4',      (41,41), 0),
    UR('res5',      (47,42), 63),
    UR('lock',      (63,48), 0xFFFF),
    ]

# build the new userrow string
new_row_bin = ""
for row in rows:
    print(row)
    new_row_bin = row.bin + new_row_bin

# convert to integer
new_row = int(new_row_bin, 2)

print("default: %x" % DEFAULT_USERROW)
print("new    : %x" % new_row)
