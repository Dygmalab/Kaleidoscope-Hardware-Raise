DEFAULT_USERROW=0xFFFFFC5DD8E0C7FF

class NVM(object):

    def __init__(self):
        self.user_row = DEFAULT_USERROW
        self.rows = [
            # page 31 for NVM User Row Mapping
            UR('bootprot',  (2,0), 7),          # page 361
            UR('res1',      (3,3), 1),
            UR('eeprom',    (6,4), 7),          # page 361
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
        
        # check string building works
        self.update_user_row()
        assert(self.user_row == DEFAULT_USERROW)

    def update_user_row(self):
        # build the new userrow string
        new_row_bin = ""
        for row in self.rows:
            new_row_bin = row.get_bin() + new_row_bin
        # convert to integer
        self.user_row = int(new_row_bin, 2)

    def get_user_row(self):
        return self.user_row

    def __str__(self):
        return "%x" % self.user_row

    def update_row(self, name, value):
        for row in self.rows:
            if row.name == name:
                row.update_value(value)
                break
        else:
            exit("no row found with name %s" % name)
        self.update_user_row()

    def update_with_hex(self, userrow):
        userrow_bin = bin(userrow).replace('0b','')
        assert(len(userrow_bin) == 64)
        for row in self.rows:
            row.update_from_hex(userrow_bin)
        self.update_user_row()

    def show(self):
        for row in self.rows:
            print(row)
        print("user_row hex %X" % self.user_row)
            
        
class UR(object):

    def __init__(self, name, bits, value):
        self.name = name
        self.bits = bits
        self.value = value 
    
    def get_bin(self):
        return "{:0{width}b}".format(self.value, width=self.bits[0]-self.bits[1]+1)
   
    def __str__(self):
        return "%10s : %X" % (self.name, self.value)

    def update_from_hex(self, userrow):
        self.slice = userrow[63-self.bits[0]:63-self.bits[1]+1]
        self.value = int(self.slice, 2)

    def update_value(self, value):
        self.value = value

if __name__ == "__main__":

    nvm = NVM()
    nvm.update_row("bootprot", 2) # 8192 bytes protected
    nvm.update_row("eeprom", 0) # 16384 bytes for eeprom
    nvm.show()
