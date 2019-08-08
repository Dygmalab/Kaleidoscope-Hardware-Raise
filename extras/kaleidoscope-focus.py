#! /usr/bin/env python
## kaleidoscope-focus -- Bidirectional communication plugin, host helper
## Copyright (C) 2017  Gergely Nagy
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

import serial
import sys
import time
import os
import readline
import atexit
import argparse

class Commander (object):
    def run (self):
        cmd = raw_input ("> ");

        if cmd == "quit" or cmd == "exit":
            sys.exit (0)

        if cmd == "":
            return

        print ""

        hadOutput = False
        with serial.Serial (args.port, 9600, timeout = 1) as ser:
            ser.write (cmd + "\n")
            while True:
                resultLine = ser.readline ()

                if resultLine == "\r\n" or resultLine == "\n":
                    resultLine = " "
                else:
                    resultLine = resultLine.rstrip ()

                if resultLine == ".":
                    break

                if resultLine:
                    hadOutput = True
                    print "<", resultLine

        if hadOutput:
            print ""

parser = argparse.ArgumentParser(description="talk to the Kaleidoscope keyboard via serial to Focus Plugin")
parser.add_argument('--port', default='/dev/ttyACM0')
args = parser.parse_args()

commander = Commander ()

histfile = os.path.join (os.path.expanduser ("~"), ".kaleidoscope-commander.hist")
try:
    readline.read_history_file (histfile)
except IOError:
    pass
atexit.register (readline.write_history_file, histfile)

while True:
    try:
        commander.run ()
    except EOFError:
        sys.exit (0)
    except Exception:
        print "WARNING: Connection to serial lost, sleeping 10s..."
        time.sleep (10)
        print "WARNING: Sleep over, resuming!"
        pass
