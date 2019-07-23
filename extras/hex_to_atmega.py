#!/usr/bin/env python2.7

from collections import namedtuple
import os
dirname = os.path.dirname(__file__)

page_size = 64
frame_size = 16
memory_size = 8192
blank = 0xff
delay_ms = 1

template_file = os.path.join(dirname, '../src/Side-Flasher.cpp.template')
c_file = os.path.join(dirname, '../src/Side-Flasher.cpp')
hex_file = os.path.join(dirname, '../firmware-binaries/latest/side.hex')
template = open(template_file, 'r').read()
hex_in = open(hex_file, 'r').read()

Line = namedtuple('Line', ['size', 'offset', 'kind', 'data', 'checksum'])

def parse_line(line):
    """Parses an Intel HEX line into a Line tuple"""
    if len(line) < 11:
        exit("invalid Intel HEX line: %s" % line)
    if line[0] != ':':
        exit("invalid Intel HEX line: %s" % line)
    line = line[1:]
    try:
        int(line, 16)
    except ValueError:
        exit("invalid Intel HEX line: %s" % line)
    line = line.decode('hex')
    return Line(line[0], line[1:3], line[3], line[4:-1], line[-1])


hex_lines = [parse_line(l) for l in hex_in.strip().split()]
hex_lines = [l for l in hex_lines if l.kind == '\x00']

mem = [blank] * memory_size
offsets = []
data = []
for line in hex_lines:
    offset = (ord(line.offset[0]) << 8) + ord(line.offset[1])
    for i, x in enumerate(line.data):
        mem[offset + i] = ord(x)

# # if the first offset is not 0, then we need to write an additional 4 bytes for some reason
# for i in xrange(0, len(mem)):
#     if mem[i] != 0:
#         if i >= 4:
#             mem[i-4] = blank
#             mem[i-3] = blank
#             mem[i-2] = blank
#             mem[i-1] = blank
#         break
#
# scan memory
for i in xrange(0, len(mem), page_size):
    # can skip this page
    if all(x == blank for x in mem[i:i+page_size]):
        continue
    offsets.append(i)
    data.extend(mem[i:i+page_size])

offsets_text = ', '.join(str(x) for x in offsets)
data_text = ', '.join(hex(x) for x in data)

with open(c_file, 'w') as fh:
    fh.write(template % (page_size, frame_size, blank, len(offsets), len(data), delay_ms, offsets_text, data_text))
