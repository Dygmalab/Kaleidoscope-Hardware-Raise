#!/usr/bin/env python2.7

import os
import sys
import subprocess
import re

dirname = os.path.dirname(__file__)
core_dir = os.path.abspath(os.path.join(dirname, "../.."))

# main firmware file commit is passed in from Makefile
firmware_commit = sys.argv[1]

# side commit is taken from the firmware binary in use
latest_binary_directory = os.path.realpath(os.path.join(dirname, '../firmware-binaries/latest'))
try:
    side_commit = re.search('Side-(.+)-SideBoot', latest_binary_directory).group(1)
except AttributeError:
    side_commit = 'n/a'

# core commit is taken from repo itself
core_commit = subprocess.check_output("git -C %s log --pretty=format:'%%h' -n1" % core_dir, shell=True)

# template file
template_file = os.path.join(dirname, '../src/Raise-Focus.cpp.template')
template = open(template_file, 'r').read()
c_file = os.path.join(dirname, '../src/Raise-Focus.cpp')

git_versions = "fw: %s core: %s side: %s" % (firmware_commit, core_commit, side_commit)
print(git_versions)
with open(c_file, 'w') as fh:
    fh.write(template % git_versions)

