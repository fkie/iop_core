#!/usr/bin/python

# ROS/IOP Bridge
# Copyright (c) 2017 Fraunhofer
#
# This program is dual licensed; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# version 2 as published by the Free Software Foundation, or
# enter into a proprietary license agreement with the copyright
# holder.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; or you can read the full license at
# <http://www.gnu.de/documents/gpl-2.0.html>
#
# :author: Alexander Tiderko

from sys import argv
import os

outfile = argv[1]
source_dir = argv[2]
ignores = set(argv[3:])
outf = open(outfile, "w")
for path, dirs, files in os.walk(source_dir):
    cppfiles = [ f for f in files if f.endswith(".cpp") ]
    for f in cppfiles:
        fpath = os.path.join(path, f)
        fshort = fpath[len(source_dir)+1:]
        if fshort in ignores:
            continue
        print "JAUS: Packaging %s" % fshort
        inf = open(fpath, "r")
        content = inf.read()
        inf.close()
        outf.write("/* #### %s #### */" % fshort)
        outf.write (content)
outf.close()
