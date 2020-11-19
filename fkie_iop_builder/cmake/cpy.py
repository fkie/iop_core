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
from shutil import copy2

destdir = argv[1]
for srcfile in argv[2:]:
    print("JAUS: Overriding %s" % srcfile)
    dstfile = os.path.join(destdir, srcfile)
    dstdir, dstname = os.path.split(dstfile)
    if not os.path.isdir(dstdir): os.makedirs(dstdir)
    copy2 (srcfile, dstfile)
