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

from sys import argv, stderr
import os
from shutil import copy2
import difflib

srcdir = argv[1]
destdir = argv[2]
error_msgs = []
#TODO: Added a proper merge function to print all errors
for hfile in argv[3:]:
    print "JAUS: Merge %s" % hfile
    srcfile = os.path.join(srcdir, hfile)
    dstfile = os.path.join(destdir, hfile)
    genfile = "%s.gen"%srcfile
    try:
        copy2 (dstfile, genfile)
    except Exception as e:
        print "JAUS: can't copy generated %s to %s because of error: %s" % (hfile, dstfile, e)
        genfile = "%s.gen"%dstfile
        copy2 (dstfile, genfile)
        print "JAUS:   you find this file in: %s.gen"%dstfile
    if os.path.isfile(dstfile):
      if dstfile.endswith('.h') and not os.path.isfile("%s.old"%dstfile) and not os.path.dirname(dstfile).endswith('include'):
        raise Exception("%s was not updated by code generator. Is the path correct?"%dstfile)
      a = file(genfile, 'rt').readlines()
      b = file("%s"%srcfile, 'rt').readlines()
      differ = difflib.Differ()
      diffsrcfile = "%s.diff"%srcfile
      try:
        fdiff = file(diffsrcfile, 'w')
      except Exception as e:
        print "JAUS: can't create diff %s because of error: %s" % (diffsrcfile, e)
        diffsrcfile = "%s.diff"%dstfile
        fdiff = file(diffsrcfile, 'w')
        print "JAUS:   you find this file in: %s"%diffsrcfile
      if fdiff is not None:
        fdiff.writelines(difflib.unified_diff(a, b, fromfile=genfile, tofile=diffsrcfile))
        fdiff.close()

#       diff = differ.compare(a, b)
#       result = []
#       removed_lines = []
#       last_removed_line = ''
#       compare2last = False
#       for line in diff:
#         if line:
#           print "line:", line
#           if line[0:2] == '? ':
#             if last_removed_line:
#               compare2last = True
#             pass
#           elif line[0:2] == '- ':
#             removed_content = line[2:].strip()
#             if removed_content and removed_content[0] not in ['/', '*', '#']:
#               removed_lines.append(removed_content)
#               last_removed_line = removed_content
#           elif line[0:2] == '+ ':
#             if compare2last:
#               # are tabs replaced by spaces?
#               if not last_removed_line.replace(line[2:].strip(), '').strip():
#                 removed_lines.pop()
#             result.append(line[2:])
#             removed_content = ''
#             compare2last = False
#           else:
#             result.append(line[2:])
#             removed_content = ''
#             compare2last = False
#       if removed_lines:
#         error_msg = "WARNING: outdated OVERRIDE file:\n"
#         error_msg += "  --> %s\n"%hfile
#         error_msg += "  override files remove the generated code by JAUS toolkit:\n---\n%s\n---\n"%'\n'.join(removed_lines)
#         error_msg += "  Update your code!\n"
#         error_msg += "You can see all changes by calling:\n"
#         error_msg += "  diff %s.gen %s\n"%(srcfile, srcfile)
#         if dstfile.endswith('.h'):
#           error_msgs.append(error_msg)
#         else:
#           print error_msg
      if dstfile.endswith('.h'):
          try:
              copy2 (srcfile, dstfile)
          except Exception as e:
              print "JAUS: can't copy %s to %s because of error: %s" % (srcfile, dstfile, e)
#              copy2 (dstfile, "%s.gen"%dstfile)
#              print "JAUS:   you find this file in: %s.gen"%dstfile
#        file(dstfile, 'w').writelines(result)
if error_msgs:
  raise Exception("%s"%'\n'.join(error_msgs))
