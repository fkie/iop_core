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
from lxml import etree

NSMAP = {None : "urn:jaus:jsidl:1.0",
         "ns_auto" : "urn:jaus:jsidl:plus",
         "ns2" : "urn:jaus:jsidl:plus",
         "ns3" : "urn:jaus:jsidl:1.1"}

class RefDissolver(object):
  '''
  A helper class to replace declared_type_ref by referenced types because the 
  jaustoolset (v2.2.1) does not support this.
  '''
  def __init__(self):
    self.mset_root = etree.Element("{urn:jaus:jsidl:plus}message_set",
                                   nsmap=NSMAP,
                                   name="auto_message_set")
    self.included_message_sets = []
    self.const_types = {}

  def add_message_set(self, srcpath):
    messageset_path = srcpath
    if not os.path.isdir(messageset_path):
      messageset_path = os.path.dirname(srcfile)
      messageset_path = os.path.join(messageset_path, 'MessageSet')
    if os.path.exists(messageset_path):
      if messageset_path not in self.included_message_sets:
        print "JAUS: Include MessageSets from %s" % messageset_path
        delayed_files = []
        for messagefile in os.listdir(messageset_path):
          msfile = os.path.join(messageset_path, messagefile)
          message_def_tree = self._replace_constants(msfile)
          if message_def_tree is not None:
            self.mset_root.append(message_def_tree)
          else:
            delayed_files.append(msfile)
        for msfile in delayed_files:
          print "JAUS:   Include (second try) MessageSet file %s" % msfile
          message_def_tree = self._replace_constants(msfile)
          if message_def_tree is not None:
            self.mset_root.append(message_def_tree)
        self.included_message_sets.append(messageset_path)
    else:
        print "JAUS: MessageSet '%s' not exists, ignore!" % messageset_path

  def _replace_constants(self, msg_file):
    print "JAUS:   Include MessageSet file %s" % msg_file
    message_def_tree = etree.parse(msg_file).getroot()
    # the defined consts are in the separate file, only add to the knonwn list
    if message_def_tree.tag == '{urn:jaus:jsidl:1.0}declared_const_set':
      self._extract_const_set(message_def_tree)
    else: # search for const declarations, append to known and replace all used constants
      const_set_elements = message_def_tree.findall("./{urn:jaus:jsidl:1.0}declared_const_set")
      if const_set_elements is not None:
        for const_set_element in const_set_elements:
          const_id = self._extract_const_set(message_def_tree)
          if const_id is not None:
            try:
              message_def_tree.remove(const_set_element)
              string_el = etree.tostring(message_def_tree)
              for const_name, const_value in self.const_types[const_id].items():
                string_el = string_el.replace(const_name, const_value)
              message_def_tree = etree.fromstring(string_el)
            except Exception as e:
              print e
#              declared_const_set_ref
    const_ref_elements = message_def_tree.findall("./{urn:jaus:jsidl:1.0}declared_const_set_ref")
    if const_ref_elements is not None:
      for const_ref_element in const_ref_elements:
        ref_id = const_ref_element.attrib['id']
        if ref_id not in self.const_types:
          print "JAUS:       reference for constants not found: %s [%s]", ref_id, const_ref_element.attrib['name']
          return None
        else:
          # replace referenced constants
          try:
            string_el = etree.tostring(message_def_tree)
            for const_name, const_value in self.const_types[ref_id].items():
              print "JAUS:       search and replace constant '%s.%s' : '%s'"%(const_ref_element.attrib['name'], const_name, const_value)
              string_el = string_el.replace("%s.%s"%(const_ref_element.attrib['name'], const_name), const_value)
            message_def_tree = etree.fromstring(string_el)
          except Exception as e:
            print e
    return message_def_tree

  def _extract_const_set(self, message_def_tree):
    try:
      const_id = message_def_tree.attrib['id']
      self.const_types[const_id] = {}
      for child in list(message_def_tree):
        print "JAUS:       constant found:", child.tag, child.attrib['name'], child.attrib['const_value']
        self.const_types[const_id][child.attrib['name']] = child.attrib['const_value']
      return const_id
    except Exception as e:
      print e
    return None

  def _filter_by_id(self, declared_type_set_list, ref_id, ref_version):
    result = None
    if declared_type_set_list:
      if ref_id is None or ref_version is None:
        return declared_type_set_list[0]
      else:
        for i in range(len(declared_type_set_list)):
          item = declared_type_set_list[i]
          if 'id' in item.attrib and 'version' in item.attrib:
            if item.attrib['id'] == ref_id and item.attrib['version'] == ref_version:
              return item
    return None

  def _get_declared_type_set(self, xml_root, ref_name, ref_id=None, ref_version=None):
    declared_type_set = None
    search_attr = ref_name
    if ref_name or ref_id:
      search_attr = "[@name='%s']"%ref_name if ref_id is None else "[@id='%s']"%ref_id
    if not ref_name and xml_root.tag == '{urn:jaus:jsidl:1.0}declared_type_set':
      declared_type_set = xml_root
    if declared_type_set is None:
      declared_type_set = self._filter_by_id(xml_root.findall("./{urn:jaus:jsidl:1.0}declared_type_set%s"%search_attr), ref_id, ref_version)
      if declared_type_set is None:
        declared_type_set = self._filter_by_id(xml_root.findall("./{urn:jaus:jsidl:1.0}declared_type_set/{urn:jaus:jsidl:1.0}declared_type_set_ref%s"%search_attr), ref_id, ref_version)
        if declared_type_set is None:
          declared_type_set = self._filter_by_id(xml_root.findall(".{urn:jaus:jsidl:1.0}declared_type_set_ref%s"%search_attr), ref_id, ref_version)
          if declared_type_set is None:
            declared_type_set = self._filter_by_id(xml_root.findall(".{urn:jaus:jsidl:1.0}declared_type_set_ref%s"%search_attr), ref_id, ref_version)
    res_ref_id = None
    res_ref_version = None
    if declared_type_set is not None:
      try:
        res_ref_id = declared_type_set.attrib['id']
        res_ref_version = declared_type_set.attrib['version']
      except:
        pass
    if declared_type_set is None:
      raise Exception("Can not find reference '%s' in '%s' for %s"%(ref_name, xml_root.tag, xml_root.attrib['name']))
    return (declared_type_set, res_ref_id, res_ref_version)

  def _deref_type(self, declared_type, service_def_root):
    declared_type_path = declared_type.attrib['declared_type_ref'].split('.')
    if len(declared_type_path) < 2:
      # the declared_type_set is in this service definition, search for first occurrence of 'declared_type_set'
      (declared_type_set, ref_id, ref_version) = self._get_declared_type_set(service_def_root, '')
    else:
      (declared_type_set, ref_id, ref_version) = self._get_declared_type_set(service_def_root, declared_type_path[0])
    if declared_type_set.tag == '{urn:jaus:jsidl:1.0}declared_type_set_ref':
      (declared_type_set, ref_id, ref_version) = self._get_declared_type_set(self.mset_root, declared_type_path[0], ref_id, ref_version)
    for i in range(1,len(declared_type_path)-1):
      (declared_type_set, ref_id, ref_version) = self._get_declared_type_set(declared_type_set, declared_type_path[i])
      if declared_type_set.tag == '{urn:jaus:jsidl:1.0}declared_type_set_ref':
        (declared_type_set, ref_id, ref_version) = self._get_declared_type_set(self.mset_root, declared_type_path[i], ref_id, ref_version)
    derefed_element = declared_type_set.find("./*[@name='%s']"%declared_type_path[-1])
    if derefed_element is None:
      raise Exception("Cannot find '%s' referenced in <%s>%s"%(declared_type.attrib['declared_type_ref'], service_def_root.tag, service_def_root.attrib['name']))
    new_el = etree.fromstring(etree.tostring(derefed_element))
    # replace the attributes of referenced object
    for key in declared_type.keys():
      if not key.startswith('declared'):
        new_el.attrib[key] = declared_type.attrib[key]
    self.deref_children(new_el, declared_type_set)
    return new_el

  def deref_children(self, root, service_def_root):
    children = list(root)
    replace = {}
    for i in range(len(children)):
      child = children[i]
      if list(child):
        self.deref_children(child, service_def_root)
      else:
        if 'declared_type_ref' in child.attrib:
          print "JAUS:   disolve <%s>'%s' as %s"%(child.tag, child.attrib['declared_type_ref'], child.attrib['name'])
          replace[(child, i)] = self._deref_type(child, service_def_root)
    for (remove, pos), add in replace.items():
      root.remove(remove)
      root.insert(pos, add)

class ServiceSet(object):
  '''
  Creates a service_set (JAUS Component) which includes different services.
  '''

  def __init__(self, service_set_name, service_set_id, ref_dissolver=RefDissolver()):
    from lxml import etree
    self.root = etree.Element(etree.QName("urn:jaus:jsidl:plus", 'service_set'),
                              nsmap=NSMAP,
                              name="%s"%service_set_name,
                              id="%s"%service_set_id,
                              version="0.1")
    self.ref_dissolver = ref_dissolver
    description = etree.SubElement(self.root, "description")
    description.text = "By fkie_iop_builder auto-generated service set"

  def write(self, outfile):
    xmltree = etree.ElementTree(self.root)
    xml_declaration = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
    xmltree.write(outfile, xml_declaration=xml_declaration, method='xml')

  def append(self, service_def_file):
    '''
    Adds a service_def and dissolves all tags with 'declared_type_ref'
    attribute. The RefDissolver-class used for this purpose.
    '''
    print "JAUS: Including %s" % srcfile
    service_tree = etree.parse(service_def_file)
    # in case it is not done before
    self.ref_dissolver.add_message_set(service_def_file)
    self.ref_dissolver.deref_children(service_tree.getroot(), service_tree.getroot())
    self.root.append(service_tree.getroot())

outfile = argv[1]
service_set_name = argv[2]
service_set_id = argv[3]
ref_dissolver = RefDissolver()
service_set = ServiceSet(service_set_name, service_set_id, ref_dissolver)
for srcfile in argv[4:]:
  ref_dissolver.add_message_set(srcfile)
for srcfile in argv[4:]:
  if os.path.isfile(srcfile):
      service_set.append(srcfile)
service_set.write(outfile)
