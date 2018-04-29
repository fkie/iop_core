#!/usr/bin/python
import argparse
import os
import shutil
import re
from itertools import chain

parser = argparse.ArgumentParser()
parser.add_argument("jts_gui_dir")
parser.add_argument("output_dir")
parser.add_argument("install_prefix")
args = parser.parse_args()

path_jts_gui = os.path.abspath(args.jts_gui_dir)
path_jts_common = os.path.join(args.jts_gui_dir, "templates/Common")
path_jts_cpp_include = os.path.join(path_jts_common, "include")
path_jts_cpp_lib = os.path.join(path_jts_common, "lib")
path_jts_java_classes = os.path.join(path_jts_gui, "build/classes")
path_jts_java_lib = os.path.join(path_jts_gui, "lib")
path_jts_schemas = os.path.join(path_jts_gui, "resources/schema")
path_jts_templates = os.path.join(path_jts_gui, "templates")
path_jts_node_manager = os.path.join(path_jts_gui, "../nodeManager")

try:
    os.makedirs(args.output_dir)
except OSError:
    pass

jts_defs = ["-DTIXML_USE_STL"]
jts_libs = ["Common", "pthread", "rt"]

with open(os.path.join(args.output_dir, "jaustoolsetConfig.cmake"), "w") as config:
    config.write('''\
set(jaustoolset_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}")
get_filename_component(jaustoolset_INSTALL_PREFIX "${jaustoolset_INSTALL_PREFIX}" PATH)
get_filename_component(jaustoolset_INSTALL_PREFIX "${jaustoolset_INSTALL_PREFIX}" PATH)
get_filename_component(jaustoolset_INSTALL_PREFIX "${jaustoolset_INSTALL_PREFIX}" PATH)

set(jaustoolset_GUI_PATH "${jaustoolset_INSTALL_PREFIX}/share/jaustoolset")
set(jaustoolset_INCLUDE_DIRS "${jaustoolset_INSTALL_PREFIX}/include/jaustoolset")
set(jaustoolset_DEFINITIONS "%(defs)s")
set(jaustoolset_LIBRARIES "%(libs)s")
''' % {
    "defs": ' '.join(jts_defs),
    "libs": ';'.join(jts_libs)
})
    for lib in jts_libs:
        if os.path.isfile(os.path.join(path_jts_cpp_lib, "lib" + lib + ".so")):
            config.write('''
add_library(%(name)s SHARED IMPORTED)
set_property(TARGET %(name)s PROPERTY IMPORTED_LOCATION "${jaustoolset_INSTALL_PREFIX}/lib/lib%(name)s.so")
''' % {"name": lib})


with open(os.path.join(args.output_dir, "make_install.cmake"), "w") as installer:
    installer.write('set(_jts_include "%s")\n' % path_jts_cpp_include)
    installer.write('set(_jts_out "%s")\n' % path_jts_cpp_lib)
    installer.write('set(_jts_java_lib "%s")\n' % path_jts_java_lib)
    installer.write('set(_jts_java_classes "%s")\n' % path_jts_java_classes)
    installer.write('set(_jts_schemas "%s")\n' % path_jts_schemas)
    installer.write('set(_jts_templates "%s")\n' % path_jts_templates)
    installer.write('set(_jts_node_manager "%s")\n' % path_jts_node_manager)
    for lib in jts_libs:
        if os.path.isfile(os.path.join(path_jts_cpp_lib, "lib" + lib + ".so")):
            installer.write('file(INSTALL "${_jts_out}/lib%s.so" DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" USE_SOURCE_PERMISSIONS)\n' % lib)
    for root, dirs, files in os.walk(path_jts_cpp_include):
        h_files = [f for f in files if f.endswith(".h")]
        if h_files:
            installer.write('file(INSTALL ' + ' '.join(['"${_jts_include}/%s"' % os.path.relpath(os.path.join(root, h), path_jts_cpp_include) for h in h_files]) + ' DESTINATION "${CMAKE_INSTALL_PREFIX}/include/jaustoolset/%s")\n' % root[len(path_jts_cpp_include) + 1:])
    for jar in ["jargs-1.0/jargs.jar", "smc/Smc.jar", "smc/statemap.jar", "jaxb-plugins/commons-lang-2.5.jar"]:
        installer.write('file(INSTALL ' + '"${_jts_java_lib}/%s"' % jar + ' DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jaustoolset/lib/%s")\n' % os.path.dirname(jar))
    for root, dirs, files in os.walk(path_jts_java_classes):
        h_files = [f for f in files if f.endswith(".class")]
        if h_files:
            installer.write('file(INSTALL ' + ' '.join(['"${_jts_java_classes}/%s"' % os.path.relpath(os.path.join(root, h), path_jts_java_classes) for h in h_files]) + ' DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jaustoolset/%s")\n' % root[len(path_jts_gui) + 1:])
    for root, dirs, files in os.walk(path_jts_schemas):
        h_files = [f for f in files]
        if h_files:
            installer.write('file(INSTALL ' + ' '.join(['"${_jts_schemas}/%s"' % os.path.relpath(os.path.join(root, h), path_jts_schemas) for h in h_files]) + ' DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jaustoolset/%s")\n' % root[len(path_jts_gui) + 1:])
    for root, dirs, files in os.walk(path_jts_templates):
        h_files = [f for f in files]
        if h_files:
            installer.write('file(INSTALL ' + ' '.join(['"${_jts_templates}/%s"' % os.path.relpath(os.path.join(root, h), path_jts_templates) for h in h_files]) + ' DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jaustoolset/%s")\n' % root[len(path_jts_gui) + 1:])
    nm = os.path.join(path_jts_node_manager, "bin/NodeManager")
    if os.path.isfile(nm):
        installer.write('file(INSTALL "${_jts_node_manager}/bin/NodeManager" DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" USE_SOURCE_PERMISSIONS RENAME "JTSNodeManager")\n')
        installer.write('file(INSTALL "${_jts_node_manager}/bin/NodeManager" DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jaustoolset" USE_SOURCE_PERMISSIONS RENAME "JTSNodeManager")\n')
