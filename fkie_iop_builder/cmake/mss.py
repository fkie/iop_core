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
from sys import version_info
import os
from typing import Dict, List, Optional, Set, Tuple

from lxml import etree
from lxml.etree import _Element, _ElementTree

# Default namespace map used for all generated XML elements
NSMAP: Dict[Optional[str], str] = {
    None: "urn:jaus:jsidl:1.1",
    "plus": "urn:jaus:jsidl:plus"
}

# Commonly used namespace prefixes for tag matching
NS_JSIDL: str = "{urn:jaus:jsidl:1.1}"
NS_PLUS: str = "{urn:jaus:jsidl:plus}"


class RefDissolver(object):
    """
    Helper class to replace 'declared_type_ref' attributes with the actual
    referenced type definitions. This is necessary because jaustoolset (v2.2.1)
    does not support declared_type_ref resolution natively.

    The dissolver loads message set files, registers service definitions,
    resolves constant references, and provides methods to inline all declared
    type references within a service definition tree.
    """

    def __init__(self) -> None:
        # Root element that aggregates all loaded message set definitions
        self.mset_root: _Element = etree.Element(
            f"{NS_PLUS}message_set",
            nsmap=NSMAP,
            name="auto_message_set"
        )
        # Tracks which message set directories have already been included
        self.included_message_sets: List[str] = []
        # Maps constant set IDs to their name->value dictionaries
        self.const_types: Dict[str, Dict[str, str]] = {}
        # Registry of loaded service definitions keyed by service name
        self.service_defs: Dict[str, _Element] = {}
        # Registry of loaded service definitions keyed by service id
        self.service_defs_by_id: Dict[str, _Element] = {}
        # Tracks which directories have already been scanned for service defs
        self.scanned_dirs: Set[str] = set()
        # Tracks references to prevent circular reference
        self._resolving: Set[str] = set()

    def register_service_def(self, service_def_file: str) -> None:
        """
        Parses and registers a service definition file so that its inline
        message definitions can be found when resolving client_of references.
        Also scans the same directory for other service_def XML files to
        handle cross-references between services in the same package.

        Args:
            service_def_file: Path to the service definition XML file.
        """
        if not os.path.isfile(service_def_file):
            return
        self._register_single_service_def(service_def_file)
        dir_path: str = os.path.dirname(os.path.abspath(service_def_file))
        self._scan_directory_for_service_defs(dir_path)

    def _register_single_service_def(self, service_def_file: str) -> None:
        """
        Parses a single file and registers it by both name and id
        if it is a service_def.

        Args:
            service_def_file: Path to the XML file.
        """
        if not os.path.isfile(service_def_file):
            return
        try:
            tree: _Element = etree.parse(service_def_file).getroot()
            if tree.tag == f'{NS_JSIDL}service_def':
                name: str = tree.attrib.get('name', '')
                svc_id: str = tree.attrib.get('id', '')
                if name and name not in self.service_defs:
                    print(f"JAUS: Register service_def '{name}' (id='{svc_id}') from {service_def_file}")
                    self.service_defs[name] = tree
                if svc_id and svc_id not in self.service_defs_by_id:
                    self.service_defs_by_id[svc_id] = tree
        except Exception as e:
            print(f"JAUS: Warning: Could not parse '{service_def_file}': {e}")

    def _scan_directory_for_service_defs(self, dir_path: str) -> None:
        """
        Scans a directory for XML files containing service definitions and
        registers them. This ensures that cross-referenced services (e.g.
        via client_of) in the same directory are available for resolution.

        Args:
            dir_path: Directory path to scan.
        """
        if not dir_path or not os.path.isdir(dir_path):
            return
        if dir_path in self.scanned_dirs:
            return
        self.scanned_dirs.add(dir_path)
        for filename in os.listdir(dir_path):
            if filename.endswith('.xml'):
                full_path: str = os.path.join(dir_path, filename)
                if os.path.isfile(full_path):
                    self._register_single_service_def(full_path)

    def add_message_set(self, srcpath: str) -> None:
        """
        Loads all message definition files from the 'MessageSet' directory
        adjacent to the given source path. Files that fail on first pass
        (due to unresolved constant references) are retried in a second pass.

        Args:
            srcpath: Path to a service definition file or a MessageSet directory.
        """
        messageset_path: str = srcpath
        if not os.path.isdir(messageset_path):
            messageset_path = os.path.dirname(srcpath)
            messageset_path = os.path.join(messageset_path, 'MessageSet')

        if os.path.exists(messageset_path):
            if messageset_path not in self.included_message_sets:
                print(f"JAUS: Include MessageSets from {messageset_path}")
                delayed_files: List[str] = []

                for messagefile in os.listdir(messageset_path):
                    msfile: str = os.path.join(messageset_path, messagefile)
                    message_def_tree: Optional[_Element] = self._replace_constants(msfile)
                    if message_def_tree is not None:
                        self.mset_root.append(message_def_tree)
                    else:
                        delayed_files.append(msfile)

                # Second pass for files that had unresolved constant refs
                for msfile in delayed_files:
                    print(f"JAUS:   Include (second try) MessageSet file {msfile}")
                    message_def_tree = self._replace_constants(msfile)
                    if message_def_tree is not None:
                        self.mset_root.append(message_def_tree)

                self.included_message_sets.append(messageset_path)
        else:
            print(f"JAUS: MessageSet '{messageset_path}' not exists, ignore!")

    def _replace_constants(self, msg_file: str) -> Optional[_Element]:
        """
        Parses a single message set file, extracts any declared constants,
        and replaces constant references within the file with their values.

        Args:
            msg_file: Path to the message definition XML file.

        Returns:
            The processed XML element tree root, or None if a required
            constant reference could not be resolved yet.
        """
        print(f"JAUS:   Include MessageSet file {msg_file}")
        message_def_tree: _Element = etree.parse(msg_file).getroot()

        # If the file is purely a declared_const_set, just extract and store it
        if message_def_tree.tag == f'{NS_JSIDL}declared_const_set':
            self._extract_const_set(message_def_tree)
            return None

        # Extract inline const sets and replace their usages in the tree
        const_set_elements: List[_Element] = message_def_tree.findall(
            f"./{NS_JSIDL}declared_const_set"
        )
        if const_set_elements is not None:
            for const_set_element in const_set_elements:
                const_id: Optional[str] = self._extract_const_set(message_def_tree)
                if const_id is not None:
                    try:
                        message_def_tree.remove(const_set_element)
                        string_el: str = etree.tostring(message_def_tree).decode() \
                            if version_info[0] > 2 \
                            else etree.tostring(message_def_tree)
                        for const_name, const_value in self.const_types[const_id].items():
                            string_el = string_el.replace(const_name, const_value)
                        message_def_tree = etree.fromstring(string_el)
                    except Exception as e:
                        print(e)

        # Resolve declared_const_set_ref elements (references to external const sets)
        const_ref_elements: List[_Element] = message_def_tree.findall(
            f"./{NS_JSIDL}declared_const_set_ref"
        )
        if const_ref_elements is not None:
            for const_ref_element in const_ref_elements:
                ref_id: str = const_ref_element.attrib['id']
                if ref_id not in self.const_types:
                    print(
                        f"JAUS:       reference for constants not found: "
                        f"{ref_id} [{const_ref_element.attrib['name']}]"
                    )
                    return None
                else:
                    # Replace all occurrences of "refname.constname" with the value
                    try:
                        string_el = etree.tostring(message_def_tree).decode() \
                            if version_info[0] > 2 \
                            else etree.tostring(message_def_tree)
                        for const_name, const_value in self.const_types[ref_id].items():
                            print(
                                f"JAUS:       search and replace constant "
                                f"'{const_ref_element.attrib['name']}.{const_name}' : '{const_value}'"
                            )
                            string_el = string_el.replace(
                                f"{const_ref_element.attrib['name']}.{const_name}",
                                const_value
                            )
                        message_def_tree = etree.fromstring(string_el)
                    except Exception as e:
                        import traceback
                        print(traceback.format_exc())

        return message_def_tree

    def _extract_const_set(self, message_def_tree: _Element) -> Optional[str]:
        """
        Extracts constant definitions from a declared_const_set element
        and stores them in self.const_types keyed by the set's ID.

        Args:
            message_def_tree: The XML element containing declared constants.

        Returns:
            The ID of the extracted constant set, or None on failure.
        """
        try:
            const_id: str = message_def_tree.attrib['id']
            self.const_types[const_id] = {}
            for child in list(message_def_tree):
                print(
                    f"JAUS:       constant found: {child.tag} "
                    f"{child.attrib['name']} {child.attrib['const_value']}"
                )
                self.const_types[const_id][child.attrib['name']] = child.attrib['const_value']
            return const_id
        except Exception as e:
            print(e)
        return None

    def _filter_by_id(
        self,
        declared_type_set_list: List[_Element],
        ref_id: Optional[str],
        ref_version: Optional[str]
    ) -> Optional[_Element]:
        """
        Filters a list of XML elements to find one matching the given ID and version.
        If ref_id or ref_version is None, returns the first element if available.

        Args:
            declared_type_set_list: List of candidate elements.
            ref_id: Expected 'id' attribute value.
            ref_version: Expected 'version' attribute value.

        Returns:
            The matching element, or None if not found.
        """
        result: Optional[_Element] = None
        if declared_type_set_list:
            if ref_id is None or ref_version is None:
                return declared_type_set_list[0]
            else:
                for item in declared_type_set_list:
                    if 'id' in item.attrib and 'version' in item.attrib:
                        if item.attrib['id'] == ref_id and item.attrib['version'] == ref_version:
                            return item
        return result

    def _get_declared_type_set(
        self,
        xml_root: _Element,
        ref_name: str,
        ref_id: Optional[str] = None,
        ref_version: Optional[str] = None
    ) -> Tuple[_Element, Optional[str], Optional[str]]:
        """
        Searches for a declared type set (or reference) within the given XML root
        element. The search proceeds through multiple locations:
          1. Direct declared_type_set children
          2. declared_type_set_ref inside declared_type_set
          3. Top-level declared_type_set_ref
          4. references/client_of elements
          5. message_set/input_set/declared_message_def elements
          6. message_set/output_set/declared_message_def elements

        Args:
            xml_root: The XML element to search within.
            ref_name: The name attribute to match.
            ref_id: Optional ID to filter by.
            ref_version: Optional version to filter by.

        Returns:
            Tuple of (found_element, resolved_id, resolved_version).

        Raises:
            Exception: If no matching reference can be found.
        """
        declared_type_set: Optional[_Element] = None
        search_attr: str = ref_name

        if ref_name or ref_id:
            search_attr = (
                f"[@name='{ref_name}']" if ref_id is None
                else f"[@id='{ref_id}']"
            )

        # If ref_name is empty and root is already a declared_type_set, use it directly
        if not ref_name and xml_root.tag == f'{NS_JSIDL}declared_type_set':
            declared_type_set = xml_root

        # Search: direct declared_type_set child
        if declared_type_set is None:
            declared_type_set = self._filter_by_id(
                xml_root.findall(f"./{NS_JSIDL}declared_type_set{search_attr}"),
                ref_id, ref_version
            )

        # Search: declared_type_set_ref inside a declared_type_set
        if declared_type_set is None:
            declared_type_set = self._filter_by_id(
                xml_root.findall(
                    f"./{NS_JSIDL}declared_type_set/{NS_JSIDL}declared_type_set_ref{search_attr}"
                ),
                ref_id, ref_version
            )

        # Search: top-level declared_type_set_ref
        if declared_type_set is None:
            declared_type_set = self._filter_by_id(
                xml_root.findall(f"./{NS_JSIDL}declared_type_set_ref{search_attr}"),
                ref_id, ref_version
            )

        # Search: references/client_of (for services that reference another service)
        if declared_type_set is None:
            declared_type_set = self._filter_by_id(
                xml_root.findall(
                    f"./{NS_JSIDL}references/{NS_JSIDL}client_of{search_attr}"
                ),
                ref_id, ref_version
            )

        # Search: message_set/input_set/declared_message_def
        if declared_type_set is None:
            declared_type_set = self._filter_by_id(
                xml_root.findall(
                    f"./{NS_JSIDL}message_set/{NS_JSIDL}input_set/"
                    f"{NS_JSIDL}declared_message_def{search_attr}"
                ),
                ref_id, ref_version
            )

        # Search: message_set/output_set/declared_message_def
        if declared_type_set is None:
            declared_type_set = self._filter_by_id(
                xml_root.findall(
                    f"./{NS_JSIDL}message_set/{NS_JSIDL}output_set/"
                    f"{NS_JSIDL}declared_message_def{search_attr}"
                ),
                ref_id, ref_version
            )

        # Extract id and version from found element
        res_ref_id: Optional[str] = None
        res_ref_version: Optional[str] = None
        if declared_type_set is not None:
            try:
                res_ref_id = declared_type_set.attrib['id']
                res_ref_version = declared_type_set.attrib['version']
            except KeyError:
                pass

        if declared_type_set is None:
            raise Exception(
                f"Can not find reference '{ref_name}' in "
                f"'{xml_root.tag}' for {xml_root.attrib['name']}"
            )

        return (declared_type_set, res_ref_id, res_ref_version)

    def _find_service_def_by_id(self, svc_id: str) -> Optional[_Element]:
        """
        Finds a registered service definition by its id attribute.

        Args:
            svc_id: The service ID (e.g. 'urn:jaus:jss:iop:PathReporter').

        Returns:
            The service_def element or None.
        """
        return self.service_defs_by_id.get(svc_id)

    def _find_message_in_service_def(
        self,
        service_id: str,
        target_name: str,
        declared_type: _Element
    ) -> Optional[_Element]:
        """
        Searches a registered service definition (looked up by id) for a
        message_def or declared_message_def by name in both input_set and
        output_set. If a declared_message_def is found, it is recursively
        resolved.

        Args:
            service_id: The id of the service definition to search
                        (e.g. 'urn:jaus:jss:iop:PathReporter').
            target_name: Name of the message to find.
            declared_type: The original declaring element (for attribute merging).

        Returns:
            The resolved element, or None if not found.
        """
        svc_root: Optional[_Element] = self._find_service_def_by_id(service_id)
        if svc_root is None:
            return None

        # Search for message_def in input_set and output_set
        for set_tag in ('input_set', 'output_set'):
            derefed_element: Optional[_Element] = svc_root.find(
                f"./{NS_JSIDL}message_set/{NS_JSIDL}{set_tag}/"
                f"{NS_JSIDL}message_def[@name='{target_name}']"
            )
            if derefed_element is not None:
                new_el: _Element = etree.fromstring(etree.tostring(derefed_element))
                for key in declared_type.keys():
                    if not key.startswith('declared'):
                        new_el.attrib[key] = declared_type.attrib[key]
                self.deref_children(new_el, svc_root)
                return new_el

        # Search for declared_message_def in input_set and output_set
        # (the referenced service itself may use declared refs that need resolving)
        for set_tag in ('input_set', 'output_set'):
            derefed_element = svc_root.find(
                f"./{NS_JSIDL}message_set/{NS_JSIDL}{set_tag}/"
                f"{NS_JSIDL}declared_message_def[@name='{target_name}']"
            )
            if derefed_element is not None:
                return self._deref_type(derefed_element, svc_root)

        return None

    def _find_message_in_mset_root(
        self,
        target_name: str,
        declared_type: _Element
    ) -> Optional[_Element]:
        """
        Searches all loaded message set files for a message definition by name.

        Args:
            target_name: Name of the message to find.
            declared_type: The original declaring element (for attribute merging).

        Returns:
            The resolved element, or None if not found.
        """
        for mset_child in self.mset_root:
            derefed_element: Optional[_Element] = mset_child.find(
                f"./*[@name='{target_name}']"
            )
            if derefed_element is None:
                derefed_element = mset_child.find(f".//*[@name='{target_name}']")
            if derefed_element is not None:
                new_el: _Element = etree.fromstring(etree.tostring(derefed_element))
                for key in declared_type.keys():
                    if not key.startswith('declared'):
                        new_el.attrib[key] = declared_type.attrib[key]
                self.deref_children(new_el, mset_child)
                return new_el
        return None

    def _deref_type(self, declared_type: _Element, service_def_root: _Element) -> _Element:
        """
        Resolves a single declared_type_ref to the actual type definition.

        The declared_type_ref attribute uses dot-notation (e.g.
        'pathReporter.ReportPathReporterCapabilities'). This method walks
        the path segments to locate the referenced element, handling:
        - client_of: resolved by looking up the service by its 'id' attribute
        - declared_type_set_ref: resolved via mset_root by ID

        Args:
            declared_type: The element containing the 'declared_type_ref' attribute.
            service_def_root: The root of the service definition being processed.

        Returns:
            A deep copy of the resolved element with attributes merged from
            the declaring element.

        Raises:
            Exception: If the referenced type cannot be found or a circular
                       reference is detected.
        """
        declared_type_path: List[str] = declared_type.attrib['declared_type_ref'].split('.')

        # Guard against circular references to prevent infinite recursion
        ref_key: str = declared_type.attrib['declared_type_ref']
        svc_name_ctx: str = service_def_root.attrib.get('name', '')
        resolution_key: str = f"{svc_name_ctx}::{ref_key}"
        if resolution_key in self._resolving:
            raise Exception(
                f"Circular reference detected while resolving '{ref_key}' "
                f"in service '{svc_name_ctx}'. Resolution chain: {self._resolving}"
            )
        self._resolving.add(resolution_key)

        try:
            # Determine the starting type set based on path length
            if len(declared_type_path) < 2:
                # No dot prefix: use the first declared_type_set found in the service def
                (declared_type_set, ref_id, ref_version) = self._get_declared_type_set(
                    service_def_root, ''
                )
            else:
                # First segment identifies the type set or client_of reference
                (declared_type_set, ref_id, ref_version) = self._get_declared_type_set(
                    service_def_root, declared_type_path[0]
                )

            # Handle client_of references: use the 'id' attribute to look up
            # the referenced service definition directly, avoiding name mismatches
            if declared_type_set.tag == f'{NS_JSIDL}client_of':
                target_name: str = declared_type_path[-1]
                client_of_id: str = declared_type_set.attrib.get('id', '')

                # Primary lookup: find the service definition by its unique id
                if client_of_id:
                    result: Optional[_Element] = self._find_message_in_service_def(
                        client_of_id, target_name, declared_type
                    )
                    if result is not None:
                        return result

                # Fallback: search in all loaded message set files
                result = self._find_message_in_mset_root(target_name, declared_type)
                if result is not None:
                    return result

                raise Exception(
                    f"Cannot find '{declared_type.attrib['declared_type_ref']}' "
                    f"in any loaded message set or service def "
                    f"(via client_of id='{client_of_id}'). "
                    f"Registered service_defs by id: {list(self.service_defs_by_id.keys())}"
                )

            # Handle declared_type_set_ref: resolve via the aggregated message set root
            if declared_type_set.tag == f'{NS_JSIDL}declared_type_set_ref':
                (declared_type_set, ref_id, ref_version) = self._get_declared_type_set(
                    self.mset_root, declared_type_path[0], ref_id, ref_version
                )

            # Walk intermediate path segments (for nested type set references)
            for i in range(1, len(declared_type_path) - 1):
                (declared_type_set, ref_id, ref_version) = self._get_declared_type_set(
                    declared_type_set, declared_type_path[i]
                )
                if declared_type_set.tag == f'{NS_JSIDL}declared_type_set_ref':
                    (declared_type_set, ref_id, ref_version) = self._get_declared_type_set(
                        self.mset_root, declared_type_path[i], ref_id, ref_version
                    )

            # Find the final target element by name in the resolved type set
            derefed_element: Optional[_Element] = declared_type_set.find(
                f"./*[@name='{declared_type_path[-1]}']"
            )
            if derefed_element is None:
                raise Exception(
                    f"Cannot find '{declared_type.attrib['declared_type_ref']}' "
                    f"referenced in <{service_def_root.tag}>"
                    f"{service_def_root.attrib['name']} ::: {declared_type_set}"
                )

            # Create a deep copy and merge non-declared attributes from the referencing element
            new_el: _Element = etree.fromstring(etree.tostring(derefed_element))
            for key in declared_type.keys():
                if not key.startswith('declared'):
                    new_el.attrib[key] = declared_type.attrib[key]

            # Recursively resolve any nested declared_type_refs in the resolved element
            self.deref_children(new_el, declared_type_set)
            return new_el

        finally:
            # Always remove the resolution key to allow the same ref to be resolved
            # in a different context (non-circular usage)
            self._resolving.discard(resolution_key)

    def deref_children(self, root: _Element, service_def_root: _Element) -> None:
        """
        Recursively traverses all children of the given root element and replaces
        any element that has a 'declared_type_ref' attribute with the resolved
        actual type definition.

        Args:
            root: The parent element whose children should be processed.
            service_def_root: The service definition root used for resolving references.
        """
        children: List[_Element] = list(root)
        replace: Dict[Tuple[_Element, int], _Element] = {}

        for i in range(len(children)):
            child: _Element = children[i]
            if list(child):
                # Recurse into children that have their own sub-elements
                self.deref_children(child, service_def_root)
            else:
                if 'declared_type_ref' in child.attrib:
                    print(
                        f"JAUS:   dissolve <{child.tag}>"
                        f"'{child.attrib['declared_type_ref']}' "
                        f"as {child.attrib['name']}"
                    )
                    replace[(child, i)] = self._deref_type(child, service_def_root)

        # Perform replacements in-place, preserving element order
        for (remove, pos), add in replace.items():
            root.remove(remove)
            root.insert(pos, add)


class ServiceSet(object):
    """
    Creates a service_set (JAUS Component) which includes different services.
    Each service definition is added via append(), which triggers full
    declared_type_ref resolution before inclusion.
    """

    def __init__(
        self,
        service_set_name: str,
        service_set_id: str,
        ref_dissolver: RefDissolver = RefDissolver()
    ) -> None:
        """
        Args:
            service_set_name: Human-readable name for the service set.
            service_set_id: Unique identifier for the service set.
            ref_dissolver: RefDissolver instance used for type resolution.
        """
        self.root: _Element = etree.Element(
            etree.QName("urn:jaus:jsidl:plus", 'service_set'),
            nsmap=NSMAP,
            name=f"{service_set_name}",
            id=f"{service_set_id}",
            version="0.1"
        )
        self.ref_dissolver: RefDissolver = ref_dissolver
        description: _Element = etree.SubElement(self.root, "description")
        description.text = "By fkie_iop_builder auto-generated service set"

    def write(self, outfile: str) -> None:
        """
        Serializes the service set XML tree to a file.

        Args:
            outfile: Output file path.
        """
        xmltree: _ElementTree = etree.ElementTree(self.root)
        xml_declaration: str = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        xmltree.write(
            outfile,
            xml_declaration=xml_declaration,
            method='xml',
            encoding='UTF-8',
            pretty_print=True
        )

    def append(self, service_def_file: str) -> None:
        """
        Adds a service definition to this service set. All declared_type_ref
        attributes within the service definition are resolved (inlined) using
        the RefDissolver before appending.

        Args:
            service_def_file: Path to the service definition XML file.
        """
        print(f"JAUS: Including {service_def_file}")
        parser: etree.XMLParser = etree.XMLParser(remove_blank_text=True)
        service_tree: _ElementTree = etree.parse(service_def_file, parser)

        # Ensure message sets are loaded and service def is registered
        self.ref_dissolver.add_message_set(service_def_file)
        self.ref_dissolver.register_service_def(service_def_file)

        # Resolve all declared_type_ref attributes recursively
        self.ref_dissolver.deref_children(
            service_tree.getroot(), service_tree.getroot()
        )

        self.root.append(service_tree.getroot())


# --- Main entry point ---
if __name__ == "__main__" or True:
    outfile: str = argv[1]
    service_set_name: str = argv[2]
    service_set_id: str = '0'

    ref_dissolver: RefDissolver = RefDissolver()
    service_set: ServiceSet = ServiceSet(service_set_name, service_set_id, ref_dissolver)

    # First pass: load all message sets and register service definitions
    for srcfile in argv[3:]:
        ref_dissolver.add_message_set(srcfile)
        ref_dissolver.register_service_def(srcfile)

    # Second pass: include actual service definition files
    for srcfile in argv[3:]:
        if os.path.isfile(srcfile):
            service_set.append(srcfile)

    service_set.write(outfile)
