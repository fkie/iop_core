/***********           LICENSE HEADER   *******************************
JAUS Tool Set
Copyright (c)  2010, United States Government
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

       Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.

       Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in the 
documentation and/or other materials provided with the distribution.

       Neither the name of the United States Government nor the names of 
its contributors may be used to endorse or promote products derived from 
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE.
*********************  END OF LICENSE ***********************************/

package org.jts.codegenerator.support;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.List;
import java.util.TreeSet;
import org.jts.codegenerator.CodeGeneratorException;
import org.jts.codegenerator.CppCode;
import org.jts.codegenerator.Util;
import org.jts.jsidl.binding.*;


public class ROSPluginGenerator
{
	private StringBuffer sDefBuf = new StringBuffer();

	public ROSPluginGenerator()
	{
	}


	public void addServiceDef(ServiceDef sDef, String whiteListNamespaces)
	{
		String namespace = CppCode.makeNamespace(sDef.getId(), sDef.getVersion());
		if (!whiteListNamespaces.contains(namespace)) {
			return;
		}
		String serviceName = Util.upperCaseFirstLetter(sDef.getName()) + "Service";
		sDefBuf.append("\t<class name=\"" + Util.upperCaseFirstLetter(sDef.getName()) + "\"");
		sDefBuf.append(" ");
		sDefBuf.append("type=\"" + namespace + "::" + serviceName + "\"");
		sDefBuf.append(" ");
		sDefBuf.append("base_class_type=\"JTS::Service\"");
		sDefBuf.append(System.getProperty("line.separator"));
		sDefBuf.append("\t\t");
		sDefBuf.append("id=\"" + sDef.getId() + "\"");
		sDefBuf.append(" ");
		sDefBuf.append("version=\"" + sDef.getVersion() + "\"");
		sDefBuf.append(">").append(System.getProperty("line.separator"));
		sDefBuf.append("\t</class>").append(System.getProperty("line.separator"));
	}

	/**
	 * 
	 * @param outDir
	 * @param name
	 * @return
	 */
	public void generateXML(String outDir, String packageName)
	{
		String fileSep = System.getProperty("file.separator");
		File file = new File(outDir + fileSep + "plugin_iop.xml");
		StringBuffer buf = new StringBuffer();
		buf.append("<library path=\"" + packageName + "\">").append(System.getProperty("line.separator"));
		buf.append(sDefBuf.toString());
		buf.append("</library>").append(System.getProperty("line.separator"));
		try {
			if (file.createNewFile()) {
				FileWriter output = new FileWriter(file);
				output.write(buf.toString());
				output.close();
			}
		} catch (IOException e) {
			System.out.println("Exception while create ROS plugin file " + outDir + fileSep + "plugin_iop.xml :");
			e.printStackTrace();
		}
    	}
}
