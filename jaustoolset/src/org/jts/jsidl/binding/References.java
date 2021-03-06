//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, vhudson-jaxb-ri-2.2-147 
// See <a href="http://java.sun.com/xml/jaxb">http://java.sun.com/xml/jaxb</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2018.12.21 at 10:07:33 AM CET 
//


package org.jts.jsidl.binding;

import java.util.ArrayList;
import java.util.List;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlType;
import org.apache.commons.lang.builder.EqualsBuilder;
import org.apache.commons.lang.builder.HashCodeBuilder;
import org.apache.commons.lang.builder.ToStringBuilder;
import org.apache.commons.lang.builder.ToStringStyle;


/**
 * <p>Java class for anonymous complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType>
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;sequence>
 *         &lt;element ref="{urn:jaus:jsidl:1.0}inherits_from" minOccurs="0"/>
 *         &lt;element ref="{urn:jaus:jsidl:1.0}client_of" maxOccurs="unbounded" minOccurs="0"/>
 *       &lt;/sequence>
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "", propOrder = {
    "inheritsFrom",
    "clientOf"
})
@XmlRootElement(name = "references")
public class References {

    @XmlElement(name = "inherits_from")
    protected InheritsFrom inheritsFrom;
    @XmlElement(name = "client_of")
    protected List<ClientOf> clientOf;

    /**
     * Gets the value of the inheritsFrom property.
     * 
     * @return
     *     possible object is
     *     {@link InheritsFrom }
     *     
     */
    public InheritsFrom getInheritsFrom() {
        return inheritsFrom;
    }

    /**
     * Sets the value of the inheritsFrom property.
     * 
     * @param value
     *     allowed object is
     *     {@link InheritsFrom }
     *     
     */
    public void setInheritsFrom(InheritsFrom value) {
        this.inheritsFrom = value;
    }

    /**
     * Gets the value of the clientOf property.
     * 
     * <p>
     * This accessor method returns a reference to the live list,
     * not a snapshot. Therefore any modification you make to the
     * returned list will be present inside the JAXB object.
     * This is why there is not a <CODE>set</CODE> method for the clientOf property.
     * 
     * <p>
     * For example, to add a new item, do as follows:
     * <pre>
     *    getClientOf().add(newItem);
     * </pre>
     * 
     * 
     * <p>
     * Objects of the following type(s) are allowed in the list
     * {@link ClientOf }
     * 
     * 
     */
    public List<ClientOf> getClientOf() {
        if (clientOf == null) {
            clientOf = new ArrayList<ClientOf>();
        }
        return this.clientOf;
    }

    @Override
    public String toString() {
        return ToStringBuilder.reflectionToString(this, ToStringStyle.MULTI_LINE_STYLE);
    }

    @Override
    public boolean equals(Object that) {
        String[] ignore={"interpretation"};
		return EqualsBuilder.reflectionEquals(this, that, ignore);
    }

    @Override
    public int hashCode() {
        return HashCodeBuilder.reflectionHashCode(this);
    }

}
