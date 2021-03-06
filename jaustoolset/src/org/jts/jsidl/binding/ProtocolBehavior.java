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
import javax.xml.bind.annotation.XmlAttribute;
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
 *         &lt;element ref="{urn:jaus:jsidl:1.0}start" maxOccurs="unbounded"/>
 *         &lt;element ref="{urn:jaus:jsidl:1.0}state_machine" maxOccurs="unbounded"/>
 *       &lt;/sequence>
 *       &lt;attribute name="is_stateless" type="{http://www.w3.org/2001/XMLSchema}boolean" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "", propOrder = {
    "start",
    "stateMachine"
})
@XmlRootElement(name = "protocol_behavior")
public class ProtocolBehavior {

    @XmlElement(required = true)
    protected List<Start> start;
    @XmlElement(name = "state_machine", required = true)
    protected List<StateMachine> stateMachine;
    @XmlAttribute(name = "is_stateless")
    protected Boolean isStateless;

    /**
     * Gets the value of the start property.
     * 
     * <p>
     * This accessor method returns a reference to the live list,
     * not a snapshot. Therefore any modification you make to the
     * returned list will be present inside the JAXB object.
     * This is why there is not a <CODE>set</CODE> method for the start property.
     * 
     * <p>
     * For example, to add a new item, do as follows:
     * <pre>
     *    getStart().add(newItem);
     * </pre>
     * 
     * 
     * <p>
     * Objects of the following type(s) are allowed in the list
     * {@link Start }
     * 
     * 
     */
    public List<Start> getStart() {
        if (start == null) {
            start = new ArrayList<Start>();
        }
        return this.start;
    }

    /**
     * Gets the value of the stateMachine property.
     * 
     * <p>
     * This accessor method returns a reference to the live list,
     * not a snapshot. Therefore any modification you make to the
     * returned list will be present inside the JAXB object.
     * This is why there is not a <CODE>set</CODE> method for the stateMachine property.
     * 
     * <p>
     * For example, to add a new item, do as follows:
     * <pre>
     *    getStateMachine().add(newItem);
     * </pre>
     * 
     * 
     * <p>
     * Objects of the following type(s) are allowed in the list
     * {@link StateMachine }
     * 
     * 
     */
    public List<StateMachine> getStateMachine() {
        if (stateMachine == null) {
            stateMachine = new ArrayList<StateMachine>();
        }
        return this.stateMachine;
    }

    /**
     * Gets the value of the isStateless property.
     * 
     * @return
     *     possible object is
     *     {@link Boolean }
     *     
     */
    public Boolean isIsStateless() {
        return isStateless;
    }

    /**
     * Sets the value of the isStateless property.
     * 
     * @param value
     *     allowed object is
     *     {@link Boolean }
     *     
     */
    public void setIsStateless(Boolean value) {
        this.isStateless = value;
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
