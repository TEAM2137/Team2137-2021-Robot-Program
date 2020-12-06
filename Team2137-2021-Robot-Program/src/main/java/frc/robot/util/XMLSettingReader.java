package frc.robot.util;

import java.io.File;
import java.util.HashMap;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import frc.robot.Constants;
import frc.robot.Constants.MotorTypes;

public class XMLSettingReader {
    private File settingFile;
    private HashMap<String, String> settings;
    private HashMap<String, Motor> motors;

    public XMLSettingReader(String dir, String fileName) {
        this.settingFile = new File(dir + fileName);
        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();

        Document doc = null;

        try { 
            DocumentBuilder db = dbf.newDocumentBuilder();  
            doc = db.parse(this.settingFile);  
            doc.getDocumentElement().normalize();  
        } catch(Exception e) {
            e.printStackTrace();
        }

        NodeList list = doc.getChildNodes();
        
        for (int i = 0; i < list.getLength(); i++) {
            Node tmp = list.item(i);
            
            switch (tmp.getNodeName()) {
                case "Motor":
                    Motor mtmp = parseMotor(tmp);
                    this.motors.put(mtmp.getMotorName(), mtmp);
                    break;
                default:
                    this.settings.put(tmp.getNodeName(), tmp.getTextContent());
                    break;
            }
        }
    }
    
    public Motor parseMotor(Node stepNode) {
        Element element = (Element) stepNode;
        
        NodeList parmNodeList = element.getElementsByTagName("parm");
        String[] tmpParms = new String[parmNodeList.getLength()];

        for (int i = 0; i < parmNodeList.getLength(); i++) {
            Element id = (Element) parmNodeList.item(0);
            tmpParms[Integer.parseInt(id.getAttribute("id"))] = parmNodeList.item(i).getTextContent();
        }

        return new Motor(element.getElementsByTagName("Name").item(0).getTextContent(),
                        Integer.parseInt(element.getElementsByTagName("ID").item(0).getTextContent()),
                        MotorTypes.valueOf(element.getElementsByTagName("Type").item(0).getTextContent()),
                        Boolean.valueOf(element.getElementsByTagName("Inverted").item(0).getTextContent()),
                        tmpParms);
    }

    public Motor getMotor(String str) {
        return this.motors.get(str);
    }

    public String getSetting(String str) {
        if (this.settings.containsKey(str)) {
            return this.settings.get(str);
        } else {
            return "";
        }
    }

    public double getSetting(String str, double defaultVal) {
        return Double.parseDouble(getSetting(this.settings.get(str), String.valueOf(defaultVal)));
    }
    public String getSetting(String str, String defaultVal) {
        if (this.settings.containsKey(str)) {
            return this.settings.get(str);
        } else {
            return defaultVal;
        }
    }

    public XMLSettingReader(String fileName) {
        this(Constants.strSettingFileDirectory, fileName);
    }
}