package frc.robot.util;

import java.io.File;
import java.util.HashMap;
import java.util.function.Consumer;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import frc.robot.Constants;

public class XMLStepReader {
    private final File stepFile;
    private HashMap<Integer, Step> steps;
    private HashMap<String, String> settings;

    public XMLStepReader(String dir, String fileName) {
        this.stepFile = new File(dir + fileName);
        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();

        Document doc = null;

        try { 
            DocumentBuilder db = dbf.newDocumentBuilder();  
            doc = db.parse(this.stepFile);  
            doc.getDocumentElement().normalize();  
        } catch(Exception e) {
            e.printStackTrace();
        }

        NodeList tmpStep = doc.getElementsByTagName("Step");

        for (int i = 0; i < tmpStep.getLength(); i++) {
            this.steps.put(i, parseSteps(tmpStep.item(i)));
        }
    }

    public HashMap<Integer, Step> getSteps() {
        return this.steps;
    }

    public void forEachStep(Consumer<Step> stepConsumer) {
        this.steps.forEach((E, V) -> {
            stepConsumer.accept(V);
        });
    }

    public Step parseSteps(Node stepNode) {
        Element element = (Element) stepNode;
        NodeList parmNodeList = element.getElementsByTagName("parm");
        String[] tmpParms = new String[parmNodeList.getLength()];

        for (int i = 0; i < parmNodeList.getLength(); i++) {
            Element id = (Element) parmNodeList.item(0);
            tmpParms[Integer.parseInt(id.getAttribute("id"))] = parmNodeList.item(i).getTextContent();
        }

        return new Step(element.getElementsByTagName("command").item(0).getTextContent(),
                        element.getElementsByTagName("speed").item(0).getTextContent(),
                        element.getElementsByTagName("distance").item(0).getTextContent(),
                        element.getElementsByTagName("parallel").item(0).getTextContent(),
                        tmpParms);
    }

    public XMLStepReader(String fileName) {
        this(Constants.strStepFileDirectory, fileName);
    }
}