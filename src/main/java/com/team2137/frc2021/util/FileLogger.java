package com.team2137.frc2021.util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FileLogger {
    public enum EventType {
        Debug ("Debug: "),
        Error ("ERROR: "),
        Warning ("Warning: "),
        Status ("Status: ");

        String name = "";

        EventType(String string) {
            this.name = string;    
        }

        public String toString() {
            return this.name;
        }
    }

    private final int debug;
    private FileWriter writer;
    private DriverStation dStation;
    private final String logName;

    public FileLogger(int _debug){
        this.debug = _debug;
        cleanLogs();

        DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");
        LocalDateTime now = LocalDateTime.now();

        this.logName = "log_" + dtf.format(now);

//        try {
//			this.writer = new FileWriter(Constants.strLogDirectory + this.logName);
//		} catch (IOException e) {
//			e.printStackTrace();
//		}
    }

    public FileLogger(int _debug, DriverStation ds) {
        this(_debug);
        this.dStation = ds;
    }

    public synchronized void writeEventDS(int _debug, EventType eventType, String message) {
        writeEvent(_debug, eventType, message);
        if (this.dStation != null) DriverStation.reportWarning(message, eventType == EventType.Error);
    }

    /**
     * Writes an event to a log file stored on the directory given
     * Writes only if the given debug level is above the one given in the contructor
     * 
     * @param _debug
     * @param Title
     * @param Message
     */
    public synchronized void writeEvent(int _debug, String Title, String Message){
        writeEvent(_debug, Title + "-" + Message);
    }
    
    /**
     * Writes an event to a log file stored on the directory given
     * Writes only if the given debug level is above the one given in the contructor
     * The event type is used as the Title for the log
     * 
     * @param _debug
     * @param event
     * @param Message
     */
    public synchronized void writeEvent(int _debug, EventType event, String Message) {
        writeEvent(_debug, event.toString() + Message);
    }
    
    /**
     * Writes an event to a glof file stored on the diectory given
     * Writes only if the given debug level is above the one given in the contructor
     * Only writes the message with no Title
     * 
     * @param _debug
     * @param Message
     */
    public synchronized void writeEvent(int _debug, String Message) {
        if (_debug >= this.debug) {
            writeLine(Message);
        }
    }
    
    /**
     * Internal function that writes a single line to the file and able to handle errors thrown
     * @param toWrite
     */
    private synchronized void writeLine(String toWrite) {
        try {
            this.writer.write(toWrite + "\n");
        } catch (Exception e) {
            System.out.println("Error Writing To File Logger!!");
            e.printStackTrace();
        }
    }

    /**
     * List all the files in the Log directory
     * Then if there is more than the max log amount then delete
     */
    public void cleanLogs(){
//        File[] fileList = new File(Constants.strLogDirectory).listFiles();
//
//        if (fileList.length > Constants.intMaxLogFiles) {
//            for (int i = Constants.intMaxLogFiles; i < fileList.length; i++) {
//                fileList[i].delete();
//            }
//        }
    }

    /**
     * Close the FileLogger writer
     */
    public void close(){
        try {
			this.writer.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
}