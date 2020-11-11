package frc.robot.util;

import frc.robot.Constants.MotorTypes;

public interface HardwareMotorLayout {
    String getMotorName();  

    MotorTypes getMotorType();

    int getMotorID();
    default int getID() {
        return getMotorID();
    }
}