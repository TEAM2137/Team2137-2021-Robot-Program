package frc.robot.base;

import frc.robot.util.FileLogger;
import frc.robot.util.XMLSettingReader;

public interface HardwareBasicDriveTrain {
    void init(XMLSettingReader xmlSettingReader, FileLogger log);
}