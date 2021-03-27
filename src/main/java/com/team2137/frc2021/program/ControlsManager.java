package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.libs.GamePad;
import com.team2137.libs.GamePad.Axis;
import com.team2137.libs.GamePad.Button;
import com.team2137.libs.GamePad.DPad;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Class to manage controls all in one place
 * This allows easy changing of controls without needing modify the main robot code
 */
public class ControlsManager {

    static GamePad driverController     = new GamePad(0);
    static GamePad operatorController   = new GamePad(1);

    public enum Control {

        DriveAxis               (Axis.kLeftY, driverController),
        StrafeAxis              (Axis.kLeftX, driverController),
        RotationAxis            (Axis.kRightX, driverController),
        XLockButton             (Button.kBumperRight, driverController),

        HeadingTargetButton     (Button.kBumperLeft, operatorController),

        IntakeButton            (Button.kA, driverController),

        ShooterStage1           (Button.kBumperRight, operatorController),
        ShooterStage2           (Button.kY, operatorController),
        ShooterStage3           (Button.kB, operatorController),
        ShooterStage4           (Button.kA, operatorController);

        private int id;
        private GenericHID controller;
        private InputType inputType;

        Control(int id, GenericHID controller, InputType inputType) {
            this.id = id;
            this.controller = controller;
            this.inputType = inputType;
        }

        Control(Button button, GenericHID controller) {
            this.id = button.getPort();
            this.controller = controller;
            this.inputType = InputType.Button;
        }

        Control(Axis button, GenericHID controller) {
            this.id = button.getPort();
            this.controller = controller;
            this.inputType = InputType.Axis;
        }

        Control(DPad button, GenericHID controller) {
            this.id = button.getPort();
            this.controller = controller;
            this.inputType = InputType.POV;
        }
    }

    public static enum InputType {
        Axis, Button, POV;
    }

    /**
     * Gets the value of an axis
     * @param control The control to grab the value from
     * @return The value of the axis
     */
    public static double getAxis(Control control) {
        if (control.inputType != InputType.Axis) {
            return 0;
        }
        return control.controller.getRawAxis(control.id);
    }

    public static double getAxis(Control control, double deadBandWidth) {
        if (control.inputType != InputType.Axis) {
            return 0;
        }
        return applyDeadBand(control.controller.getRawAxis(control.id), deadBandWidth);
    }

    /**
     * Gets whether a button is pressed or not
     * @param control The control to grab the value from
     * @return Whether the button is pressed or not
     */
    public static boolean getButton(Control control) {
        if (control.inputType != InputType.Button) {
            return false;
        }
        return control.controller.getRawButton(control.id);
    }

    /**
     * Gets whether a POV (ex: D-pad) is pointing in a certain direction
     * @param control The control to reference (id = direction)
     * @return Whether the POV is pointing in the requested direction
     */
    public static boolean getPOVBoolean(Control control) {
        if (control.inputType != InputType.POV) {
            return false;
        }
        return control.controller.getPOV() == control.id;
    }

    /**
     * Gets the angle that a POV is pointing
     * @param control The control to reference (just checks controller's POV)
     * @return The angle the POV is pointing
     */
    public static int getPOVAngle(Control control) {
        if (control.inputType != InputType.POV) {
            return 0;
        }
        return control.controller.getPOV();
    }

    public static double applyDeadBand(double value, double deadBandWidth){
        if(Math.abs(value) < deadBandWidth)
            return 0;
        else {
            if (value > 0.0) {
                return (value - deadBandWidth) / (1.0 - deadBandWidth);
            } else {
                return (value + deadBandWidth) / (1.0 - deadBandWidth);
            }
        }
    }
}
