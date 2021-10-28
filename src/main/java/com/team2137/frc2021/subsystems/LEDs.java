package com.team2137.frc2021.subsystems;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team2137.libs.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDs extends SubsystemBase {

    private static LEDs instance;

    private CANifier canifier;

    private State currentState;
    private State defaultState;

    private double cycleCurrentHue = 0;
    private Timer cycleTimer = new Timer();

    public LEDs(CANifier canifier) {
        this.canifier = canifier;
        this.currentState = State.Off;

        this.cycleTimer.start();

        instance = this;

        setLEDs(new Color(0 ,0, 0));
    }

    public static LEDs getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
//        System.out.println(currentState);
        switch(currentState.type) {
            case Static:
                setLEDs(currentState.color);
                break;
            case HSVCycle:
                cycleCurrentHue += currentState.increment;
                cycleCurrentHue %= 360;

                double outputHue = (cycleCurrentHue + currentState.startingHue) % 360;
//                System.out.println(outputHue);
                Color color = Color.fromHSV((int) Util.clamp(outputHue, 0, 360) / 2, (int) currentState.s * 255, (int) currentState.v * 255);
//                System.out.println(color.red + " " + color.green + " " + color.blue);
                setLEDs(color);
                break;
            case Blink:
//                System.out.println(cycleTimer.get() % (currentState.onTime + currentState.offTime));
                if(cycleTimer.get() % (currentState.onTime + currentState.offTime) < currentState.onTime) {
                    setLEDs(currentState.color);
                } else {
                    setLEDs(0, 0, 0);
                }
                break;
            case ColorCycle:
                double currentTime = cycleTimer.get() % (currentState.colors.length * currentState.timePerColor);

                Color currentColor = currentState.colors[(int) Math.floor(currentTime % currentState.timePerColor)];

//                System.out.println(Math.floor(currentTime % currentState.timePerColor));

                setLEDs(currentColor);
                break;
        }
    }

    /**
     * Sets the LEDs to a certain RGB value
     * @param r the red value of the color (0-1)
     * @param g the green value of the color (0-1)
     * @param b the blue value of the color (0-1)
     */
    public void setLEDs(double r, double g, double b) {
        canifier.setLEDOutput(r, CANifier.LEDChannel.LEDChannelB);
        canifier.setLEDOutput(g, CANifier.LEDChannel.LEDChannelC);
        canifier.setLEDOutput(b, CANifier.LEDChannel.LEDChannelA);
    }

    /**
     * Sets the LEDs to a certain color
     * @param color the color to be displayed
     */
    public void setLEDs(Color color) {
        setLEDs(color.red, color.green, color.blue);
    }

    /**
     * Sets the current state of the LEDs
     * @param state the state to set
     */
    public void setState(State state) {
        currentState = state;
        cycleTimer.reset();
    }

    /**
     * Sets a state as default state, does not enable it
     * @param state the state to be default
     * @param enableNow whether to enable that state currently
     */
    public void setDefaultState(State state, boolean enableNow) {
        defaultState = state;
        if(enableNow)
            enableDefaultState();
    }

    /**
     * Enables the default state, clearing the current state
     */
    public void enableDefaultState() {
        currentState = defaultState;
    }

    public enum State {
        Off(0, 0, 0),

        Red(255, 0, 0),
        Green(0, 255, 0),
        Blue(0, 0, 255),
        Yellow(255, 255, 0),
        Orange(255, 165, 0),

        BlinkBlue(0, 0, 255, 2, 1),

        RainbowCycle(0, 1, 1, 2),

        TeamColorCycle(2, new Color(0, 0, 1), new Color(1, 0.647, 0));
        ;

        Color color;
        double onTime, offTime;

        double startingHue, s, v;
        double increment;

        double timePerColor;
        Color[] colors;

        Type type;

        /**
         * Solid RGB state
         * @param r The red value (0-255)
         * @param g The green value (0-255)
         * @param b The blue value (0-255)
         */
        State(int r, int g, int b) {
            this.color = new Color(new Color8Bit(r, g, b));
            this.type = Type.Static;
        }

        /**
         * RGB state that blinks on/off
         * @param r The red value
         * @param g The green value
         * @param b The blue value
         * @param onTime The time to keep the state on in seconds
         * @param offTime The time to keep the state off in seconds
         */
        State(int r, int g, int b, double onTime, double offTime) {
            this.color = new Color(new Color8Bit(r, g, b));
            this.type = Type.Blink;
            this.onTime = onTime;
            this.offTime = offTime;
        }

        /**
         * Starts at a certain HSV value and cycles around that
         * @param startingHue The hue to start at
         */
        State(double startingHue, double s, double v, double increment) {
            this.startingHue = startingHue;
            this.s = s;
            this.v = v;
            this.increment = increment;
            this.type = Type.HSVCycle;
        }

        /**
         * Cycles through a set of colors
         * @param timePerColor the amount of time to display each color in seconds
         * @param colors the colors to cycle through
         */
        State(double timePerColor, Color... colors) {
            this.timePerColor = timePerColor;
            this.colors = colors;
            this.type = Type.ColorCycle;
        }

        private enum Type {
            Static, Blink, HSVCycle, ColorCycle
        }
    }
}
