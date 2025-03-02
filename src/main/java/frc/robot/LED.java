package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Utility.LEDHelpers;

public class LED {
    private int locateLED = 0;
    private float chosenHue = 0;

    private final AddressableLED m_led = new AddressableLED(1);
    private final AddressableLEDBuffer ledBuffer;
    private AddressableLEDBuffer priorLedBuffer;

    public final int numberOfLEDs;
    public final int id;

    private final double[] oceanHSV = {190, 1, 0.85};
    private final double[] darkSkyHSV = {239, 0.94, 0.19};
    private final double[] stowBackgroundHSV = oceanHSV;
    private final double[] scoreBackgroundHSV = {93, 0.99, 1};
    private final double[] nominalChaserHSV = {120, 1, 1};
    private final double[] warningChaserHSV = {60, 1, 1};
    private final double[] criticalChaserHSV = {10, 1, 1};

    public static enum LEDStates {
        NORMAL,
        PARTY,
        LOCATE,
        CHOOSEHUE
    };

    public static enum ChaserStates {
        NOMINAL,
        WARNING,
        CRITICAL
    };

    public static enum LEDGripperGameStates { //TODO
        NORMAL, //all four team colors slowly sliding
        INTAKING, //alternating magenta and green chaser with trail going inward
        OUTTAKING, //alternating magenta and green chaser with trail going outward
        CORAL, //solid white and yellow
        ALGAE, //solid teal
        LOCKED_CORAL, //flashing white and yellow
        LOCKED_ALGAE, //flashing teal
        CLIMB //slowly go from black to ocean color over course of endgame
    };

    public static enum LEDArmGameStates {
        NORMAL, //solid blue with 3 indicator LEDs at top and upward chaser. Green if nothings wrong, yellow warnings (yellow master alarms), red for critical errors (master alarms)
        CLIMB //same as gripper
    }

    public LEDGripperGameStates LEDGripperGameState = LEDGripperGameStates.NORMAL;

    public LEDArmGameStates LEDArmGameState = LEDArmGameStates.NORMAL;

    public ChaserStates chaserState = ChaserStates.NOMINAL;

    private double animationIndex = 0;

    private LEDStates ledState = LEDStates.NORMAL; // default to normal

    /**
     * Creates a new LED object to control LED outputs
     */
    public LED(int count, int id) {
        numberOfLEDs = count;
        this.id = id;

        ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
        priorLedBuffer = new AddressableLEDBuffer(numberOfLEDs);

        m_led.setLength(numberOfLEDs);
    }

    /**
     * Updates the LED state based on driver inputs and robot states
     * 
     * @param driverController)
     */
    public void updateLED(XboxController driverController, boolean isAutonomous, double time) {
        // If driver presses up d-pad, increment LED state
        if (driverController.getPOV() < 45 || driverController.getPOV() > 315) {
            ledState = (ledState.equals(LEDStates.CHOOSEHUE)) ? LEDStates.NORMAL : LEDStates.values()[ledState.ordinal() + 1];
        }

        // remember previous LED buffer
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            priorLedBuffer.setRGB(i, ledBuffer.getRed(i), ledBuffer.getGreen(i), ledBuffer.getBlue(i));
        }

        //update LED strip specific states
        if (id == 1) {
            switch (Robot.masterState) {
                case CLMB:
                    LEDArmGameState = LEDArmGameStates.CLIMB;
                    break;
                default:
                    LEDArmGameState = LEDArmGameStates.NORMAL;
            }
        }

        // Depending on current LED state, send a new output to the LEDs
        switch (ledState) {
            case NORMAL:
                // Normal
                if (id == 1) {
                    switch (LEDArmGameState) {
                        case NORMAL:
                            double[] bgRGB;
                            switch (Robot.masterState) {
                                case SCOR:
                                    bgRGB = LEDHelpers.hsvToRgb(scoreBackgroundHSV);
                                    break;
                                case FEED:
                                    bgRGB = LEDHelpers.hsvToRgb(scoreBackgroundHSV);
                                    break;
                                default:
                                    bgRGB = LEDHelpers.hsvToRgb(stowBackgroundHSV);
                            }

                            //update chaser state
                            //TODO
                            if (Robot.loopTime_ms > 50) { //critical conditions
                                chaserState = ChaserStates.CRITICAL;
                            } else if (Robot.loopTime_ms > 30) { //warning conditions
                                chaserState = ChaserStates.WARNING;
                            } else { //if none are met, chaser is nominal
                                chaserState = ChaserStates.NOMINAL;
                            }

                            //update chaser color based on chaser state
                            double[] chaserRGB;
                            switch (chaserState) {
                                case CRITICAL:
                                    chaserRGB = LEDHelpers.hsvToRgb(criticalChaserHSV);
                                    break;
                                case WARNING:
                                    chaserRGB = LEDHelpers.hsvToRgb(warningChaserHSV);
                                    break;
                                default:
                                    chaserRGB = LEDHelpers.hsvToRgb(nominalChaserHSV);
                                    break;
                            }

                            // set all LED's to the bg color, except the chaser which is chaserRGB
                            for (int i = 0; i < ledBuffer.getLength(); i++) {
                                if (i == animationIndex || i < 2) {
                                    ledBuffer.setRGB(i, (int) chaserRGB[0], (int) chaserRGB[1], (int) chaserRGB[2]);
                                } else {
                                    ledBuffer.setRGB(i, (int) bgRGB[0], (int) bgRGB[1], (int) bgRGB[2]);
                                }
                            }

                            animationIndex += 1;
                            animationIndex %= numberOfLEDs - 3;
                            break;
                        case CLIMB:
                            for (int i = 0; i < ledBuffer.getLength(); i++) {
                                double t = time - (160 * 1000);
                                double[] hsv = (i > ((4*Math.sin(Math.PI*t/2000))+(t/1000))/20*ledBuffer.getLength()) ? darkSkyHSV:oceanHSV; //makes a nice wave effect
                                double[] rgb = LEDHelpers.hsvToRgb(hsv);
                                ledBuffer.setRGB(0, (int) rgb[0], (int) rgb[1], (int) rgb[2]);
                            }
                    }
                } else if (id == 2) {

                }
                break;
            case PARTY:
                // Party mode
                
                break;
            case LOCATE:
                double[] rgb = LEDHelpers.hsvToRgb(nominalChaserHSV);
                ledBuffer.setRGB(locateLED, (int) rgb[0], (int) rgb[1], (int) rgb[2]);
                break;
            case CHOOSEHUE:
                double[] chooseRGB = LEDHelpers.hsvToRgb(chosenHue, 1, 1);
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    // Sets the specified LED to the HSV values
                    ledBuffer.setRGB(i, (int) chooseRGB[0], (int) chooseRGB[1], (int) chooseRGB[2]);
                }
                break;
        }

    }

    /**
     * Sends stored values in the LED object to leds
     */
    public void updateOutputs() {
        if (ledBuffer != priorLedBuffer) {
            m_led.setData(ledBuffer);
        }
    }
}
