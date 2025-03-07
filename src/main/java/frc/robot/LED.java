package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utility.LEDHelpers;

public class LED {
    private int locateLED = 0;
    private float chosenHue = 250;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer ledBuffer;
    private AddressableLEDBuffer priorLedBuffer;

    public final int numberOfLEDs;
    public final int numStrips;
    public final int id;

    private final double[] oceanHSV = {180, 1, 1};
    private final double[] darkSkyHSV = {239, 0.94, 0.19};
    private final double[] stowBackgroundHSV = {oceanHSV[0], 1, 0.5};
    private final double[] scoreBackgroundHSV = {300, 1, 1};
    private final double[] feedBackgroundHSV = {250, 1, 1};
    private final double[] nominalChaserHSV = {120, 1, 1};
    private final double[] warningChaserHSV = {40, 1, 1};
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
    private double animationIndex2 = 0;

    private boolean LEDStateEdge = false;

    private LEDStates ledState = LEDStates.NORMAL; // default to normal

    /**
     * Creates a new LED object to control LED outputs
     * count * numStrips = total LED's
     */
    public LED(int count, int numStrips, int id, int port) {
        numberOfLEDs = count;
        this.numStrips = numStrips;
        this.id = id;

        ledBuffer = new AddressableLEDBuffer(numberOfLEDs * numStrips);
        priorLedBuffer = new AddressableLEDBuffer(numberOfLEDs * numStrips);

        m_led = new AddressableLED(port);

        m_led.setLength(numberOfLEDs * numStrips);

        for (int i = 0; i < numberOfLEDs; i++) {
            double[] grb = LEDHelpers.rgbtogrb(255, 0, 255);
            ledBuffer.setRGB(i, (int) grb[0], (int) grb[1], (int) grb[2]);
        }

        m_led.setData(ledBuffer);
        //m_led.setBitTiming(300, 600, 900, 600);
        m_led.start();
    }

    /**
     * Updates the LED state based on driver inputs and robot states
     * 
     * @param driverController
     */
    public void updateLED(XboxController driverController, boolean isAutonomous, double time, boolean atPos) {
        // If driver presses up d-pad, increment LED state
        if ((driverController.getPOV() < 45 || driverController.getPOV() > 315) && driverController.getPOV() >= 0) {
            if (!LEDStateEdge) {
                ledState = (ledState.equals(LEDStates.CHOOSEHUE)) ? LEDStates.NORMAL : LEDStates.values()[ledState.ordinal() + 1];
                LEDStateEdge = true;
            }
        } else {
            LEDStateEdge = false;
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
                                case STOW:
                                    bgRGB = LEDHelpers.hsvToRgb(scoreBackgroundHSV);
                                    if (atPos) {
                                        animationIndex2+=0.1; // 1/2 second per toggle of LED
                                        if (animationIndex2 > 2) {
                                            animationIndex2 = 0;
                                        }
                                    } else {
                                        animationIndex2 = -1;
                                    }
                                    break;
                                case FEED:
                                    bgRGB = LEDHelpers.hsvToRgb(feedBackgroundHSV);
                                    animationIndex2 = -1;
                                    break;
                                default:
                                    bgRGB = LEDHelpers.hsvToRgb(stowBackgroundHSV);
                                    animationIndex2 = -1;
                            }

                            if (animationIndex2 > 1) {
                                bgRGB = new double[] {0,0,0};
                            }

                            //update chaser state
                            //TODO
                            boolean swerveFaults = Drivetrain.driveFaults[0] || Drivetrain.driveFaults[1] || Drivetrain.driveFaults[2] || Drivetrain.driveFaults[3] || Drivetrain.azimuthFaults[0] || Drivetrain.azimuthFaults[1] || Drivetrain.azimuthFaults[2] || Drivetrain.azimuthFaults[3];
                            if (Robot.loopTime_ms > 50 || swerveFaults) { //critical conditions
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
                                if (i == Math.floor(animationIndex) || i < 2) {
                                    LEDHelpers.setLED(ledBuffer, i, chaserRGB);
                                    if (i > 2) {
                                        LEDHelpers.setLED(ledBuffer, i-1, new double[] {0,0,0});
                                    }
                                } else {
                                    LEDHelpers.setLED(ledBuffer, i, bgRGB);
                                }
                            }

                            animationIndex += 0.5;
                            if (animationIndex >= numberOfLEDs) {
                                animationIndex = 2;
                            }
                            break;
                        case CLIMB:
                            for (int i = 0; i < ledBuffer.getLength(); i++) {
                                double t = time - (160 * 1000);
                                double[] hsv = (i > ((4*Math.sin(Math.PI*t/2000))+(t/1000))/20*ledBuffer.getLength()) ? darkSkyHSV:oceanHSV; //makes a nice wave effect
                                double[] rgb = LEDHelpers.hsvToRgb(hsv);
                                LEDHelpers.setLED(ledBuffer, 0, rgb);
                            }
                    }
                } else if (id == 2) {

                }
                break;
            case PARTY:
                // Party mode
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    LEDHelpers.setLED(ledBuffer, i, LEDHelpers.hsvToRgb((((double) i)/ledBuffer.getLength())*360 + animationIndex, 1, 0.75 - (Math.sin(animationIndex/180*Math.PI)/4)));
                }
                animationIndex += 5;
                if (animationIndex > 360) {
                    animationIndex = 0;
                }
                break;
            case LOCATE:
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    if (locateLED == i) {
                        LEDHelpers.setLED(ledBuffer, i, new double[] {255,0,255});
                    } else {
                        LEDHelpers.setLED(ledBuffer, i, new double[] {0,0,0});
                    }
                }
                break;
            case CHOOSEHUE:
                double[] chooseRGB = LEDHelpers.hsvToRgb(chosenHue, 1, 1);
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    // Sets the specified LED to the HSV values
                    LEDHelpers.setLED(ledBuffer, i, chooseRGB);
                }
                break;
        }

    }

    /**
     * Sends stored values in the LED object to leds
     */
    public void updateOutputs() {
        if (ledBuffer != priorLedBuffer) { 
            if (numStrips > 1) { // code for multiple strips in one pwm port
                for (int i = 0; i < numberOfLEDs; i++) {
                    Color color = ledBuffer.getLED(i);
                    for (int i2 = 1; i2 < numStrips; i2++) {
                        LEDHelpers.setLED(ledBuffer, i+(i2*(numberOfLEDs)), color);
                    }
                }
            }
            m_led.setData(ledBuffer);
        }
    }
}
