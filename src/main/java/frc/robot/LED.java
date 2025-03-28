package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Utility.ClassHelpers.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Utility.LEDHelpers;

public class LED {
    private int locateLED = 0;
    private float chosenHue = 250;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer ledBuffer;
    private AddressableLEDBuffer priorLedBuffer;

    public final int numberOfLEDs;
    public final int numStrips;

    public static enum StripIDs {
        AFRAME,
        EXT
    };

    public final StripIDs id;

    public final double[][] teamColorsRGB = {{255,0,153}, {71,143,205}, {118,255,3}, {247,235,18}};

    private final double[] oceanHSV = {180, 1, 1};
    private final double[] darkSkyHSV = {239, 1, 0.19};
    private final double[] stowBackgroundHSV = {oceanHSV[0], 1, 0.5};
    private final double[] scoreBackgroundHSV = {330, 1, 0.6};
    private final double[] feedBackgroundHSV = {240, 1, 0.4};
    private final double[] nominalChaserHSV = {120, 1, 1};
    private final double[] warningChaserHSV = {50, 1, 1};
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

    private Timer batteryCriticalTimer = new Timer();
    private Timer batteryWarningTimer = new Timer();
    private Timer loopTimeTimer = new Timer();

    public static Timer hasPieceFlash = new Timer();

    private LEDStates ledState = LEDStates.NORMAL; // default to normal

    /**
     * Creates a new LED object to control LED outputs
     * count * numStrips = total LED's
     */
    public LED(int count, int numStrips, StripIDs id, int startLed) {
        numberOfLEDs = count;
        this.numStrips = numStrips;
        this.id = id;

        ledBuffer = new AddressableLEDBuffer(numberOfLEDs * numStrips);
        priorLedBuffer = new AddressableLEDBuffer(numberOfLEDs * numStrips);

        m_led = new AddressableLED(0);

        m_led.setLength(numberOfLEDs * numStrips);

        for (int i = 0; i < numberOfLEDs; i++) {
            if (i <= numberOfLEDs/4) {
                LEDHelpers.setLED(ledBuffer, i, teamColorsRGB[0]);
            } else if (i <= numberOfLEDs/2 && i > numberOfLEDs/4) {
                LEDHelpers.setLED(ledBuffer, i, teamColorsRGB[1]);
            } else if (i <= numberOfLEDs/4*3 && i > numberOfLEDs/2) {
                LEDHelpers.setLED(ledBuffer, i, teamColorsRGB[2]);
            } else {
                LEDHelpers.setLED(ledBuffer, i, teamColorsRGB[3]);
            }
        }

        m_led.setData(ledBuffer);
        //m_led.setBitTiming(300, 600, 900, 600);
        m_led.start();
    }

    public static Command hasPieceBlink() {
        return Commands.startEnd(
            () -> {
                LED.hasPieceFlash.reset();
            },
            () -> {}
        );
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
        if (id == StripIDs.EXT) {
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
                switch (id) {
                    case EXT:
                        switch (LEDArmGameState) {
                            case NORMAL:
                                double[] bgRGB;
                                switch (Robot.masterState) {
                                    case SCOR:
                                        bgRGB = scoreBackgroundHSV;
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
                                        bgRGB = feedBackgroundHSV;
                                        break;
                                    default:
                                        bgRGB = stowBackgroundHSV;
                                        animationIndex2 = -1;
                                }

                                if (LED.hasPieceFlash.getTimeMillis() < 1000) {
                                    if (Intake.hasPiece) {
                                        animationIndex2+=0.2; // 1/4 second per toggle of LED
                                        if (animationIndex2 > 2) {
                                            animationIndex2 = 0;
                                        }
                                    } else {
                                        animationIndex2 = -1;
                                    }
                                }

                                if (animationIndex2 > -0.5) {
                                    bgRGB[2] *= Math.min(Math.max((animationIndex2 + ((animationIndex2 > 1) ? -2:0)) * ((animationIndex2 > 1) ? -1:1),0),1); //blinking animation
                                }
                                bgRGB = LEDHelpers.hsvToRgb(bgRGB);

                                //update timers
                                if (RobotController.getBatteryVoltage() > 9) {
                                    batteryCriticalTimer.reset();
                                }
                                if (RobotController.getBatteryVoltage() > 10) {
                                    batteryWarningTimer.reset();
                                }
                                if (Robot.loopTime_ms < 50) {
                                    loopTimeTimer.reset();
                                }

                                //update chaser state
                                //TODO
                                boolean swerveFaults = Drivetrain.driveFaults[0] || Drivetrain.driveFaults[1] || Drivetrain.driveFaults[2] || Drivetrain.driveFaults[3] || Drivetrain.azimuthFaults[0] || Drivetrain.azimuthFaults[1] || Drivetrain.azimuthFaults[2] || Drivetrain.azimuthFaults[3];
                                if (loopTimeTimer.getTimeMillis() > 1000 || swerveFaults || batteryCriticalTimer.getTimeMillis() > 1000) { //critical conditions
                                    chaserState = ChaserStates.CRITICAL;
                                } else if (Robot.loopTime_ms > 100 || batteryWarningTimer.getTimeMillis() > 1000) { //warning conditions
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
                                    if (i == Math.floor(animationIndex)) {
                                        LEDHelpers.setLED(ledBuffer, i, chaserRGB);
                                        if (i > 1) {
                                            LEDHelpers.setLED(ledBuffer, i-1, new double[] {0,0,0});
                                        }
                                    } else {
                                        LEDHelpers.setLED(ledBuffer, i, bgRGB);
                                    }
                                }

                                //make front of A-frame white when robot is in score
                                if (Robot.masterState == Robot.MasterStates.SCOR) {
                                    for (int i = numberOfLEDs/2+1; i < numberOfLEDs; i++) {
                                        LEDHelpers.setLED(ledBuffer, i, new double[] {0,0,0.5});
                                    }
                                }

                                animationIndex += 0.5;
                                if (animationIndex > numberOfLEDs) {
                                    animationIndex = 1;
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
                        break;
                    case AFRAME:
                        break;
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
