package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Utility.LEDHelpers;
import frc.robot.Utility.ClassHelpers.BlinkLight;

public class LED {
    private int locateLED = 0;
    private float chosenHue = 0;

    private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer;
    private AddressableLEDBuffer priorLedBuffer;
    private final int numberOfLEDs;

    private final float stowBackground = 300;
    private final float shootingBackground = 240;
    private final float ampBackground = 210;
    private final float climbingBackground = 180;
    private final float nominalChaser = 120;
    private final float humanControl = 120;
    private final float robotControl = 14;

    private final String nominalChaserGRB = LEDHelpers.rgbtogrb(LEDHelpers.hsvToRgb(nominalChaser, 1, 1));
    private final String humanControlGRB = LEDHelpers.rgbtogrb(LEDHelpers.hsvToRgb(humanControl, 1, 1));
    private final String robotControlGRB = LEDHelpers.rgbtogrb(LEDHelpers.hsvToRgb(robotControl, 1, 1));

    enum LEDStates {
        NORMAL,
        PARTY,
        LOCATE,
        CHOOSEHUE
    }

    private double chaserStatus = 0;

    private LEDStates ledState = LEDStates.NORMAL; // default to normal

    private final BlinkLight blinkyBoi250 = new BlinkLight(250);
    private final BlinkLight blinkyBoi50 = new BlinkLight(50);

    /**
     * Creates a new LED object to control LED outputs
     */
    public LED() {
        switch (Robot.robotProfile) {
            case "2024_Robot":
                numberOfLEDs = 20;
                break;
            case "Steve2":
                numberOfLEDs = 20;
                break;
            case "Triceptasaurus":
                numberOfLEDs = 42;
                break;
            default:
                numberOfLEDs = 20;
        }
        m_ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
        priorLedBuffer = new AddressableLEDBuffer(numberOfLEDs);

        m_led.setLength(numberOfLEDs);
    }

    /**
     * Updates the LED state based on driver inputs and robot states
     * 
     * @param driverController)
     */
    public void updateLED(XboxController driverController, boolean isAutonomous) {
        // If driver presses up d-pad, increment LED state
        if (driverController.getPOV() < 45 || driverController.getPOV() > 315) {
            ledState = (ledState.equals(LEDStates.CHOOSEHUE)) ? LEDStates.values()[ledState.ordinal() + 1] : LEDStates.NORMAL;
        }

        // remember previous LED buffer
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            priorLedBuffer.setRGB(i, m_ledBuffer.getRed(i), m_ledBuffer.getGreen(i), m_ledBuffer.getBlue(i));
        }

        // Depending on current LED state, send a new output to the LEDs
        switch (ledState) {
            case NORMAL:
                // Normal
                String rgb = LEDHelpers.hsvToRgb(stowBackground, 1, 1);
                switch (Robot.masterState) {
                    case STOWED:
                        // Blink while intaking without a piece (250ms)
                        if ((!Intake.bbBroken) && Intake.enabled) {
                            if (blinkyBoi250.blinkLight()) {
                                rgb = LEDHelpers.hsvToRgb(stowBackground, 1, 1);
                            } else {
                                rgb = LEDHelpers.hsvToRgb(1, 1, 0);
                            }
                        } else {
                            rgb = LEDHelpers.hsvToRgb(stowBackground, 1, 1);
                        }
                        break;
                    case SHOOTING:
                        // Blink while ready to shoot (50ms) and while intaking without a piece (250ms)
                        if (Shooter.spunUp) {
                            if (blinkyBoi50.blinkLight()) {
                                rgb = LEDHelpers.hsvToRgb(shootingBackground, 1, 1);
                            } else {
                                rgb = LEDHelpers.hsvToRgb(1, 1, 0);
                            }
                        } else {
                            if ((!Intake.bbBroken) && Intake.enabled) {
                                if (blinkyBoi250.blinkLight()) {
                                    rgb = LEDHelpers.hsvToRgb(shootingBackground, 1, 1);
                                } else {
                                    rgb = LEDHelpers.hsvToRgb(1, 1, 0);
                                }
                            } else {
                                rgb = LEDHelpers.hsvToRgb(shootingBackground, 1, 1);
                            }
                        }
                        break;
                    case AMP:
                        // Blink while ready to shoot (50ms) and while intaking without a piece (250ms)
                        if (Shooter.spunUp) {
                            if (blinkyBoi50.blinkLight()) {
                                rgb = LEDHelpers.hsvToRgb(ampBackground, 1, 1);
                            } else {
                                rgb = LEDHelpers.hsvToRgb(1, 1, 0);
                            }
                        } else {
                            if ((!Intake.bbBroken) && Intake.enabled) {
                                if (blinkyBoi250.blinkLight()) {
                                    rgb = LEDHelpers.hsvToRgb(stowBackground, 1, 1);
                                } else {
                                    rgb = LEDHelpers.hsvToRgb(1, 1, 0);
                                }
                            }
                        }
                        break;
                    case CLIMBING:
                        if (Elevator.climbed) {
                            rgb = LEDHelpers.hsvToRgb(climbingBackground, 1, 1);
                        } else {
                            if (blinkyBoi250.blinkLight()) {
                                rgb = LEDHelpers.hsvToRgb(climbingBackground, 1, 1);
                            } else {
                                rgb = LEDHelpers.hsvToRgb(1, 1, 0);
                            }
                        }
                        break;
                    case TRAP:
                        rgb = LEDHelpers.hsvToRgb(climbingBackground, 1, 1);
                    default:
                        rgb = LEDHelpers.hsvToRgb(stowBackground, 1, 1);
                        break;
                }
                var normalGrbVal = LEDHelpers.rgbtogrb(rgb);

                // Set background color and chaser
                for (var i = 0; i < m_ledBuffer.getLength() - 3; i++) {
                    if (chaserStatus == i) {
                        m_ledBuffer.setRGB(i,
                                Integer.parseInt(nominalChaserGRB.substring(0,3)),
                                Integer.parseInt(nominalChaserGRB.substring(3,6)),
                                Integer.parseInt(nominalChaserGRB.substring(6,9)));
                    } else {
                        m_ledBuffer.setRGB(i,
                                Integer.parseInt(normalGrbVal.substring(0,3)),
                                Integer.parseInt(normalGrbVal.substring(3,6)),
                                Integer.parseInt(normalGrbVal.substring(6,9)));
                    }
                }
                m_ledBuffer.setRGB(numberOfLEDs - 3, 0, 0, 0);

                // Set the final 2 lights to show robot control mode
                if (!isAutonomous) {
                    m_ledBuffer.setRGB(numberOfLEDs - 2,
                            Integer.parseInt(humanControlGRB.substring(0,3)),
                            Integer.parseInt(humanControlGRB.substring(3,6)),
                            Integer.parseInt(humanControlGRB.substring(6,9)));
                    m_ledBuffer.setRGB(numberOfLEDs - 1,
                            Integer.parseInt(humanControlGRB.substring(0,3)),
                            Integer.parseInt(humanControlGRB.substring(3,6)),
                            Integer.parseInt(humanControlGRB.substring(6,9)));
                } else {
                    m_ledBuffer.setRGB(numberOfLEDs - 2,
                            Integer.parseInt(robotControlGRB.substring(0,3)),
                            Integer.parseInt(robotControlGRB.substring(3,6)),
                            Integer.parseInt(robotControlGRB.substring(6,9)));
                    m_ledBuffer.setRGB(numberOfLEDs - 1,
                            Integer.parseInt(robotControlGRB.substring(0,3)),
                            Integer.parseInt(robotControlGRB.substring(3,6)),
                            Integer.parseInt(robotControlGRB.substring(6,9)));
                }
                chaserStatus += 1;
                chaserStatus %= numberOfLEDs - 3;
                break;
            case PARTY:
                // Party mode
                for (var i = 0; i < m_ledBuffer.getLength(); i++) {

                    var partyGrbVal = LEDHelpers.rgbtogrb(LEDHelpers.hsvToRgb((int)(360 / numberOfLEDs * i + chaserStatus)%360, 1, 1));

                    // Sets the specified LED to the HSV values
                    m_ledBuffer.setRGB(i, (i * 20) + Integer.parseInt(partyGrbVal.substring(0,3)),
                            (i * 20) + Integer.parseInt(partyGrbVal.substring(3,6)),
                            (i * 20) + Integer.parseInt(partyGrbVal.substring(6,9)));
                }
                // Increment chaser (Chaser is a double so that increments don't have to be
                // integers)
                chaserStatus += 2;
                chaserStatus %= 360;
                break;
            case LOCATE:
                m_ledBuffer.setRGB(locateLED,
                        Integer.parseInt(nominalChaserGRB.substring(0,3)),
                        Integer.parseInt(nominalChaserGRB.substring(3,6)),
                        Integer.parseInt(nominalChaserGRB.substring(6,9)));
                break;
            case CHOOSEHUE:
                String chooseGRB = LEDHelpers.rgbtogrb(LEDHelpers.hsvToRgb(chosenHue, 1, 1));
                for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                    // Sets the specified LED to the HSV values
                    m_ledBuffer.setRGB(i,
                            Integer.parseInt(chooseGRB.substring(0,3)),
                            Integer.parseInt(chooseGRB.substring(3,6)),
                            Integer.parseInt(chooseGRB.substring(6,9)));
                }
                break;
        }

    }

    /**
     * Sends stored values in the LED object to leds
     */
    public void updateOutputs() {
        if (m_ledBuffer != priorLedBuffer) {
            m_led.setData(m_ledBuffer);
        }
    }
}
