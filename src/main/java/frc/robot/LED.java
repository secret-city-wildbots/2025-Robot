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

    private final float ocean = 198;
    private final double[] darkSkyHSV = {239, 0.94, 0.19};
    private final float stowBackground = ocean;
    private final float nominalChaser = 120;
    private final float humanControl = 120;
    private final float robotControl = 14;

    private final double[] nominalChaserGRB = LEDHelpers.rgbtogrb(LEDHelpers.hsvToRgb(nominalChaser, 1, 1));
    private final double[] humanControlGRB = LEDHelpers.rgbtogrb(LEDHelpers.hsvToRgb(humanControl, 1, 1));
    private final double[] robotControlGRB = LEDHelpers.rgbtogrb(LEDHelpers.hsvToRgb(robotControl, 1, 1));

    enum LEDStates {
        NORMAL,
        PARTY,
        LOCATE,
        CHOOSEHUE
    };

    enum LEDGripperGameStates { //TODO
        NORMAL, //all four team colors slowly sliding
        INTAKING, //alternating magenta and green chaser with trail going inward
        OUTTAKING, //alternating magenta and green chaser with trail going outward
        CORAL, //solid white and yellow
        ALGAE, //solid teal
        LOCKED_CORAL, //flashing white and yellow
        LOCKED_ALGAE, //flashing teal
        CLIMB //slowly go from black to ocean color over course of endgame
    };

    enum LEDArmGameStates { //TODO
        NORMAL, //solid blue with 3 indicator LEDs at top. Green if nothings wrong, yellow warnings (yellow master alarms), red for critical errors (master alarms)
        CLIMB //same as gripper
    }

    private double chaserStatus = 0;

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
                    for (var i = 0; i < ledBuffer.getLength(); i++) {
                        double[] hsv = darkSkyHSV;
                        double[] rgb = LEDHelpers.hsvToRgb(hsv);
                        ledBuffer.setRGB(0, (int) rgb[0], (int) rgb[1], (int) rgb[2]); //time until end of competition
                    }
            }
        }

        // Depending on current LED state, send a new output to the LEDs
        switch (ledState) {
            case NORMAL:
                // Normal
                double[] rgb = LEDHelpers.hsvToRgb(stowBackground, 1, 0.63);
                switch (Robot.masterState) {
                    case CLMB:
                        break;
                    case SCOR:
                        break;
                    case FEED:
                        break;
                    case STOW:
                        break;
                }
                chaserStatus += 1;
                chaserStatus %= numberOfLEDs - 3;
                break;
            case PARTY:
                // Party mode
                
                break;
            case LOCATE:
                ledBuffer.setRGB(locateLED, (int) nominalChaserGRB[0], (int) nominalChaserGRB[1], (int) nominalChaserGRB[2]);
                break;
            case CHOOSEHUE:
                double[] chooseGRB = LEDHelpers.rgbtogrb(LEDHelpers.hsvToRgb(chosenHue, 1, 1));
                for (var i = 0; i < ledBuffer.getLength(); i++) {
                    // Sets the specified LED to the HSV values
                    ledBuffer.setRGB(i, (int) chooseGRB[0], (int) chooseGRB[1], (int) chooseGRB[2]);
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
