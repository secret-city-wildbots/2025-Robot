package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
//import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Utility.ActuatorInterlocks;

public class Elevator {
    public static boolean stowed = true;
    public static boolean climbed = false;

    private double height = 0;
    private double motorTemp = 0;

    private final TalonFX elevator = new TalonFX(17, "rio");
    private TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    private final double elevatorRatio; // from inches to rotations, multiplier
    private final double elevatorFeedForward;
    private double elevatorArbitraryFFScalar = 0;
    private double elevatorOutput = 0; // inches
    private final double allowedError = 0.2; // inches

    private PIDController elevatorController = new PIDController(0.06, 0, 0);

    private MotionMagicConfigs motionMagicConfigs = elevatorConfig.MotionMagic;

    private boolean unlockElevator0 = false;

    // Prior loop kp, i, and d tracking
    @SuppressWarnings("unused")
    private double kp0 = 0.12;
    @SuppressWarnings("unused")
    private double ki0 = 0;
    @SuppressWarnings("unused")
    private double kd0 = 0;

    /**
     * Creates a new elevator object
     */
    public Elevator() {
        switch (Robot.robotProfile) {
            case "2024_Robot":
                elevatorRatio = 7.72;
                break;
            case "Steve2":
                elevatorRatio = 7.72;
                break;
            default:
                elevatorRatio = 7.72;
        }

        elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        elevatorConfig.Slot0.kP = elevatorController.getP();
        elevatorConfig.Slot0.kI = elevatorController.getI();
        elevatorConfig.Slot0.kD = elevatorController.getD();

        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        elevator.getConfigurator().apply(elevatorConfig);

        // gravity * mass * arbitrary FF scalar
        elevatorFeedForward = 32.17 * 20 * elevatorArbitraryFFScalar;
    }

    /**
     * Reads in motor values and stores them in the elevator object
     */
    public void updateSensors() {
        height = elevator.getPosition().getValueAsDouble() / elevatorRatio;
        Dashboard.elevatorPosition.set(height);
        motorTemp = elevator.getDeviceTemp().getValueAsDouble();
        Dashboard.elevatorTemp.set(motorTemp);
        climbed = Math.abs(height - elevatorOutput) < allowedError;
    }

    /**
     * Updates stored values in the elevator object to use for motor outputs
     */
    public void updateElevator() {
        switch (Robot.masterState) {
            case STOWED:
            case SHOOTING:
                elevatorOutput = 0; // inches
                break;
            case AMP:
                elevatorOutput = 5;
                break;
            case CLIMBING:
                elevatorOutput = 5;
                break;
            case TRAP:
                // idk what to put here yet, it depends on the trap sequence
                break;
        }
    }

    /**
     * Takes stored values in elevator object and send outputs to motors
     * Also sets brake/coast mode for elevator motor
     */
    public void updateOutputs() {
        /* PID tuning code START */
            // double kp = Dashboard.freeTuningkP.get();
            // double ki = Dashboard.freeTuningkI.get();
            // double kd = Dashboard.freeTuningkD.get();
            // if ((kp0 != kp) || (ki0 != ki) || (kd0 != kd)) {
            //     elevatorConfig.Slot0.kP = kp;
            //     elevatorConfig.Slot0.kI = ki;
            //     elevatorConfig.Slot0.kD = kd;
            //     this.elevator.getConfigurator().apply(elevatorConfig);
            //     kp0 = kp;
            //     ki0 = ki;
            //     kd0 = kd;
            // }
            /* PID tuning code END */

        ActuatorInterlocks.TAI_TalonFX_Position(elevator, "Elevator_(p)", Dashboard.freeTuningVariable.get() * elevatorRatio, elevatorFeedForward);

        // Put elevator in coast while unlocked and only when changed
        boolean unlockElevator = Dashboard.unlockElevator.get();
        if (unlockElevator != unlockElevator0) {
            if (unlockElevator) {
                elevator.setNeutralMode(NeutralModeValue.Coast);
            } else {
                elevator.setNeutralMode(NeutralModeValue.Brake);
            }
        }
        unlockElevator0 = unlockElevator;
    }
}
