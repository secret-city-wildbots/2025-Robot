package frc.robot;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utility.ActuatorInterlocks;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Intake {
    public static boolean enabled = false;
    private boolean indexing = false;
    private double outerIntakePower;
    private double innerIntakePower;
    private double indexerIntakePower;
    public static boolean bbBroken = false;

    public double innerTemp = -1;
    public double frontTemp = -1;
    public double backTemp = -1;
    public double indexerTemp = -1;
    public double innerVelocity = -1;
    public double frontVelocity = -1;
    public double backVelocity = -1;
    public double indexerVelocity = -1;

    private final TalonFX inner = new TalonFX(19, "rio");
    private final TalonFX front = new TalonFX(24, "rio");
    private final TalonFX back = new TalonFX(25, "rio");
    private final CANSparkMax indexer = new CANSparkMax(18, MotorType.kBrushless);
    private final SparkAbsoluteEncoder indexEncoder = indexer.getAbsoluteEncoder();
    private final SparkLimitSwitch beamBreak = indexer.getForwardLimitSwitch(Type.kNormallyOpen);

    /**
     * Creates a new intake object
     * 
     * @param innerPower   The output power between -1 and 1 for the central intake
     * @param outerPower   The output power between -1 and 1 for the outer intakes
     * @param indexerPower The output power between -1 and 1 for the indexer
     */
    public Intake(
            double innerPower,
            double outerPower,
            double indexerPower) {
        outerIntakePower = outerPower;
        innerIntakePower = innerPower;
        indexerIntakePower = indexerPower;

        TalonFXConfiguration innerConfig = new TalonFXConfiguration();
        innerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        inner.getConfigurator().apply(innerConfig);

        beamBreak.enableLimitSwitch(true);
    }

    /**
     * Reads in sensor and motor values and stores them internally in the intake
     * object
     */
    public void updateSensors() {
        bbBroken = beamBreak.isPressed();
        Dashboard.beambreak.set(bbBroken);
        innerTemp = inner.getDeviceTemp().getValueAsDouble();
        frontTemp = front.getDeviceTemp().getValueAsDouble();
        backTemp = back.getDeviceTemp().getValueAsDouble();
        Dashboard.intakeTemps.set(new double[] { frontTemp, innerTemp, backTemp });
        frontVelocity = front.getVelocity().getValueAsDouble() / 60;
        innerVelocity = inner.getVelocity().getValueAsDouble() / 60;
        backVelocity = back.getVelocity().getValueAsDouble() / 60;
        Dashboard.intakeVelocities.set(new double[] { frontVelocity, innerVelocity, backVelocity });
        indexerVelocity = indexEncoder.getVelocity() / 60;
        Dashboard.indexerVelocity.set(indexerVelocity);
        indexerTemp = indexer.getMotorTemperature();
        Dashboard.indexerTemp.set(indexerTemp);
    }

    /**
     * Updates intake values in the intake object to use for motor outputs
     * 
     * @param driveController
     */
    public void updateIntake(XboxController driveController) {
        // Turns off intake and allow indexing when note is picked up
        if (bbBroken) {
            enabled = false;
            beamBreak.enableLimitSwitch(false);
        } else {
            beamBreak.enableLimitSwitch(true);
        }
        if (driveController.getRightBumperPressed()) {
            toggle();
        }
        indexing = (driveController.getLeftTriggerAxis() > 0.7) ||
                (enabled && (!bbBroken)) ||
                (bbBroken && (driveController.getRightTriggerAxis() > 0.7) && Shooter.spunUp);
    }

    /**
     * Directly toggles the intake on or off
     * Always toggles off if currently on
     * Only toggles on if no note is currently picked up and wrist and elevator are
     * stowed
     */
    public void toggle() {
        if (enabled) {
            enabled = false;
        } else if (!bbBroken) {
            if (Shooter.wristStowed && Elevator.stowed) {
                enabled = true;
            } else {
                enabled = false;
            }
        } else {
            enabled = false;
        }
    }

    /**
     * Uses stored values in the intake object to send to motors for outputs
     */
    public void updateOutputs() {
        Dashboard.intaking.set(enabled);
        ActuatorInterlocks.TAI_TalonFX_Power(inner, "Center_Intake_(p)", (enabled) ? innerIntakePower : 0);
        ActuatorInterlocks.TAI_TalonFX_Power(front, "Outer_Roller_Front_(p)", (enabled) ? outerIntakePower : 0);
        ActuatorInterlocks.TAI_TalonFX_Power(back, "Outer_Roller_Back_(p)", (enabled) ? outerIntakePower : 0);
        ActuatorInterlocks.TAI_SparkMAX_Power(indexer, "Indexer_(p)", (indexing) ? indexerIntakePower : 0);
    }
}
