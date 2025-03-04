package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Dashboard;
import frc.robot.Dashboard;
import frc.robot.Robot;
import frc.robot.Commands.ArmCommands;
import frc.robot.Robot.MasterStates;
import frc.robot.Utility.ActuatorInterlocks;

public class Arm extends SubsystemBase {
    // Constants
    private final double pivotRatio;
    private final double extenderRatio_m_to_rot;
    private final double wristRatio;
    private final double maxExtensionDistance_m;
    private final double maxForwardPivotAngle;
    private final double maxBackwardPivotAngle;
    private final double maxForwardWristAngle_rad;
    private final double maxBackwardWristAngle_rad;

    private final double maxAcceptableAngleError_rad = 1;
    private final double maxAcceptableExtensionError_m = Units.inchesToMeters(0.5);

    // Motors
    private final TalonFX pivot;
    private final TalonFX[] pivotFollowers = new TalonFX[3];
    private final TalonFX extender;
    TalonFXConfiguration extenderConfig;
    private final TalonFX extenderFollower;
    TalonFXConfiguration extenderFollowerConfig;

    private final SparkMax wrist;
    private final SparkAbsoluteEncoder wristEncoder;
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    // private double kp0 = 0.0;
    // private double ki0 = 0.0;
    // private double kd0 = 0.0;

    // Sensor values
    private Rotation2d pivotRotation = new Rotation2d();
    private double extenderPosition_m = 0.0;
    private Rotation2d wristRotation = new Rotation2d();

    // Encoder values
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0, 1, 0);

    // Outputs
    private Rotation2d pivotOutput = new Rotation2d();
    private double extenderOutput_m = 0.0;
    private Rotation2d wristOutput = new Rotation2d();
    private double wristFF = 0.0;
    private final double wristFFArbitraryScalar = 0.2;
    public static int scoreHeight = 1;
    public static int pickupHeight = 1;
    private final Trigger switchPieces = new Trigger(() -> Robot.scoreCoral);
    private final Trigger stowTrigger = new Trigger(() -> Robot.masterState.equals(MasterStates.STOW));
    private final Trigger scoreTrigger = new Trigger(() -> Robot.masterState.equals(MasterStates.SCOR));
    private final Trigger feedrTrigger = new Trigger(() -> Robot.masterState.equals(MasterStates.FEED));
    private final Trigger climbTrigger = new Trigger(() -> Robot.masterState.equals(MasterStates.CLMB));
    private final Trigger manipATrigger;
    private final Trigger manipXTrigger;
    private final Trigger manipYTrigger;
    private final Trigger manipBTrigger;

    public Arm() {
        switch (Robot.robotProfile) {
            case "2025_Robot":
            // Temp values
                pivotRatio = 
                    (84.0 / 8.0) * // First gear reduction
                    (18.0 / 1.6); // Capstan reduction, Total reduction: 118.13:1
                maxForwardPivotAngle = Units.degreesToRadians(90);
                maxBackwardPivotAngle = Units.degreesToRadians(-90);

                extenderRatio_m_to_rot = 1.0 / Units.inchesToMeters( // Convert to meters for use elsewhere
                    (1.0 / 5.25) *  // ratio of motor rotations to spool rotations
                    Math.PI * 1.432 // Spool diameter (in) -> circumference (in)
                    * 2.0 // Two stages moving together doubles movement
                );
                maxExtensionDistance_m = Units.inchesToMeters(43);

                wristRatio = 
                    70.0 * // Neo reduction
                    (34.0 / 12.0); // Small herringbone to big herringbone
                maxForwardWristAngle_rad = Units.degreesToRadians(112);
                maxBackwardWristAngle_rad = Units.degreesToRadians(-125);
                break;
            case "COTS_Testbed":
            // Temp values
                pivotRatio = 
                    (84.0 / 8.0) * // First gear reduction
                    (18.0 / 1.55); // Capstan reduction, Total reduction: 121.935:1
                maxForwardPivotAngle = Units.degreesToRadians(90);
                maxBackwardPivotAngle = Units.degreesToRadians(-90);

                extenderRatio_m_to_rot = 1.0 / Units.inchesToMeters( // Convert to meters for use elsewhere
                    (1.0 / 5.25) *  // ratio of motor rotations to spool rotations
                    Math.PI * 1.432 // Spool diameter (in) -> circumference (in)
                    * 2.0 // Two stages moving together doubles movement
                );
                maxExtensionDistance_m = Units.inchesToMeters(43);

                wristRatio = 
                    70.0 * // Neo reduction
                    (34.0 / 12.0); // Small herringbone to big herringbone
                maxForwardWristAngle_rad = Units.degreesToRadians(112);
                maxBackwardWristAngle_rad = Units.degreesToRadians(-125);
                break;
            default:
            // Temp values
                pivotRatio = 
                    (84.0 / 8.0) * // First gear reduction
                    (18.0 / 1.55); // Capstan reduction, Total reduction: 121.935:1
                maxForwardPivotAngle = Units.degreesToRadians(90);
                maxBackwardPivotAngle = Units.degreesToRadians(-90);

                extenderRatio_m_to_rot = 1.0 / Units.inchesToMeters( // Convert to meters for use elsewhere
                    (1.0 / 5.25) *  // ratio of motor rotations to spool rotations
                    Math.PI * 1.432 // Spool diameter (in) -> circumference (in)
                    * 2.0 // Two stages moving together doubles movement
                );
                maxExtensionDistance_m = Units.inchesToMeters(43);

                wristRatio = 
                    70.0 * // Neo reduction
                    (34.0 / 12.0); // Small herringbone to big herringbone
                maxForwardWristAngle_rad = Units.degreesToRadians(112);
                maxBackwardWristAngle_rad = Units.degreesToRadians(-125);
                break;
        }

        // Pivot and pivot follower configurations
        pivot = new TalonFX(30);
        pivot.getDutyCycle().setUpdateFrequency(50);
        pivot.getMotorVoltage().setUpdateFrequency(50);
        pivot.getTorqueCurrent().setUpdateFrequency(50);
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Temp values
        pivotConfig.Slot0.kP = 0.0;
        pivotConfig.Slot0.kI = 0.0;
        pivotConfig.Slot0.kD = 0.0;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 
            Units.radiansToRotations(maxForwardPivotAngle) * pivotRatio;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 
            Units.radiansToRotations(maxBackwardPivotAngle) * pivotRatio;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivot.getConfigurator().apply(pivotConfig);        
        
        TalonFXConfiguration[] pivotFollowerConfigs = new TalonFXConfiguration[3];
        for (int i = 0; i<3; i++) {
            pivotFollowers[i] = new TalonFX(31+i);
            pivotFollowerConfigs[i] = new TalonFXConfiguration();
            pivotFollowerConfigs[i].MotorOutput.NeutralMode = NeutralModeValue.Brake;
            pivotFollowers[i].getConfigurator().apply(pivotFollowerConfigs[i]);
            // Check if motor is on the same side as the master (30). If so, follow master with same configuration
            if (i==0) {
                // motor 31 will be on the same side as the master motor.
                pivotFollowers[i].setControl(new Follower(30, false));
            } else {
                // motors 32 and 33 will be on the opposite side as the master motor requiring the follower to oppose the
                // master direction
                pivotFollowers[i].setControl(new Follower(30, true));
            }
            
        }

        // // Extender configurations
        extender = new TalonFX(34);
        // extender.setPosition(Units.inchesToMeters(-1) * extenderRatio_m_to_rot);
        // extenderConfig = new TalonFXConfiguration();
        // extenderConfig.MotorOutput.PeakForwardDutyCycle = 0.3;
        // extenderConfig.MotorOutput.PeakReverseDutyCycle = -0.3;
        // extenderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // extenderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // // Temp values
        // extenderConfig.Slot0.kP = 0.17;
        // extenderConfig.Slot0.kI = 0.0;
        // extenderConfig.Slot0.kD = 0.0;
        // extenderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 
        //     maxExtensionDistance_m * extenderRatio_m_to_rot;
        // extenderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 
        //     0.0;
        // extenderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // extenderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // extender.getConfigurator().apply(extenderConfig);

        extenderFollower = new TalonFX(35);
        // extenderFollowerConfig = new TalonFXConfiguration();
        // extenderFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // extenderFollower.getConfigurator().apply(extenderFollowerConfig);
        // extenderFollower.setControl(new Follower(34, false));

        // // Wrist configurations
        wrist = new SparkMax(36, MotorType.kBrushless);
        wristEncoder = wrist.getAbsoluteEncoder();
        // wrist.getEncoder().setPosition(wristEncoder.getPosition() - 198.333);
        // wristConfig.idleMode(IdleMode.kBrake);
        // wristConfig.inverted(true);
        // wristConfig.closedLoop.pid(0.05,0.0,0.0);
        // wristConfig.closedLoop.maxOutput(0.4);
        // wristConfig.closedLoop.minOutput(-0.4);
        // wristConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        // wristConfig.closedLoop.positionWrappingEnabled(true);
        // wristConfig.closedLoop.positionWrappingInputRange(0, 198.333);
        // wristConfig.absoluteEncoder.inverted(true);
        // wristConfig.absoluteEncoder.zeroOffset(0.8068750);
        // wristConfig.absoluteEncoder.positionConversionFactor(198.333);
        // // wristConfig.softLimit.reverseSoftLimit(Units.radiansToRotations(maxBackwardWristAngle_rad)*wristRatio);
        // // wristConfig.softLimit.forwardSoftLimit(Units.radiansToRotations(maxForwardWristAngle_rad)*wristRatio);
        // // wristConfig.softLimit.reverseSoftLimitEnabled(true);
        // // wristConfig.softLimit.forwardSoftLimitEnabled(true);
        // wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        // Trigger configurations
        switchPieces.onChange(Commands.runOnce(this::switchPiece, this));
        stowTrigger.onTrue(Commands.runOnce(this::stow, this));
        scoreTrigger.onTrue(ArmCommands.score(this));
        feedrTrigger.onTrue(ArmCommands.pickup(this));
        climbTrigger.onTrue(ArmCommands.climb(this));
        manipATrigger = Robot.manipCommandController.a();
        manipATrigger.onTrue(Commands.runOnce(() -> {
            if (Robot.scoreCoral) { 
                scoreHeight = 2;
                if (Robot.masterState.equals(MasterStates.SCOR)) {ArmCommands.score(this).schedule();}
            } else { 
                pickupHeight = 2;
                if (Robot.masterState.equals(MasterStates.FEED)) {ArmCommands.pickup(this).schedule();}
            }
        }, this));
        manipXTrigger = Robot.manipCommandController.x();
        manipXTrigger.onTrue(Commands.runOnce(() -> {
            if (Robot.scoreCoral) { 
                scoreHeight = 3;
                if (Robot.masterState.equals(MasterStates.SCOR)) {ArmCommands.score(this).schedule();} 
            } else { 
                pickupHeight = 3;
                if (Robot.masterState.equals(MasterStates.FEED)) {ArmCommands.pickup(this).schedule();}
            }
        }, this));
        manipYTrigger = Robot.manipCommandController.y();
        manipYTrigger.onTrue(Commands.runOnce(() -> {
            if (Robot.scoreCoral) { scoreHeight = 4; } else { scoreHeight = 6;}
            if (Robot.masterState.equals(MasterStates.SCOR)) {ArmCommands.score(this).schedule();};
        }, this));
        manipBTrigger = Robot.manipCommandController.b();
        manipBTrigger.onTrue(Commands.runOnce(() -> {
            if (Robot.scoreCoral) { scoreHeight = 1; } else { scoreHeight = 5;}
            if (Robot.masterState.equals(MasterStates.SCOR)) {ArmCommands.score(this).schedule();};
        }, this));

        stow();
    }

    public void updateSensors(XboxController manipController) {
        pivotRotation = new Rotation2d();
        // pivotRotation = Rotation2d.fromRotations(pivot.getRotorPosition().getValueAsDouble() / pivotRatio);
        
        // extenderPosition_m = 0.0;
        extenderPosition_m = extender.getPosition().getValueAsDouble() / extenderRatio_m_to_rot;
        
        wristRotation = Rotation2d.fromRotations(wristEncoder.getPosition() / wristRatio);
        wristFF = pivotRotation.plus(wristRotation).getSin() * wristFFArbitraryScalar;

        // PID Tuning
        // double kp = Dashboard.freeTuningkP.get();
        // double ki = Dashboard.freeTuningkI.get();
        // double kd = Dashboard.freeTuningkD.get();
        // if ((kp0 != kp) || (ki0 != ki) || (kd0 != kd)) {
        // wristConfig.closedLoop.p(kp);
        // wristConfig.closedLoop.i(ki);
        // wristConfig.closedLoop.d(kd);
        // wrist.configure(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // kp0 = kp;
        // ki0 = ki;
        // kd0 = kd;
        // }
        // Dashboard.pidTuningGoalActual.set(new double[] { wristOutput.getDegrees(), wristRotation.getDegrees() });
    
        // PID Tuning
        // double kp = Dashboard.freeTuningkP.get();
        // double ki = Dashboard.freeTuningkI.get();
        // double kd = Dashboard.freeTuningkD.get();
        // if ((kp0 != kp) || (ki0 != ki) || (kd0 != kd)) {
        //     extenderConfig.Slot0.kP = kp;
        //     extenderConfig.Slot0.kI = ki;
        //     extenderConfig.Slot0.kD = kd;
        //     extender.getConfigurator().apply(extenderConfig);
        //     kp0 = kp;
        //     ki0 = ki;
        //     kd0 = kd;
        // }
        // Dashboard.pidTuningGoalActual.set(new double[] { Dashboard.freeTuningVariable.get(), Units.metersToInches(extenderPosition_m) });
    }

    public void switchPiece() {
        switch (scoreHeight) {
            case 1, 2:
                // Convert to low algae and processor
                scoreHeight = 5;
                pickupHeight = 2;
                break;
            case 3, 4:
                // Convert to high algae and barge
                scoreHeight = 6;
                pickupHeight = 3;
                break;
            case 5, 6:
                // Convert to low coral and feeder settings
                scoreHeight = 2;
                pickupHeight = 1;
        }
    }

    public boolean hasArrived() {
        boolean hasArrived = false;
        if((Math.abs(pivotRotation.getRadians() - pivotOutput.getRadians())) < maxAcceptableAngleError_rad) {
            if((Math.abs(extenderPosition_m - extenderOutput_m)) < maxAcceptableExtensionError_m) {
                if ((Math.abs(wristRotation.getRadians() - wristOutput.getRadians())) < maxAcceptableAngleError_rad) {
                    hasArrived = true;
                }}}
        return hasArrived;
    }

    public void stow() {
        updateArm(0, 0, Rotation2d.fromDegrees(40));
    }

    public void autoStow() {
        stow();
    }

    public void score() {
        switch (scoreHeight) {
            case 1:
                scoreL1();
                break;
            
            case 2:
                scoreL2();
                break;
            
            case 3:
                scoreL3();
                break;
    
            case 4:
                scoreL4();
                break;

            case 5:
                scoreProcessor();
                break;

            case 6:
                scoreBarge();
                break;

            default:
            new Exception("Score type out of range :(").printStackTrace();
                break;
        }
    }

    private void scoreL4() {
        updateArm(
            Units.inchesToMeters(37.1), 
            Rotation2d.fromDegrees(-5), 
            Rotation2d.fromDegrees(68));
    }
    private void scoreL3() {
        updateArm(
            Units.inchesToMeters(0), 
            Rotation2d.fromDegrees(-11), 
            Rotation2d.fromDegrees(38));}

    private void scoreL2() {
        updateArm(
            Units.inchesToMeters(0), 
            Rotation2d.fromDegrees(-10), 
            Rotation2d.fromDegrees(83));}
    
    private void scoreL1() {
        // Temp values
        updateArm(0.0, 0.0, Rotation2d.fromDegrees(40));
    }
    private void scoreProcessor() {
        updateArm(
            Units.inchesToMeters(0), 
            Rotation2d.fromDegrees(-62), 
            Rotation2d.fromDegrees(-125));
    }
    private void scoreBarge() {
        updateArm(
            Units.inchesToMeters(42), 
            Rotation2d.fromDegrees(0), 
            Rotation2d.fromDegrees(-10));}

    public void pickup() {
        switch (pickupHeight) {
            case 1:
                pickupFeeder();
                break;

            case 2:
                pickupLowAlgae();
                break;

            case 3:
                pickupHighAlgae();
                break;

            default:
                new Exception("Pickup type out of range :(").printStackTrace();
                break;
        }
    }

    private void pickupFeeder() {
        updateArm(
            Units.inchesToMeters(0), 
            Rotation2d.fromDegrees(-5.5), 
            Rotation2d.fromDegrees(-125));
        }
    private void pickupLowAlgae() {
        updateArm(
            Units.inchesToMeters(0), 
            Rotation2d.fromDegrees(-4.7), 
            Rotation2d.fromDegrees(60));
        }
    private void pickupHighAlgae() {
        updateArm(
            Units.inchesToMeters(9.64), 
            Rotation2d.fromDegrees(-3), 
            Rotation2d.fromDegrees(35));
        }

    public void climbInit() {
        updateArm(
            Units.inchesToMeters(0), 
            Rotation2d.fromDegrees(-19), 
            Rotation2d.fromDegrees(40));
        }

    public void climbLift() {
        updateArm(
            Units.inchesToMeters(37.1), 
            Rotation2d.fromDegrees(-5), 
            Rotation2d.fromDegrees(10));
        }

    /**
     * Takes in a forward and upward distance along with a wrist angle and calculates the correct outputs
     * @param forwardDistance_m The horizontal distance the wrist pivot should be from the base pivot
     * @param upwardDistance_m The vertical distance the wrist pivot should be form the base pivot
     * @param wristAngle The angle the wrist should be relative to vertical (not accounting for pivot angle)
     */
    private void updateArm(double forwardDistance_m, double upwardDistance_m, Rotation2d wristAngle) {
        Rotation2d rotation = new Rotation2d();
        if (upwardDistance_m > 1e-6 && forwardDistance_m > 1e-6) {
            rotation = new Rotation2d(upwardDistance_m, forwardDistance_m);
        }

        updateArm(
            Math.hypot(forwardDistance_m, upwardDistance_m), 
            rotation, wristAngle.minus(rotation)
        );
    }

    /**
     * Takes in a pivot and wrist angle and an extension distance directly output to motors
     * @param extensionDistance_m The distance radially the extender should go to
     * @param pivotAngle The angle relative to vertical the pivot should go to
     * @param wristAngle The angle relative to the pivot the wrist should go to
     */
    private void updateArm(double extensionDistance_m, Rotation2d pivotAngle, Rotation2d wristAngle) {
        pivotOutput = new Rotation2d(
            MathUtil.clamp(pivotAngle.getRadians(), 
            maxBackwardPivotAngle, maxForwardPivotAngle));
        
        extenderOutput_m = 
            MathUtil.clamp(extensionDistance_m, 
            0.0, maxExtensionDistance_m);

        wristOutput = new Rotation2d(
            MathUtil.clamp(wristAngle.getRadians(), 
            maxBackwardWristAngle_rad, maxForwardWristAngle_rad));
    }

    boolean unlockExtender0 = false;
    public void updateOutputs() {
        System.out.println(encoder.get());
        ActuatorInterlocks.testActuatorInterlocks(
            pivot, "Pivot_(p)", 
            pivotOutput.getRotations() * pivotRatio, 0.0);

        ActuatorInterlocks.testActuatorInterlocks(
            extender, "Extender_(p)", 
            extenderOutput_m * extenderRatio_m_to_rot, 0.03);

        boolean unlockExtender = Dashboard.unlockExtender.get();
        if (unlockExtender && (!unlockExtender0)) {
            extenderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            extender.getConfigurator().apply(extenderConfig);
            extenderFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            extenderFollower.getConfigurator().apply(extenderFollowerConfig);
        } else if ((!unlockExtender) && unlockExtender0) {
            extenderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            extender.getConfigurator().apply(extenderConfig);
            extenderFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            extenderFollower.getConfigurator().apply(extenderFollowerConfig);
        }
        unlockExtender0 = unlockExtender;
            
        ActuatorInterlocks.testActuatorInterlocks(
            wrist, "Wrist_(p)", 
            wristOutput.getRotations() * wristRatio, -wristFF);
    }
}
