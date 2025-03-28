package frc.robot.Subsystems;

// import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
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
    private final double maxForwardPivotAngle_rad;
    private final double maxBackwardPivotAngle_rad;
    private final double maxForwardWristAngle_rad;
    private final double maxBackwardWristAngle_rad;

    private final double maxAcceptableAngleError_rad = Units.degreesToRadians(1.5);
    private final double maxAcceptableExtensionError_m = Units.inchesToMeters(0.5);
    private final double closeEnoughAngleError_rad = Units.degreesToRadians(7);
    private final double closeEnoughExtensionError_m = Units.inchesToMeters(5);

    // Motors
    private final TalonFX pivot;
    private final TalonFX[] pivotFollowers = new TalonFX[3];
    private TalonFXConfiguration pivotConfig;
    private TalonFXConfiguration[] pivotFollowerConfigs;
    private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0, 1, 0);
    private final double pivotEncoderOffset_rot = 0.332;
    private final PIDController pivotPID = new PIDController(8.0, 0.0, 0.0);

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
    private double extenderPosition_m = 0.0;
    public static Rotation2d wristRotation = new Rotation2d();
    private boolean disablePivot = false;
    private boolean disableExtender = false;
    private boolean disableWrist = false;
    private DigitalInput unlockButton = new DigitalInput(1);

    // Outputs
    private Rotation2d pivotOutput = new Rotation2d();
    private double pivotFF = 0.0;
    private double extenderOutput_m = 0.0;
    private Rotation2d wristOutput = new Rotation2d();
    private double wristFF = 0.0;
    private final double wristFFArbitraryScalar = 0.2;
    public static int scoreHeight = 1;
    public static int pickupHeight = 1;
    private final Trigger driveLBTrigger;
    private final Trigger manipLBTrigger;
    private final Trigger manipATrigger;
    private final Trigger manipXTrigger;
    private final Trigger manipYTrigger;
    private final Trigger manipBTrigger;
    private final Trigger coralTrigger;
    // private final Trigger stowTrigger = new Trigger(() -> Robot.masterState.equals(MasterStates.STOW));
    private final Trigger scoreTrigger = new Trigger(() -> Robot.masterState.equals(MasterStates.SCOR));
    private final Trigger feedTrigger = new Trigger(() -> Robot.masterState.equals(MasterStates.FEED));
    private final Trigger climbTrigger = new Trigger(() -> Robot.masterState.equals(MasterStates.CLMB));

    public Arm() {
        pivotPID.enableContinuousInput(0,1);
        switch (Robot.robotProfile) {
            case "2025_Robot":
                // Temp values
                pivotRatio = (84.0 / 8.0) * // First gear reduction
                        (18.0 / 1.6); // Capstan reduction, Total reduction: 118.13:1
                maxForwardPivotAngle_rad = Units.degreesToRadians(12);
                maxBackwardPivotAngle_rad = Units.degreesToRadians(-110);

                extenderRatio_m_to_rot = 1.0 / Units.inchesToMeters( // Convert to meters for use elsewhere
                        (1.0 / 5.25) * // ratio of motor rotations to spool rotations
                                Math.PI * 1.432 // Spool diameter (in) -> circumference (in)
                                * 2.0 // Two stages moving together doubles movement
                );
                maxExtensionDistance_m = Units.inchesToMeters(43);

                wristRatio = 99.16667;
                // 35.0 * // Neo reduction
                // (34.0 / 12.0); // Small herringbone to big herringbone
                maxForwardWristAngle_rad = Units.degreesToRadians(112);
                maxBackwardWristAngle_rad = Units.degreesToRadians(-115);
                break;
            case "COTS_Testbed":
                // Temp values
                pivotRatio = (84.0 / 8.0) * // First gear reduction
                        (18.0 / 1.6); // Capstan reduction, Total reduction: 118.13:1
                maxForwardPivotAngle_rad = Units.degreesToRadians(12);
                maxBackwardPivotAngle_rad = Units.degreesToRadians(-110);

                extenderRatio_m_to_rot = 1.0 / Units.inchesToMeters( // Convert to meters for use elsewhere
                        (1.0 / 5.25) * // ratio of motor rotations to spool rotations
                                Math.PI * 1.432 // Spool diameter (in) -> circumference (in)
                                * 2.0 // Two stages moving together doubles movement
                );
                maxExtensionDistance_m = Units.inchesToMeters(43);

                wristRatio = 99.16667;
                // 35.0 * // Neo reduction
                // (34.0 / 12.0); // Small herringbone to big herringbone
                maxForwardWristAngle_rad = Units.degreesToRadians(112);
                maxBackwardWristAngle_rad = Units.degreesToRadians(-115);
                break;
            default:
                // Temp values
                pivotRatio = (84.0 / 8.0) * // First gear reduction
                        (18.0 / 1.6); // Capstan reduction, Total reduction: 118.13:1
                maxForwardPivotAngle_rad = Units.degreesToRadians(12);
                maxBackwardPivotAngle_rad = Units.degreesToRadians(-110);

                extenderRatio_m_to_rot = 1.0 / Units.inchesToMeters( // Convert to meters for use elsewhere
                        (1.0 / 5.25) * // ratio of motor rotations to spool rotations
                                Math.PI * 1.432 // Spool diameter (in) -> circumference (in)
                                * 2.0 // Two stages moving together doubles movement
                );
                maxExtensionDistance_m = Units.inchesToMeters(43);

                wristRatio = 99.16667;
                // 35.0 * // Neo reduction
                // (34.0 / 12.0); // Small herringbone to big herringbone
                maxForwardWristAngle_rad = Units.degreesToRadians(112);
                maxBackwardWristAngle_rad = Units.degreesToRadians(-115);
                break;
        }

        // Pivot and pivot follower configurations
        pivot = new TalonFX(30);
        pivot.setPosition(getPivotEncoderPosition().getRotations() * pivotRatio);
        pivot.getDutyCycle().setUpdateFrequency(50);
        pivot.getMotorVoltage().setUpdateFrequency(50);
        pivot.getTorqueCurrent().setUpdateFrequency(50);
        pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.PeakForwardDutyCycle = 0.1;
        pivotConfig.MotorOutput.PeakReverseDutyCycle = -0.1;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.radiansToRotations(maxForwardPivotAngle_rad)
                * pivotRatio;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.radiansToRotations(maxBackwardPivotAngle_rad)
                * pivotRatio;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivot.getConfigurator().apply(pivotConfig);

        pivotFollowerConfigs = new TalonFXConfiguration[3];
        for (int i = 0; i < 3; i++) {
            pivotFollowers[i] = new TalonFX(31 + i);
            pivotFollowerConfigs[i] = new TalonFXConfiguration();
            pivotFollowerConfigs[i].MotorOutput.NeutralMode = NeutralModeValue.Brake;
            pivotFollowers[i].getConfigurator().apply(pivotFollowerConfigs[i]);
            // Check if motor is on the same side as the master (30). If so, follow master
            // with same configuration
            if (i == 0) {
                // motor 31 will be on the same side as the master motor.
                pivotFollowers[i].setControl(new Follower(30, false));
            } else {
                // motors 32 and 33 will be on the opposite side as the master motor requiring
                // the follower to oppose the
                // master direction
                pivotFollowers[i].setControl(new Follower(30, true));
            }

        }

        // // Extender configurations
        extender = new TalonFX(34);
        extender.setPosition(Units.inchesToMeters(-1.0) * extenderRatio_m_to_rot);
        extenderConfig = new TalonFXConfiguration();
        extenderConfig.MotorOutput.PeakForwardDutyCycle = 0.4;
        extenderConfig.MotorOutput.PeakReverseDutyCycle = -0.25;
        extenderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        extenderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extenderConfig.Slot0.kP = 0.19;
        extenderConfig.Slot0.kI = 0.0;
        extenderConfig.Slot0.kD = 0.0;
        extenderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxExtensionDistance_m * extenderRatio_m_to_rot;
        extenderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.inchesToMeters(-0.5) * extenderRatio_m_to_rot;
        extenderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        extenderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        extender.getConfigurator().apply(extenderConfig);

        extenderFollower = new TalonFX(35);
        extenderFollowerConfig = new TalonFXConfiguration();
        extenderFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extenderFollower.getConfigurator().apply(extenderFollowerConfig);
        extenderFollower.setControl(new Follower(34, false));

        // // Wrist configurations
        wrist = new SparkMax(36, MotorType.kBrushless);
        wristEncoder = wrist.getAbsoluteEncoder();
        wrist.getEncoder().setPosition(-wristEncoder.getPosition() - wristRatio);
        wristConfig.idleMode(IdleMode.kBrake);
        wristConfig.inverted(true);
        wristConfig.closedLoop.pid(0.07, 0.0, 0.0);
        wristConfig.closedLoop.maxOutput(0.3);
        wristConfig.closedLoop.minOutput(-0.3);
        wristConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        wristConfig.closedLoop.positionWrappingEnabled(true);
        wristConfig.closedLoop.positionWrappingInputRange(0, wristRatio);
        wristConfig.absoluteEncoder.inverted(false);

        wristConfig.absoluteEncoder.zeroOffset(0.5438459993);
        wristConfig.absoluteEncoder.positionConversionFactor(wristRatio);
        // wristConfig.softLimit.reverseSoftLimit(Units.radiansToRotations(maxBackwardWristAngle_rad)*wristRatio);
        // wristConfig.softLimit.forwardSoftLimit(Units.radiansToRotations(maxForwardWristAngle_rad)*wristRatio);
        // wristConfig.softLimit.reverseSoftLimitEnabled(true);
        // wristConfig.softLimit.forwardSoftLimitEnabled(true);
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        scoreTrigger.onTrue(Commands.runOnce(() -> score().schedule()));
        feedTrigger.onTrue(Commands.runOnce(() -> pickup().schedule()));
        climbTrigger.onTrue(Commands.runOnce(() -> ArmCommands.climb(this).schedule()));
        coralTrigger = new Trigger(() -> Robot.scoreCoral);
        coralTrigger.onTrue(Commands.runOnce(() -> {scoreHeight = 1; pickupHeight = 1;}));
        coralTrigger.onFalse(Commands.runOnce(() -> {scoreHeight = 6; pickupHeight = 2;}));
        driveLBTrigger = Robot.driverCommandController.leftBumper();
        driveLBTrigger.onTrue(
            Commands.runOnce(() -> ArmCommands.groundPickup(this).schedule())
        );
        manipLBTrigger = Robot.manipCommandController.leftBumper();
        manipLBTrigger.onTrue(
        Commands.either(
            Commands.either(
                Commands.runOnce(() -> drivingStow().schedule()),
                Commands.runOnce(() -> scoringStow().schedule()),
                () -> wristRotation.getDegrees() < 50
            ),
            Commands.runOnce(()-> carefulStow().schedule()),
            () -> !Robot.masterState0.equals(MasterStates.FEED)
        ));
        manipATrigger = Robot.manipCommandController.a();
        manipBTrigger = Robot.manipCommandController.b();
        manipXTrigger = Robot.manipCommandController.x();
        manipYTrigger = Robot.manipCommandController.y();
        manipATrigger.onTrue(
            Commands.either(
                Commands.runOnce(() -> {
                    scoreHeight = 2;
                    Dashboard.scoringState.set(1);
                }).andThen(ArmCommands.score(this).onlyIf(() -> Robot.masterState.equals(MasterStates.SCOR))),
                Commands.runOnce(() -> {
                    pickupHeight = 2;
                }).andThen(ArmCommands.pickup(this).onlyIf(() -> Robot.masterState.equals(MasterStates.FEED))),
                () -> Robot.scoreCoral
            )
        );
        manipBTrigger.onTrue(
            Commands.either(
                Commands.runOnce(() -> {
                    scoreHeight = 1;
                    Dashboard.scoringState.set(0);
                }).andThen(ArmCommands.score(this).onlyIf(() -> Robot.masterState.equals(MasterStates.SCOR))),
                Commands.runOnce(() -> {
                    scoreHeight = 5;
                }).andThen(ArmCommands.score(this).onlyIf(() -> Robot.masterState.equals(MasterStates.SCOR))),
                () -> Robot.scoreCoral
            )
        );
        manipXTrigger.onTrue(
            Commands.either(
                Commands.runOnce(() -> {
                    scoreHeight = 3;
                    Dashboard.scoringState.set(2);
                }).andThen(ArmCommands.score(this).onlyIf(() -> Robot.masterState.equals(MasterStates.SCOR))),
                Commands.runOnce(() -> {
                    pickupHeight = 3;
                }).andThen(ArmCommands.pickup(this).onlyIf(() -> Robot.masterState.equals(MasterStates.FEED))),
                () -> Robot.scoreCoral
            )
        );
        manipYTrigger.onTrue(
            Commands.either(
                Commands.runOnce(() -> {
                    scoreHeight = 4;
                    Dashboard.scoringState.set(3);
                }).andThen(ArmCommands.score(this).onlyIf(() -> Robot.masterState.equals(MasterStates.SCOR))),
                Commands.runOnce(() -> {
                    scoreHeight = 6;
                }).andThen(ArmCommands.score(this).onlyIf(() -> Robot.masterState.equals(MasterStates.SCOR))),
                () -> Robot.scoreCoral
            )
        );
        drivingStow().schedule();
    }

    private Rotation2d getPivotEncoderPosition() {
        double encoderRotations = pivotEncoderOffset_rot - pivotEncoder.get();
        return Rotation2d.fromRotations(encoderRotations);
    }

    public boolean hasArrived() {
        boolean hasArrived = false;
        if ((Math.abs(getPivotEncoderPosition().getRadians() - pivotOutput.getRadians())) < maxAcceptableAngleError_rad) {
            if ((Math.abs(extenderPosition_m - extenderOutput_m)) < maxAcceptableExtensionError_m) {
                double realWristRotation_rad = wristRotation.getRadians();
                if (realWristRotation_rad > Math.PI) {realWristRotation_rad -= Math.PI * 2;}
                if ((Math.abs(realWristRotation_rad - wristOutput.getRadians())) < maxAcceptableAngleError_rad) {
                    hasArrived = true;
                }
            }
        }
        return hasArrived;
    }

    public boolean closeEnough() {
        boolean hasArrived = false;
        if ((Math.abs(getPivotEncoderPosition().getRadians() - pivotOutput.getRadians())) < closeEnoughAngleError_rad) {
            System.out.println("pivot");
            if ((Math.abs(extenderPosition_m - extenderOutput_m)) < closeEnoughExtensionError_m) {
                System.out.println("extender");
                double realWristRotation_rad = wristRotation.getRadians();
                if (realWristRotation_rad > Math.PI) {realWristRotation_rad -= Math.PI * 2;}
                if ((Math.abs(realWristRotation_rad - wristOutput.getRadians())) < closeEnoughAngleError_rad) {
                    System.out.println("wrist");
                    hasArrived = true;
                }
            }
        }
        return hasArrived;
    }

    public void updateSensors() {
        if (Dashboard.calibrateExtender.get()) {
            extender.setPosition(Units.inchesToMeters(-1.0) * extenderRatio_m_to_rot);
        }
        extenderPosition_m = extender.getPosition().getValueAsDouble() / extenderRatio_m_to_rot;
        Dashboard.extenderPosition_in.set(Units.metersToInches(extenderPosition_m));

        wristRotation = Rotation2d.fromRotations(wristEncoder.getPosition() / wristRatio);
        wristFF = getPivotEncoderPosition().plus(wristRotation).getSin() * wristFFArbitraryScalar;
        double realWristRotation_deg = wristRotation.getDegrees();
        if (realWristRotation_deg > 180) {realWristRotation_deg -= 360;}
        if (realWristRotation_deg < -180) {realWristRotation_deg += 360;}
        Dashboard.wristPosition_deg.set(realWristRotation_deg);

        if (getPivotEncoderPosition().getDegrees() < -15) {
            pivotFF = -getPivotEncoderPosition().getSin()*0.016;
            // pivotFF = 0.0;
        }

        Dashboard.pivotPosition_deg.set(getPivotEncoderPosition().getDegrees());

        Dashboard.extenderTemp_C.set(extender.getDeviceTemp().getValueAsDouble());
        Dashboard.wristTemp_C.set(wrist.getMotorTemperature());
        Dashboard.pivotTemp_C.set(pivot.getDeviceTemp().getValueAsDouble());

        /* PID tuning code START */
            // double kp = Dashboard.freeTuningkP.get();
            // double ki = Dashboard.freeTuningkI.get();
            // double kd = Dashboard.freeTuningkD.get();
            // if ((kp0 != kp) || (ki0 != ki) || (kd0 != kd)) {
            // wristConfig.closedLoop.pid(kp, ki, kd);
            // wrist.configure(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            // kp0 = kp;
            // ki0 = ki;
            // kd0 = kd;
            // }
            // Dashboard.pidTuningGoalActual.set(new double[] {wristOutput.getDegrees(), wristRotation.getDegrees()});
        /* PID tuning code END */
    }


    private void updateArm(double extensionDistance_m, Rotation2d pivotAngle, Rotation2d wristAngle) {
        pivotOutput = new Rotation2d(
                MathUtil.clamp(pivotAngle.getRadians(),
                        maxBackwardPivotAngle_rad, maxForwardPivotAngle_rad));

        extenderOutput_m = MathUtil.clamp(extensionDistance_m,
                -0.5, maxExtensionDistance_m);

        wristOutput = new Rotation2d(
                MathUtil.clamp(wristAngle.getRadians(),
                        maxBackwardWristAngle_rad, maxForwardWristAngle_rad));
    }

    public void updatePivot(Rotation2d pivotAngle) {
        pivotOutput = new Rotation2d(
                MathUtil.clamp(pivotAngle.getRadians(),
                        maxBackwardPivotAngle_rad, maxForwardPivotAngle_rad));
    }

    public void updateWrist(Rotation2d wristAngle) {
        wristOutput = new Rotation2d(
                MathUtil.clamp(wristAngle.getRadians(),
                        maxBackwardWristAngle_rad, maxForwardWristAngle_rad));
    }


    public void updateExtender(double extensionDistance_m) {
        extenderOutput_m = MathUtil.clamp(extensionDistance_m,
                -0.5, maxExtensionDistance_m);
    }

    boolean unlockExtender0 = false;
    boolean unlockPivot0 = false;
    boolean unlockWrist0 = false;
    private Rotation2d pivotOutput0 = new Rotation2d();
    private double extenderOutput0 = 0.0;
    private Rotation2d wristOutput0 = new Rotation2d();

    public void updateOutputs() {
        Dashboard.armArrived.set(hasArrived());

        // Pivot 
        double finalPivotOutput_rot = 0.0;
        if (disablePivot) {
            finalPivotOutput_rot = pivotOutput0.getRotations();
        } else {
            finalPivotOutput_rot = pivotOutput.getRotations();
            pivotOutput0 = pivotOutput;
            // finalPivotOutput_rot = Units.degreesToRotations(-Dashboard.freeTuningVariable.get());
        }
        if (Dashboard.overridePivot.get()) {
            ActuatorInterlocks.testActuatorInterlocks(pivot, "Pivot_(p)", 0.0);
        }
        finalPivotOutput_rot = MathUtil.clamp(finalPivotOutput_rot, Units.radiansToRotations(maxBackwardPivotAngle_rad),
                Units.radiansToRotations(maxForwardPivotAngle_rad));
        ActuatorInterlocks.testActuatorInterlocks(
            pivot, "Pivot_(p)",
            pivotPID.calculate(getPivotEncoderPosition().getRotations(), finalPivotOutput_rot) + pivotFF);

        // Extender
        double finalextenderOutput = 0.0;
        if (disableExtender) {
            finalextenderOutput = extenderOutput0;
        } else {
            finalextenderOutput = extenderOutput_m;
            extenderOutput0 = extenderOutput_m;
            // finalextenderOutput =
            // Units.inchesToMeters(Dashboard.freeTuningVariable.get()) *
            // extenderRatio_m_to_rot;
        }
        if (Dashboard.overrideExtender.get()) {
            ActuatorInterlocks.testActuatorInterlocks(extender, "Extender_(p)", 0.0);
        }
        finalextenderOutput = MathUtil.clamp(finalextenderOutput, Units.inchesToMeters(-0.5), maxExtensionDistance_m);
        ActuatorInterlocks.testActuatorInterlocks(
                extender, "Extender_(p)",
                finalextenderOutput * extenderRatio_m_to_rot, 0.03);

        // Wrist
        double finalwristOutput_rot = 0.0;
        if (disableWrist) {
            finalwristOutput_rot = wristOutput0.getRotations();
        } else {
            finalwristOutput_rot = wristOutput.getRotations();
            wristOutput0 = wristOutput;
            // finalwristOutput_rot =
            // Units.degreesToRotations(Dashboard.freeTuningVariable.get());
        }
        if (Dashboard.overrideWrist.get()) {
            finalwristOutput_rot = 0.0;
        }
        finalwristOutput_rot = MathUtil.clamp(finalwristOutput_rot, Units.radiansToRotations(maxBackwardWristAngle_rad),
                Units.radiansToRotations(maxForwardWristAngle_rad));
        ActuatorInterlocks.testActuatorInterlocks(
                wrist, "Wrist_(p)",
                finalwristOutput_rot * wristRatio, -wristFF);

        lockUnlockJoints();
    }

    boolean unlockButton0 = false;
    private void lockUnlockJoints() {
        // Pivot
        boolean unlockPivot = Dashboard.unlockPivot.get();
        if ((unlockPivot && (!unlockPivot0)) || (unlockButton.get() && !unlockButton0)) {
            pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            pivot.getConfigurator().apply(pivotConfig);
            for (int i = 0; i < 3; i++) {
                pivotFollowerConfigs[i].MotorOutput.NeutralMode = NeutralModeValue.Coast;
                pivotFollowers[i].getConfigurator().apply(pivotFollowerConfigs[i]);
            }
        } else if (((!unlockPivot) && unlockPivot0) || (!unlockButton.get() && unlockButton0) || (Robot.isEnabled && !Robot.isEnabled0)) {
            pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            pivot.getConfigurator().apply(pivotConfig);
            for (int i = 0; i < 3; i++) {
                pivotFollowerConfigs[i].MotorOutput.NeutralMode = NeutralModeValue.Brake;
                pivotFollowers[i].getConfigurator().apply(pivotFollowerConfigs[i]);
            }
        }
        unlockPivot0 = unlockPivot;
        // Extender
        boolean unlockExtender = Dashboard.unlockExtender.get();
        if ((unlockExtender && (!unlockExtender0)) || (unlockButton.get() && !unlockButton0)) {
            extenderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            extender.getConfigurator().apply(extenderConfig);
            extenderFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            extenderFollower.getConfigurator().apply(extenderFollowerConfig);
        } else if (((!unlockExtender) && unlockExtender0) || (!unlockButton.get() && unlockButton0) || (Robot.isEnabled && !Robot.isEnabled0)) {
            extenderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            extender.getConfigurator().apply(extenderConfig);
            extenderFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            extenderFollower.getConfigurator().apply(extenderFollowerConfig);
        }
        unlockExtender0 = unlockExtender;
        // Wrist
        boolean unlockWrist = Dashboard.unlockWrist.get();
        if ((unlockWrist && (!unlockWrist0)) || (unlockButton.get() && !unlockButton0)) {
            wristConfig.idleMode(IdleMode.kCoast);
            wrist.configure(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        } else if (((!unlockWrist) && unlockWrist0) || (!unlockButton.get() && unlockButton0) || (Robot.isEnabled && !Robot.isEnabled0)) {
            wristConfig.idleMode(IdleMode.kBrake);
            wrist.configure(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        unlockWrist0 = unlockWrist;

        unlockButton0 = unlockButton.get();
    }

    public Command drivingStow() {
        return Commands.startEnd(
            () -> updateArm(
                Units.inchesToMeters(0.0),
                Rotation2d.fromDegrees(-25),
                Rotation2d.fromDegrees(80)),
            () -> {},
            this
        );
    }

    public Command scoringStow() {
        return
        Commands.sequence(
            Commands.startEnd(() -> {
                updatePivot(Rotation2d.fromDegrees(-25));
                updateWrist(Rotation2d.fromDegrees(25));
            }, ()->{}, this).until(() -> closeEnough()),
            Commands.startEnd(
                () -> updateArm(
                        Units.inchesToMeters(0.0),
                        Rotation2d.fromDegrees(-25),
                        Rotation2d.fromDegrees(25)),
                () -> {},
                this
            )
        );
    }

    public Command carefulStow() {
        return 
        Commands.sequence(
            Commands.startEnd(
                () -> updatePivot(Rotation2d.fromDegrees((pickupHeight < 1.5) ? 0:-25)), 
                () -> {}, 
                this).until(() -> closeEnough()),
            Commands.startEnd(() -> updateWrist(Rotation2d.fromDegrees(0)), 
            () -> {},
            this).until(() -> closeEnough()),
            Commands.startEnd(() -> updateWrist(Rotation2d.fromDegrees(25)), 
            () -> {},
            this).until(() -> closeEnough()),
            scoringStow()
        );
    }

    public Command score() {
        Command scoringCommand;
        switch (scoreHeight) {
            case 1:
                scoringCommand = scoreL1();
                break;

            case 2:
                scoringCommand = scoreL2();
                break;

            case 3:
                scoringCommand = scoreL3();
                break;

            case 4:
                scoringCommand = scoreL4();
                break;

            case 5:
                scoringCommand = scoreProcessor();
                break;

            case 6:
                scoringCommand = scoreBarge();
                break;

            default:
                scoringCommand = Commands.none();
                new Exception("Score type out of range :(").printStackTrace();
                break;
        }
        return 
        Commands.sequence(
            retractFromReef().until(() -> closeEnough()).onlyIf(() -> Robot.masterState.equals(MasterStates.SCOR)),
            scoringCommand.until(() -> hasArrived()),
            Commands.waitUntil(() -> Intake.outtaking),
            Commands.waitUntil(() -> Robot.driverController.getRightTriggerAxis() < 0.7),
            scoringStow().until(() -> closeEnough()),
            Commands.runOnce(() -> {Robot.masterState0 = Robot.masterState;
                                    Robot.masterState = MasterStates.FEED;})
        );
    }

    public Command scoreL4() {
        return 
                Commands.startEnd(
                    () -> updateArm(
                            Units.inchesToMeters(37.6),
                            Rotation2d.fromDegrees(-1.9),
                            Rotation2d.fromDegrees(59.9)),
                    () -> {},
                    this
        );
    }

    private Command scoreL3() {
        return Commands.startEnd(() -> {
            updateArm(
                Units.inchesToMeters(6.8),
                Rotation2d.fromDegrees(-3.5),
                Rotation2d.fromDegrees(39));
                Dashboard.scoringState.set(2);},
            () -> {},
            this);
    }

    private Command scoreL2() {
        return Commands.startEnd(
            () -> {
                updateArm(
                    Units.inchesToMeters(0.0),
                    Rotation2d.fromDegrees(5),
                    Rotation2d.fromDegrees(75));
                Dashboard.scoringState.set(1);},
            () -> {},
            this
        );
    }

    private Command scoreL1() {
        return Commands.startEnd(
            () -> {
                updateArm(
                    Units.inchesToMeters(0.0),
                    Rotation2d.fromDegrees(-42),
                    Rotation2d.fromDegrees(-115));
                Dashboard.scoringState.set(0);},
            () -> {},
            this
        );
        // Temp values
    }

    private Command scoreProcessor() {
        return Commands.startEnd(
            () -> updateArm(
                Units.inchesToMeters(0.0),
                Rotation2d.fromDegrees(-62),
                Rotation2d.fromDegrees(-115)),
            () -> {},
            this
        );
    }

    private Command scoreBarge() {
        return Commands.sequence(
            Commands.startEnd(() -> updateExtender(42), ()->{}, this).until(() -> closeEnough()),
            Commands.startEnd(
            () -> updateArm(
                Units.inchesToMeters(42),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(-25)),
            () -> {},
            this)
        );
    }

    private Command retractFromReef() {
        return Commands.startEnd(
            () -> {
                updatePivot(Rotation2d.fromDegrees(-25));
                updateWrist(Rotation2d.fromDegrees(25));
            },
            () -> {},
            this
        );
    }

    public Command pickup() {
        Command pickupCommand;
        if (Robot.scoreCoral) {
            pickupHeight = 1;
        }
        switch (pickupHeight) {
            case 1:
                pickupCommand = pickupFeeder();
                break;

            case 2:
                pickupCommand = pickupLowAlgae();
                break;

            case 3:
                pickupCommand = pickupHighAlgae();
                break;

            default:
                pickupCommand = Commands.none();
                new Exception("Pickup type out of range :(").printStackTrace();
                break;
        }
        return Commands.sequence(
            pickupCommand.until(() -> hasArrived()),
            Commands.waitUntil(() -> Intake.hasPiece),
            carefulStow()
        );
    }

    public Command pickupFeeder() {
        return Commands.either(
                // Normal movement
                Commands.startEnd(
                () -> updateArm(
                        Units.inchesToMeters(-0.2),
                        Rotation2d.fromDegrees(-19.9),
                        Rotation2d.fromDegrees(-100.9)),
                () -> {},
                this),
                // Stow first when needed
                Commands.startEnd(
                () -> updateWrist(
                        Rotation2d.fromDegrees(45)),
                () -> {},
                this).until(() -> closeEnough()).andThen(
                    // Then normal movement
                    Commands.startEnd(
                        () -> updateArm(
                            Units.inchesToMeters(-0.2),
                            Rotation2d.fromDegrees(-19.9),
                            Rotation2d.fromDegrees(-100.9)),
                        () -> {},
                        this)   
                ), 
                () -> MathUtil.angleModulus(wristRotation.getRadians()) < (Math.PI / 4.0));
    }

    private Command pickupLowAlgae() {
        return Commands.startEnd(
            () -> updateArm(
                    Units.inchesToMeters(2.6),
                    Rotation2d.fromDegrees(-3.7),
                    Rotation2d.fromDegrees(62.1)),
            () -> {},
            this
        );
    }

    private Command pickupHighAlgae() {
        return Commands.sequence(
            Commands.startEnd(() -> updateExtender(14.2), ()->{}, this).until(() -> closeEnough()),
            Commands.startEnd(
            () -> updateArm(
                    Units.inchesToMeters(14.2),
                    Rotation2d.fromDegrees(-3),
                    Rotation2d.fromDegrees(44)),
            () -> {},
            this)
        );
    }

    public Command groundPickupCoral() {
        return Commands.either(
                // Normal movement
                Commands.run(() -> {
                    if (!Robot.scoreCoral) {
                        ArmCommands.groundPickup(this).schedule();
                    }
                    updateArm(
                        Units.inchesToMeters(0.8),
                        Rotation2d.fromDegrees(-99),
                        Rotation2d.fromDegrees(-82.7)
                    );
                },
                this),
                // Stow first when needed
                Commands.startEnd(
                () -> updateWrist(
                        Rotation2d.fromDegrees(45)),
                () -> {},
                this).until(() -> closeEnough()).andThen(
                    // Then normal movement
                    Commands.run(
                        () -> {
                            if (!Robot.scoreCoral) {
                                ArmCommands.groundPickup(this).schedule();
                            }
                            updateArm(
                            Units.inchesToMeters(0.8),
                            Rotation2d.fromDegrees(-99),
                            Rotation2d.fromDegrees(-82.7));},
                        this)   
                ), 
                () -> MathUtil.angleModulus(wristRotation.getRadians()) < (Math.PI / 4.0));
    }

    public Command groundPickupAlgae() {
        return Commands.either(
                // Normal movement
                Commands.run(() -> {
                if (Robot.scoreCoral) {
                    ArmCommands.groundPickup(this).schedule();
                }
                updateArm(
                        Units.inchesToMeters(-0.4),
                        Rotation2d.fromDegrees(-76.5),
                        Rotation2d.fromDegrees(-104.1));},
                this),
                // Stow first when needed
                Commands.startEnd(
                () -> updateWrist(
                        Rotation2d.fromDegrees(45)),
                () -> {},
                this).until(() -> closeEnough()).andThen(
                    // Then normal movement
                    Commands.run(
                        () -> {
                            if (Robot.scoreCoral) {
                                ArmCommands.groundPickup(this).schedule();
                            }
                            updateArm(
                            Units.inchesToMeters(-0.4),
                            Rotation2d.fromDegrees(-76.5),
                            Rotation2d.fromDegrees(-104.1));},
                        this)   
                ), 
                () -> MathUtil.angleModulus(wristRotation.getRadians()) < (Math.PI / 4.0));
    }

    public Command climbInit() {
        return Commands.runOnce(
            () -> updateArm(
                    Units.inchesToMeters(0.0),
                    Rotation2d.fromDegrees(0),
                    Rotation2d.fromDegrees(-60)),
            this
        );
    }

    public Command climb2() {
        return Commands.runOnce(
            () -> updateArm(
                    Units.inchesToMeters(10),
                    Rotation2d.fromDegrees(-30),
                    Rotation2d.fromDegrees(0)),
            this
        );
    }

    public Command climbLift() {
        return Commands.startEnd(
            () -> {
                pivotConfig.MotorOutput.PeakReverseDutyCycle = -0.5;
                pivot.getConfigurator().apply(pivotConfig);
                updateArm(
                    Units.inchesToMeters(10),
                    Rotation2d.fromDegrees(-100),
                    Rotation2d.fromDegrees(90));
            },
            () -> {
                pivotConfig.MotorOutput.PeakReverseDutyCycle = -0.1;
                pivot.getConfigurator().apply(pivotConfig);
            },
            this
        );
    }
}