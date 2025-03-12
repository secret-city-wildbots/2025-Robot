package frc.robot.Subsystems;

// import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
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
import edu.wpi.first.wpilibj.XboxController;
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
    private final double closeEnoughAngleError_rad = Units.degreesToRadians(5);
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
    private double kp0 = 0.0;
    private double ki0 = 0.0;
    private double kd0 = 0.0;

    // TOF sensors
    private TimeOfFlight frontTOFSensor = new TimeOfFlight(7);
    private double frontTOFOffset_m = Units.inchesToMeters(4.625);
    private TimeOfFlight backTOFSensor = new TimeOfFlight(8);
    private double backTOFOffset_m = Units.inchesToMeters(11.875);


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
    private final Trigger driverLBTrigger;
    private final Trigger coralTrigger;
    private final Trigger stowTrigger = new Trigger(() -> Robot.masterState.equals(MasterStates.STOW));
    private final Trigger scoreTrigger = new Trigger(() -> Robot.masterState.equals(MasterStates.SCOR));
    private final Trigger feedrTrigger = new Trigger(() -> Robot.masterState.equals(MasterStates.FEED));
    private final Trigger climbTrigger = new Trigger(() -> Robot.masterState.equals(MasterStates.CLMB));
    private final Trigger manipATrigger;
    private final Trigger manipXTrigger;
    private final Trigger manipYTrigger;
    private final Trigger manipBTrigger;
    private final Trigger manipLBTrigger;

    public Arm() {
        pivotPID.enableContinuousInput(0,1);
        switch (Robot.robotProfile) {
            case "2025_Robot":
                // Temp values
                pivotRatio = (84.0 / 8.0) * // First gear reduction
                        (18.0 / 1.6); // Capstan reduction, Total reduction: 118.13:1
                maxForwardPivotAngle_rad = Units.degreesToRadians(12);
                maxBackwardPivotAngle_rad = Units.degreesToRadians(-90);

                extenderRatio_m_to_rot = 1.0 / Units.inchesToMeters( // Convert to meters for use elsewhere
                        (1.0 / 5.25) * // ratio of motor rotations to spool rotations
                                Math.PI * 1.432 // Spool diameter (in) -> circumference (in)
                                * 2.0 // Two stages moving together doubles movement
                );
                maxExtensionDistance_m = Units.inchesToMeters(43);

                wristRatio = 70.0 * // Neo reduction
                        (34.0 / 12.0); // Small herringbone to big herringbone
                maxForwardWristAngle_rad = Units.degreesToRadians(112);
                maxBackwardWristAngle_rad = Units.degreesToRadians(-125);
                break;
            case "COTS_Testbed":
                // Temp values
                pivotRatio = (84.0 / 8.0) * // First gear reduction
                        (18.0 / 1.6); // Capstan reduction, Total reduction: 118.13:1
                maxForwardPivotAngle_rad = Units.degreesToRadians(12);
                maxBackwardPivotAngle_rad = Units.degreesToRadians(-90);

                extenderRatio_m_to_rot = 1.0 / Units.inchesToMeters( // Convert to meters for use elsewhere
                        (1.0 / 5.25) * // ratio of motor rotations to spool rotations
                                Math.PI * 1.432 // Spool diameter (in) -> circumference (in)
                                * 2.0 // Two stages moving together doubles movement
                );
                maxExtensionDistance_m = Units.inchesToMeters(43);

                wristRatio = 70.0 * // Neo reduction
                        (34.0 / 12.0); // Small herringbone to big herringbone
                maxForwardWristAngle_rad = Units.degreesToRadians(112);
                maxBackwardWristAngle_rad = Units.degreesToRadians(-125);
                break;
            default:
                // Temp values
                pivotRatio = (84.0 / 8.0) * // First gear reduction
                        (18.0 / 1.6); // Capstan reduction, Total reduction: 118.13:1
                maxForwardPivotAngle_rad = Units.degreesToRadians(12);
                maxBackwardPivotAngle_rad = Units.degreesToRadians(-90);

                extenderRatio_m_to_rot = 1.0 / Units.inchesToMeters( // Convert to meters for use elsewhere
                        (1.0 / 5.25) * // ratio of motor rotations to spool rotations
                                Math.PI * 1.432 // Spool diameter (in) -> circumference (in)
                                * 2.0 // Two stages moving together doubles movement
                );
                maxExtensionDistance_m = Units.inchesToMeters(43);

                wristRatio = 70.0 * // Neo reduction
                        (34.0 / 12.0); // Small herringbone to big herringbone
                maxForwardWristAngle_rad = Units.degreesToRadians(112);
                maxBackwardWristAngle_rad = Units.degreesToRadians(-125);
                break;
        }

        // Pivot and pivot follower configurations
        pivot = new TalonFX(30);
        pivot.setPosition(getEncoderPosition().getRotations() * pivotRatio);
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
        extenderConfig.MotorOutput.PeakForwardDutyCycle = 0.8;
        extenderConfig.MotorOutput.PeakReverseDutyCycle = -0.5;
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
        wrist.getEncoder().setPosition(wristEncoder.getPosition() - 198.333);
        wristConfig.idleMode(IdleMode.kBrake);
        wristConfig.inverted(true);
        wristConfig.closedLoop.pid(0.05, 0.0, 0.0);
        wristConfig.closedLoop.maxOutput(0.5);
        wristConfig.closedLoop.minOutput(-0.5);
        wristConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        wristConfig.closedLoop.positionWrappingEnabled(true);
        wristConfig.closedLoop.positionWrappingInputRange(0, 198.333);
        wristConfig.absoluteEncoder.inverted(true);

        wristConfig.absoluteEncoder.zeroOffset(0.982746);
        wristConfig.absoluteEncoder.positionConversionFactor(198.333);
        // wristConfig.softLimit.reverseSoftLimit(Units.radiansToRotations(maxBackwardWristAngle_rad)*wristRatio);
        // wristConfig.softLimit.forwardSoftLimit(Units.radiansToRotations(maxForwardWristAngle_rad)*wristRatio);
        // wristConfig.softLimit.reverseSoftLimitEnabled(true);
        // wristConfig.softLimit.forwardSoftLimitEnabled(true);
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Trigger configurations
        coralTrigger = new Trigger(() -> Robot.scoreCoral);
        coralTrigger.onTrue(Commands.runOnce(() -> {scoreHeight = 1; Dashboard.scoringState.set(0); pickupHeight = 1;}));
        coralTrigger.onFalse(Commands.runOnce(() -> {scoreHeight = 6; Dashboard.scoringState.set(0); pickupHeight = 2;}));
        driverLBTrigger = Robot.driverCommandController.leftBumper();
        driverLBTrigger.onTrue(ArmCommands.groundPickup(this));
        manipLBTrigger = Robot.manipCommandController.leftBumper();
        manipLBTrigger.onTrue(
            Commands.either(
                Commands.runOnce(() -> drivingStow(), this),
                Commands.runOnce(() -> scoringStow(), this),
                () -> wristRotation.getDegrees() < 50
            ).onlyIf(() -> Drivetrain.masterState0.equals(MasterStates.STOW)));
        Drivetrain.antiTipTrigger.onTrue(ArmCommands.stow(this));
        scoreTrigger.onFalse(
            Commands.either(
                // If autoStow
                Commands.sequence(
                    Commands.runOnce(() -> {
                        updatePivot(Rotation2d.fromDegrees(-25));
                        updateWrist(Rotation2d.fromDegrees(25));
                    }, this).onlyIf(() -> Robot.masterState.equals(MasterStates.STOW)),
                    Commands.waitUntil(() -> closeEnough()),
                    Commands.runOnce(() -> updateExtender(0.0)),       
                    Commands.waitUntil(() -> closeEnough()),
                    Commands.runOnce(() -> pickup(), this)
                ),
                Commands.sequence(
                    Commands.runOnce(() -> {
                        updatePivot(Rotation2d.fromDegrees(-25));
                        updateWrist(Rotation2d.fromDegrees(25));
                    }, this).onlyIf(() -> Robot.masterState.equals(MasterStates.STOW)),
                    Commands.waitUntil(() -> closeEnough()),
                    Commands.runOnce(() -> updateExtender(0.0))
                ),
                Robot.manipCommandController.leftBumper().negate()
            )
        );
        feedrTrigger.onFalse(carefulStow().onlyIf(() -> !Robot.isAutonomous));
        stowTrigger.onFalse(
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> 
                        {updatePivot(Rotation2d.fromDegrees(-5));
                        updateExtender(Units.inchesToMeters(37.1));}),
                    Commands.waitUntil(() -> closeEnough()),
                    Commands.runOnce(() -> updateWrist(Rotation2d.fromDegrees(67)))),
                Commands.none(),
                () -> Robot.masterState.equals(MasterStates.SCOR) && scoreHeight == 4).onlyIf(() -> !Robot.isAutonomous));
        climbTrigger.onFalse(carefulStow());
        scoreTrigger.onTrue(ArmCommands.score(this).onlyIf(() -> !Robot.isAutonomous));
        feedrTrigger.onTrue(ArmCommands.pickup(this).onlyIf(() -> !Robot.isAutonomous));
        climbTrigger.onTrue(ArmCommands.climb(this));
        manipATrigger = Robot.manipCommandController.a();
        manipATrigger.onTrue(Commands.runOnce(() -> {
            if (Robot.scoreCoral) {
                scoreHeight = 2;
                Dashboard.scoringState.set(0);
                if (Robot.masterState.equals(MasterStates.SCOR)) {
                    ArmCommands.score(this).schedule();
                }
            } else {
                pickupHeight = 2;
                if (Robot.masterState.equals(MasterStates.FEED)) {
                    ArmCommands.pickup(this).schedule();
                }
            }
        }, this));
        manipXTrigger = Robot.manipCommandController.x();
        manipXTrigger.onTrue(Commands.runOnce(() -> {
            if (Robot.scoreCoral) {
                scoreHeight = 3;
                if (Robot.masterState.equals(MasterStates.SCOR)) {
                    ArmCommands.score(this).schedule();
                }
            } else {
                pickupHeight = 3;
                if (Robot.masterState.equals(MasterStates.FEED)) {
                    ArmCommands.pickup(this).schedule();
                }
            }
        }, this));
        manipYTrigger = Robot.manipCommandController.y();
        manipYTrigger.onTrue(Commands.runOnce(() -> {
            if (Robot.scoreCoral) {
                scoreHeight = 4;
            } else {
                scoreHeight = 6;
            }
            if (Robot.masterState.equals(MasterStates.SCOR)) {
                ArmCommands.score(this).schedule();
            }
            ;
        }, this));
        manipBTrigger = Robot.manipCommandController.b();
        manipBTrigger.onTrue(Commands.runOnce(() -> {
            if (Robot.scoreCoral) {
                scoreHeight = 1;
            } else {
                scoreHeight = 5;
            }
            if (Robot.masterState.equals(MasterStates.SCOR)) {
                ArmCommands.score(this).schedule();
            }
            ;
        }, this));

        drivingStow();
    }

    public Rotation2d getEncoderPosition() {
        double encoderRotations = pivotEncoderOffset_rot - pivotEncoder.get();
        return Rotation2d.fromRotations(encoderRotations);
    }

    double tofRange0 = frontTOFSensor.getRange();
    public void updateSensors(XboxController manipController) {
        if (Dashboard.calibrateExtender.get()) {
            extender.setPosition(Units.inchesToMeters(-1.0) * extenderRatio_m_to_rot);
        }
        extenderPosition_m = extender.getPosition().getValueAsDouble() / extenderRatio_m_to_rot;
        Dashboard.extenderPosition_in.set(Units.metersToInches(extenderPosition_m));

        wristRotation = Rotation2d.fromRotations(wristEncoder.getPosition() / wristRatio);
        wristFF = getEncoderPosition().plus(wristRotation).getSin() * wristFFArbitraryScalar;
        double realWristRotation_deg = wristRotation.getDegrees();
        if (realWristRotation_deg > 180) {realWristRotation_deg -= 360;}
        Dashboard.wristPosition_deg.set(realWristRotation_deg);

        if (getEncoderPosition().getDegrees() < -15) {
            pivotFF = -getEncoderPosition().getSin()*0.016;
            // pivotFF = 0.0;
        }

        Dashboard.pivotPosition_deg.set(getEncoderPosition().getDegrees());

        Dashboard.extenderTemp_C.set(extender.getDeviceTemp().getValueAsDouble());
        Dashboard.wristTemp_C.set(wrist.getMotorTemperature());
        Dashboard.pivotTemp_C.set(pivot.getDeviceTemp().getValueAsDouble());

        // PID Tuning
        double kp = Dashboard.freeTuningkP.get();
        double ki = Dashboard.freeTuningkI.get();
        double kd = Dashboard.freeTuningkD.get();
        if ((kp0 != kp) || (ki0 != ki) || (kd0 != kd)) {
        wristConfig.closedLoop.p(kp);
        wristConfig.closedLoop.i(ki);
        wristConfig.closedLoop.d(kd);
        wrist.configure(wristConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
        kp0 = kp;
        ki0 = ki;
        kd0 = kd;
        }
        Dashboard.pidTuningGoalActual.set(new double[] { wristOutput.getDegrees(),
        wristRotation.getDegrees() });

        double tofRange = frontTOFSensor.getRange();
        if (Math.abs(tofRange0 - tofRange) > 0.001 && 
            Robot.manipController.getRightTriggerAxis() > 0.7 && 
            Robot.masterState.equals(MasterStates.SCOR)) {
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
            default:
                break;
        }
        }
        tofRange0 = frontTOFSensor.getRange();
        // Dashboard.pidTuningGoalActual.set(new double[] {
        // -Dashboard.freeTuningVariable.get(), wristRotation.getDegrees() });

        // PID Tuning
        // double kp = Dashboard.freeTuningkP.get();
        // double ki = Dashboard.freeTuningkI.get();
        // double kd = Dashboard.freeTuningkD.get();
        // if ((kp0 != kp) || (ki0 != ki) || (kd0 != kd)) {
        // extenderConfig.Slot0.kP = kp;
        // extenderConfig.Slot0.kI = ki;
        // extenderConfig.Slot0.kD = kd;
        // extender.getConfigurator().apply(extenderConfig);
        // kp0 = kp;
        // ki0 = ki;
        // kd0 = kd;
        // }
        // Dashboard.pidTuningGoalActual.set(new double[] {
        // Dashboard.freeTuningVariable.get(), Units.metersToInches(extenderPosition_m)
        // });

        // PID Tuning
        // double kp = Dashboard.freeTuningkP.get();
        // double ki = Dashboard.freeTuningkI.get();
        // double kd = Dashboard.freeTuningkD.get();
        // if ((kp0 != kp) || (ki0 != ki) || (kd0 != kd)) {
        //     pivotPID.setP(kp);
        //     pivotPID.setI(ki);
        //     pivotPID.setD(kd);
        //     kp0 = kp;
        //     ki0 = ki;
        //     kd0 = kd;
        // }
        // Dashboard.pidTuningGoalActual
        //         .set(new double[] { -Dashboard.freeTuningVariable.get(), getEncoderPosition().getDegrees() });
    }

    public boolean hasArrived() {
        boolean hasArrived = false;
        if ((Math.abs(getEncoderPosition().getRadians() - pivotOutput.getRadians())) < maxAcceptableAngleError_rad) {
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
        if ((Math.abs(getEncoderPosition().getRadians() - pivotOutput.getRadians())) < closeEnoughAngleError_rad) {
            if ((Math.abs(extenderPosition_m - extenderOutput_m)) < closeEnoughExtensionError_m) {
                double realWristRotation_rad = wristRotation.getRadians();
                if (realWristRotation_rad > Math.PI) {realWristRotation_rad -= Math.PI * 2;}
                if ((Math.abs(realWristRotation_rad - wristOutput.getRadians())) < closeEnoughAngleError_rad) {
                    hasArrived = true;
                }
            }
        }
        return hasArrived;
    }

    public void drivingStow() {
        updateArm(0, 
            Rotation2d.fromDegrees(-25), 
            Rotation2d.fromDegrees(80));
    }

    public void scoringStow() {
        updateArm(0, 
            Rotation2d.fromDegrees(-25), 
            Rotation2d.fromDegrees(25));
    }

    public Command carefulStow() {
        return Commands.sequence(
            Commands.runOnce(() -> updatePivot(Rotation2d.fromDegrees((Robot.scoreCoral) ? 0:-25)), this),
            Commands.waitUntil(() -> closeEnough()),
            Commands.runOnce(() -> updateWrist(Rotation2d.fromDegrees(25)), this),
            Commands.waitUntil(() -> closeEnough()),
            Commands.runOnce(() -> scoringStow())
        );
    }

    public void score() {
        Robot.masterState = MasterStates.SCOR;
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
        double tofOffset = (frontTOFSensor.getRange()*0.001) - frontTOFOffset_m;
        if (tofOffset > Units.inchesToMeters(12)) {tofOffset = 0.0;}
        updateArm(
            Units.inchesToMeters(-3.23) + tofOffset, 
            Units.inchesToMeters(36.96), 
            Rotation2d.fromDegrees(66));
        // updateArm(
        //         Units.inchesToMeters(37.1),
        //         Rotation2d.fromDegrees(-5),
        //         Rotation2d.fromDegrees(67));
        Dashboard.scoringState.set(3);
    }

    private void scoreL3() {
        updateArm(
                Units.inchesToMeters(0),
                Rotation2d.fromDegrees(-11),
                Rotation2d.fromDegrees(38));
        Dashboard.scoringState.set(2);
    }

    private void scoreL2() {
        updateArm(
                Units.inchesToMeters(0),
                Rotation2d.fromDegrees(-10),
                Rotation2d.fromDegrees(83));
        Dashboard.scoringState.set(1);
    }

    private void scoreL1() {
        updateArm(
                Units.inchesToMeters(0),
                Rotation2d.fromDegrees(-42),
                Rotation2d.fromDegrees(-125));
        Dashboard.scoringState.set(0);
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
                Rotation2d.fromDegrees(-25));
    }

    public void pickup() {
        Robot.masterState = MasterStates.FEED;
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

    public void pickupFeeder_init() {
        updateArm(
            0, 
            Rotation2d.fromDegrees(-25), 
            Rotation2d.fromDegrees(25));
    }

    public void pickupFeeder() {
        updateArm(
                Units.inchesToMeters(-0.5),
                Rotation2d.fromDegrees(-11.8),
                Rotation2d.fromDegrees(-121));
    }

    private void pickupLowAlgae() {
        updateArm(
                Units.inchesToMeters(0),
                Rotation2d.fromDegrees(-4.7),
                Rotation2d.fromDegrees(53));
    }

    private void pickupHighAlgae() {
        // updateArm(
        //     Units.inchesToMeters(-10), 
        //     Units.inchesToMeters(7.13), 
        //     Rotation2d.fromDegrees(28)
        // );
        updateArm(
                Units.inchesToMeters(12.2),
                Rotation2d.fromDegrees(-3),
                Rotation2d.fromDegrees(44));
    }

    public void groundPickup() {
        updateArm(
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(-72),
            Rotation2d.fromDegrees(-125));
    }

    public void climbInit() {
        updateArm(
                Units.inchesToMeters(0),
                Rotation2d.fromDegrees(5),
                Rotation2d.fromDegrees(-90));
    }

    public void climbLift() {
        updateArm(
                Units.inchesToMeters(0.0),
                Rotation2d.fromDegrees(5),
                Rotation2d.fromDegrees(-90));
    }

    /**
     * Takes in a forward and upward distance along with a wrist angle and
     * calculates the correct outputs
     * 
     * @param forwardDistance_m The horizontal distance the wrist pivot should be
     *                          from the base pivot
     * @param upwardDistance_m  The vertical distance the wrist pivot should be form
     *                          the base pivot
     * @param wristAngle        The angle the wrist should be relative to vertical
     *                          (not accounting for pivot angle)
     */
    private void updateArm(double forwardDistance_m, double upwardDistance_m, Rotation2d wristAngle) {
        double armlength_m = Units.inchesToMeters(25.5);
        Rotation2d rotation = new Rotation2d(Math.atan2(forwardDistance_m, upwardDistance_m + armlength_m));
        updateArm(
                Math.hypot(forwardDistance_m, upwardDistance_m + armlength_m) - armlength_m,
                rotation, wristAngle.minus(rotation));
    }

    /**
     * Takes in a pivot and wrist angle and an extension distance directly output to
     * motors
     * 
     * @param extensionDistance_m The distance radially the extender should go to
     * @param pivotAngle          The angle relative to vertical the pivot should go
     *                            to
     * @param wristAngle          The angle relative to the pivot the wrist should
     *                            go to
     */
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

    @SuppressWarnings("unused")
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
                pivotPID.calculate(getEncoderPosition().getRotations(), finalPivotOutput_rot) + pivotFF);
        // Unlock pivot
        boolean unlockPivot = Dashboard.unlockPivot.get() || (!unlockButton.get() && !Robot.isEnabled);
        if (unlockPivot && (!unlockPivot0)) {
            pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            pivot.getConfigurator().apply(pivotConfig);
            for (int i = 0; i < 3; i++) {
                pivotFollowerConfigs[i].MotorOutput.NeutralMode = NeutralModeValue.Coast;
                pivotFollowers[i].getConfigurator().apply(pivotFollowerConfigs[i]);
            }
        } else if ((!unlockPivot) && unlockPivot0) {
            pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            pivot.getConfigurator().apply(pivotConfig);
            for (int i = 0; i < 3; i++) {
                pivotFollowerConfigs[i].MotorOutput.NeutralMode = NeutralModeValue.Brake;
                pivotFollowers[i].getConfigurator().apply(pivotFollowerConfigs[i]);
            }
        }
        unlockPivot0 = unlockPivot;

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
        // Unlock extender
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

        // Wrist
        // System.out.println("Disable: " + disableWrist);
        // System.out.println("Master State: " + Robot.masterState.toString());
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
            ActuatorInterlocks.testActuatorInterlocks(wrist, "Wrist_(p)", 0.0);
        }
        finalwristOutput_rot = MathUtil.clamp(finalwristOutput_rot, Units.radiansToRotations(maxBackwardWristAngle_rad),
                Units.radiansToRotations(maxForwardWristAngle_rad));
                // System.out.println("Output: " + finalwristOutput_rot);
        ActuatorInterlocks.testActuatorInterlocks(
                wrist, "Wrist_(p)",
                finalwristOutput_rot * wristRatio, -wristFF);
        // Unlock wrist
        boolean unlockWrist = Dashboard.unlockWrist.get() || (!unlockButton.get() && !Robot.isEnabled);
        if (unlockWrist && (!unlockWrist0)) {
            wristConfig.idleMode(IdleMode.kCoast);
            wrist.configure(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        } else if ((!unlockWrist) && unlockWrist0) {
            wristConfig.idleMode(IdleMode.kBrake);
            wrist.configure(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        unlockWrist0 = unlockWrist;
    }
}
