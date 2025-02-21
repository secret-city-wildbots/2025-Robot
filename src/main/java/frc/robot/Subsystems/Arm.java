package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Utility.ActuatorInterlocks;

public class Arm extends SubsystemBase {
    // Constants
    private final double pivotRatio;
    private final double extenderRatio;
    private final double wristRatio;
    private final double maxExtensionDistance_m;
    private final double maxForwardPivotAngle;
    private final double maxBackwardPivotAngle;
    private final double maxForwardWristAngle;
    private final double maxBackwardWristAngle;

    private final double maxAcceptableAngleError_rad = 1;
    private final double maxAcceptableExtensionError_m = Units.inchesToMeters(0.5);

    // Motors
    private final TalonFX pivot;
    private final TalonFX extender;

    private final SparkMax wrist;
    private final SparkAbsoluteEncoder wristEncoder;
    private final SparkClosedLoopController wristController;
    private final SparkMaxConfig wristConfig;

    // Sensor values
    private Rotation2d pivotRotation;
    private double extenderPosition_m;
    private Rotation2d wristRotation;

    // Outputs
    private Rotation2d pivotOutput;
    private double extenderOutput_m;
    private Rotation2d wristOutput;
    private double wristFF = 0.0;
    private double wristFFArbitraryScalar = 0.0;
    private int scoreHeight = 1;
    private int pickupHeight = 1;

    public Arm() {
        pivot = new TalonFX(14);
        TalonFXConfiguration baseConfig = new TalonFXConfiguration();
        baseConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        baseConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        baseConfig.Slot0.kP = 0.0;
        baseConfig.Slot0.kI = 0.0;
        baseConfig.Slot0.kD = 0.0;
        pivot.getConfigurator().apply(baseConfig);

        extender = new TalonFX(15);
        TalonFXConfiguration extenderConfig = new TalonFXConfiguration();
        extenderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        extenderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extenderConfig.Slot0.kP = 0.0;
        extenderConfig.Slot0.kI = 0.0;
        extenderConfig.Slot0.kD = 0.0;
        extender.getConfigurator().apply(extenderConfig);

        wrist = new SparkMax(16, MotorType.kBrushless);
        wristEncoder = wrist.getAbsoluteEncoder();
        wristController = wrist.getClosedLoopController();
        wristConfig = new SparkMaxConfig();
        wristConfig.closedLoop.pid(0.5,0.0,0.0);
        wristConfig.closedLoop.maxOutput(0.2);
        wristConfig.closedLoop.minOutput(-0.2);
        wristConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        wrist.configure(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            

        switch (Robot.robotProfile) {
            case "2025_Robot":
                pivotRatio = 1.0;
                extenderRatio = 1.0;
                wristRatio = 1.0;
                maxExtensionDistance_m = 1.0;
                maxForwardPivotAngle = Math.PI / 2.0;
                maxBackwardPivotAngle = -Math.PI / 2.0;
                maxForwardWristAngle = Math.PI / 2.0;
                maxBackwardWristAngle = -Math.PI / 2.0;
                break;
            case "COTS_Testbed":
                pivotRatio = 1.0;
                extenderRatio = 1.0;
                wristRatio = 1.0;
                maxExtensionDistance_m = 1.0;
                maxForwardPivotAngle = Math.PI / 2.0;
                maxBackwardPivotAngle = -Math.PI / 2.0;
                maxForwardWristAngle = Math.PI / 2.0;
                maxBackwardWristAngle = -Math.PI / 2.0;
                break;
            default:
                pivotRatio = 1.0;
                extenderRatio = 1.0;
                wristRatio = 1.0;
                maxExtensionDistance_m = 1.0;
                maxForwardPivotAngle = Math.PI / 2.0;
                maxBackwardPivotAngle = -Math.PI / 2.0;
                maxForwardWristAngle = Math.PI / 2.0;
                maxBackwardWristAngle = -Math.PI / 2.0;
        }
    }

    public void updateSensors(XboxController manipController) {
        pivotRotation = Rotation2d.fromRotations(pivot.getRotorPosition().getValueAsDouble() / pivotRatio);
        extenderPosition_m = extender.getPosition().getValueAsDouble() / extenderRatio;
        wristRotation = Rotation2d.fromRotations(wristEncoder.getPosition() / wristRatio);
        if (Robot.scoreCoral) {
            pickupHeight = 1; // Feeder  
            if (manipController.getAButtonPressed()) {
                scoreHeight = 2;
            } else if (manipController.getXButtonPressed()) {
                scoreHeight = 3;
            } else if (manipController.getYButtonPressed()) {
                scoreHeight = 4;
            } else if (manipController.getBButtonPressed()) {
                scoreHeight = 1;
            }
        } else {
            if (manipController.getAButtonPressed()) {
                pickupHeight = 2; // Low Algae Pickup
            } else if (manipController.getXButtonPressed()) {
                pickupHeight = 3; // High Algae Pickup
            } else if (manipController.getYButtonPressed()) {
                scoreHeight = 6; // Barge
            } else if (manipController.getBButtonPressed()) {
                scoreHeight = 5; // Processor
            }
        }

        // wristFFArbitraryScalar = Dashboard.freeTuningVariable.get();
        wristFF = pivotRotation.plus(wristRotation).getSin() * wristFFArbitraryScalar;
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
        updateArm(0, 0, new Rotation2d());
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
        updateArm(0.1, 1, Rotation2d.fromDegrees(45));
    }

    private void scoreL3() {
        updateArm(0.2, 0.8, Rotation2d.fromDegrees(45));
    }

    private void scoreL2() {
        updateArm(0.2, 0.5, Rotation2d.fromDegrees(45));
    }
    
    private void scoreL1() {
        updateArm(0.3, 0.0, Rotation2d.fromDegrees(45));
    }
    private void scoreProcessor() {
        updateArm(0.3, 0.0, Rotation2d.fromDegrees(45));
    }
    private void scoreBarge() {
        updateArm(0.3, 0.0, Rotation2d.fromDegrees(45));
    }

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
        updateArm(-0.3, 0, Rotation2d.fromDegrees(-45));
    }
    private void pickupLowAlgae() {
        updateArm(0.3, 0, Rotation2d.fromDegrees(45));
    }
    private void pickupHighAlgae() {
        updateArm(0.3, 0, Rotation2d.fromDegrees(45));
    }

    private void updateArm(double forwardDistance_m, double upwardDistance_m, Rotation2d wristAngle) {
        Rotation2d rotation = new Rotation2d(upwardDistance_m, forwardDistance_m);
        extenderOutput_m = MathUtil.clamp(Math.hypot(forwardDistance_m, upwardDistance_m), 0.0,
                maxExtensionDistance_m);
        pivotOutput = new Rotation2d(MathUtil.clamp(rotation.getRadians(), maxBackwardPivotAngle, maxForwardPivotAngle));
        wristOutput = new Rotation2d(MathUtil.clamp(wristAngle.getRadians() - rotation.getRadians(), maxBackwardWristAngle, maxForwardWristAngle));
    }

    public void updateOutputs() {
        ActuatorInterlocks.TAI_TalonFX_Position(
            pivot, "Pivot_(p)", 
            pivotOutput.getRotations() * pivotRatio, 0.0);
        ActuatorInterlocks.TAI_TalonFX_Position(
            extender, "Extender_(p)", 
            extenderOutput_m * extenderRatio, 0.0);
        ActuatorInterlocks.TAI_SparkMAX_Position(
            wrist, wristController, "Wrist_(p)", 
            wristOutput.getRotations() * wristRatio, wristFF);
    }
}
