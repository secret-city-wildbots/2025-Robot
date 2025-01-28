package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Arm extends SubsystemBase{
    // Constants
    private final double basePivotRatio;
    private final double extenderRatio;
    private final double topPivotRatio;

    // Motors
    private final TalonFX basePivot;
    private final TalonFX extender;
    private final TalonFX topPivot;

    private final SparkMax intake;

    // Sensor values
    private double basePivotRotation_rad;
    private double extenderPosition_m;
    private double topPivotRotation_rad;

    // Outputs
    private double basePivotOutput_rad;
    private double extenderOutput_m;
    private double topPivotOutput_rad;

    public Arm() {
        basePivot = new TalonFX(14);
        TalonFXConfiguration baseConfig = new TalonFXConfiguration();
        baseConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        baseConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        baseConfig.Slot0.kP = 0.0;
        baseConfig.Slot0.kI = 0.0;
        baseConfig.Slot0.kD = 0.0;
        basePivot.getConfigurator().apply(baseConfig);
        
        extender = new TalonFX(15);
        TalonFXConfiguration extenderConfig = new TalonFXConfiguration();
        extenderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        extenderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extenderConfig.Slot0.kP = 0.0;
        extenderConfig.Slot0.kI = 0.0;
        extenderConfig.Slot0.kD = 0.0;
        extender.getConfigurator().apply(extenderConfig);
        
        topPivot = new TalonFX(16);
        TalonFXConfiguration topPivotConfig = new TalonFXConfiguration();
        topPivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        topPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        topPivotConfig.Slot0.kP = 0.0;
        topPivotConfig.Slot0.kI = 0.0;
        topPivotConfig.Slot0.kD = 0.0;
        topPivot.getConfigurator().apply(topPivotConfig);

        intake = new SparkMax(17, MotorType.kBrushless);

        switch(Robot.robotProfile) {
            case "2025_Robot":
                basePivotRatio = 1.0;
                extenderRatio = 1.0;
                topPivotRatio = 1.0;
                break;
            case "COTS_Testbed":
                basePivotRatio = 1.0;
                extenderRatio = 1.0;
                topPivotRatio = 1.0;
                break;
            default:
                basePivotRatio = 1.0;
                extenderRatio = 1.0;
                topPivotRatio = 1.0;
        }
    }

    public void updateSensors() {
        basePivotRotation_rad = Units.rotationsToRadians(basePivot.getRotorPosition().getValueAsDouble() / basePivotRatio);
        extenderPosition_m = extender.getPosition().getValueAsDouble() / extenderRatio;
        topPivotRotation_rad = topPivot.getPosition().getValueAsDouble() / topPivotRatio;
    }

    public void updateArm() {
        double[] outPositionxy_m = new double[]{0,0};
        double extensionDistance_m = Math.sqrt(Math.pow(outPositionxy_m[0], 2) + Math.pow(outPositionxy_m[1], 2));
        double extensionAngle_rad = Math.atan2(outPositionxy_m[0], outPositionxy_m[1]);
        extenderOutput_m = extensionDistance_m;
        basePivotOutput_rad = extensionAngle_rad;
    }

    public void updateOutputs() {
        PositionDutyCycle baseRequest = new PositionDutyCycle(Units.radiansToRotations(basePivotOutput_rad * basePivotRatio));
        basePivot.setControl(baseRequest);
        PositionDutyCycle extenderRequest = new PositionDutyCycle(extenderOutput_m * extenderRatio);
        extender.setControl(extenderRequest);
        PositionDutyCycle topRequest = new PositionDutyCycle(Units.radiansToRotations(topPivotOutput_rad * topPivotRatio));
        topPivot.setControl(topRequest);
    }
}
