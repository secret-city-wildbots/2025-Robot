package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Utility.ActuatorInterlocks;
import frc.robot.Utility.ClassHelpers.Timer;

public class Intake extends SubsystemBase {
    // Constants
    private final double coralIntakeSpeed = 0.3;
    private final double coralOuttakeSpeed = -0.7;
    private final double algaeIntakeSpeed = 0.3;
    private final double algaeOuttakeSpeed = -0.7;
    private final double stallCurrent = 100;
    private final Timer stallTimer = new Timer();

    // Sensors
    private boolean hasPiece;

    // Motors
    private final TalonFX intake;

    // Outputs
    private double intakeOutput;

    public Intake() {
        intake = new TalonFX(17);
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intake.getConfigurator().apply(intakeConfig);
    }

    public void updateSensors() {
        if (intake.getSupplyCurrent().getValueAsDouble() > stallCurrent) {
            if (stallTimer.getTimeMillis() > 500) {
                hasPiece = true;
            }
        } else {
            stallTimer.reset();
        }
    }

    public void intake() {
        intakeOutput = (Robot.scoreCoral) ? coralIntakeSpeed : algaeIntakeSpeed;
    }

    public void outtake() {
        intakeOutput = (Robot.scoreCoral) ? coralOuttakeSpeed : algaeOuttakeSpeed;
        hasPiece = false;
    }

    public void stop() {
        intakeOutput = 0.0;
    }

    public void hold() {
        intakeOutput = -0.1;
    }

    public boolean hasPiece() {
        return hasPiece;
    }

    public void updateOutputs() {
        ActuatorInterlocks.TAI_TalonFX_Power(intake, "Intake_(p)", intakeOutput);
    }
}
