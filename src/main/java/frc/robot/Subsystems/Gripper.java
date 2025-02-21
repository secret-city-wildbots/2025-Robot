package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utility.ActuatorInterlocks;
import frc.robot.Utility.ClassHelpers.Timer;

public class Gripper extends SubsystemBase {
    // Constants
    private final double gripperIntakeSpeed = 0.3;
    private final double gripperOuttakeSpeed = -0.7;
    private final double stallCurrent = 100;
    private final Timer stallTimer = new Timer();

    // Sensors
    private boolean hasPiece;

    // Motors
    private final TalonFX gripper;

    // Outputs
    private double gripperOutput;

    public Gripper() {
        gripper = new TalonFX(17);
        TalonFXConfiguration gripperConfig = new TalonFXConfiguration();
        gripperConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        gripperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        gripper.getConfigurator().apply(gripperConfig);
    }

    public void updateSensors() {
        if (gripper.getSupplyCurrent().getValueAsDouble() > stallCurrent) {
            if (stallTimer.getTimeMillis() > 500) {
                hasPiece = true;
            }
        } else {
            stallTimer.reset();
        }
    }

    public void intake() {
        gripperOutput = gripperIntakeSpeed;
    }

    public void outtake() {
        gripperOutput = gripperOuttakeSpeed;
        hasPiece = false;
    }

    public void stop() {
        gripperOutput = 0.0;
    }

    public void hold() {
        gripperOutput = -0.1;
    }

    public boolean hasPiece() {
        return hasPiece;
    }

    public void updateOutputs() {
        ActuatorInterlocks.TAI_TalonFX_Power(gripper, "Gripper_(p)", gripperOutput);
    }
}
