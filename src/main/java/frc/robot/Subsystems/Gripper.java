package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utility.ActuatorInterlocks;

public class Gripper extends SubsystemBase {
    // Constants
    private final double gripperIntakeSpeed = 0.3;
    private final double gripperOuttakeSpeed = -0.7;

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

    public void intake() {
        
    }

    public void updateOutputs() {
        ActuatorInterlocks.TAI_TalonFX_Power(gripper, "Gripper_(p)", gripperOutput);
    }
}
