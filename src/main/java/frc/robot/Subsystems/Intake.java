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
    private final Timer stallTimer = new Timer();

    // Sensors
    private boolean hasPiece;

    // Motors
    private final TalonFX intake;

    // Outputs
    public double intakeOutput;

    public Intake() {
        intake = new TalonFX(25);
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intake.getConfigurator().apply(intakeConfig);
    }

    public void updateSensors() {
        if (intakeOutput > 0.1) {
            if (intake.getVelocity().getValueAsDouble() < 1 && stallTimer.getTimeMillis() > 500) {
                hasPiece = true;
            }
        } else {
            stallTimer.reset();
        }
        System.out.println(hasPiece);
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
        intakeOutput = 0.0;
    }

    public boolean hasPiece() {
        return hasPiece;
    }

    private boolean controllerOn0 = false;
    public boolean stopIntaking() {
        boolean controllerOn = Robot.driverController.getLeftTriggerAxis() > 0.7;
        boolean toggle = controllerOn && (!controllerOn0);
        controllerOn0 = controllerOn;
        return (hasPiece() && (!controllerOn)) || toggle;
    }

    public void updateOutputs() {
        ActuatorInterlocks.testActuatorInterlocks(intake, "Intake_(p)", intakeOutput);
    }
}
