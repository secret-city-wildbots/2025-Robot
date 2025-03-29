package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Dashboard;
import frc.robot.Robot;
import frc.robot.Utility.ActuatorInterlocks;
import frc.robot.Utility.ClassHelpers.Timer;

public class Intake extends SubsystemBase {
    // Constants
    private final double coralIntakeSpeed = 0.6;
    private final double coralOuttakeSpeed = -1;
    private final double coralIntakeTopSpeed = 1;
    private final double coralOuttakeTopSpeed = -1;
    private final double algaeIntakeSpeed = 1;
    private final double algaeOuttakeSpeed = -1;
    private final Timer stallTimer = new Timer();

    // Sensors
    public static boolean hasPiece = false;
    public static boolean hasPiece0 = false;
    public static boolean intaking;
    public static boolean outtaking;
    private Trigger outtakeTrigger = new Trigger(() -> outtaking);

    // Motors
    private final TalonFX intake;
    private final TalonFX intakeTop;

    // Outputs
    public static double intakeOutput = 0;
    public static double intakeTopOutput = 0;

    public Intake() {
        intake = new TalonFX(37);
        intakeTop = new TalonFX(38);
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANifier;
        intakeConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 40;
        intakeConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        intakeConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intake.getConfigurator().apply(intakeConfig);
        outtakeTrigger.onFalse(Commands.runOnce(() -> hasPiece = false));
    }

    public void updateSensors() {
        hasPiece0 = hasPiece;
        if (Robot.scoreCoral) {
            if (intake.getForwardLimit().getValue().equals(ForwardLimitValue.Open) 
                    && !Dashboard.disableSafeties.get()) {
                if (stallTimer.getTimeMillis() > 100) {
                    hasPiece = true;
                }
            } else {
                hasPiece = false;
                stallTimer.reset();
            }
        } else {
            if (intakeOutput > 0.1 && intake.getVelocity().getValueAsDouble() < 1 
                        && !Dashboard.disableSafeties.get()) {
                if (stallTimer.getTimeMillis() > 500) {
                    hasPiece = true;
                }
            } else {
                hasPiece = false;
                stallTimer.reset();
            }
        }

        Dashboard.intakeVelocity_rpm.set(intake.getVelocity().getValueAsDouble() / 60.0);
        Dashboard.intakeTemp_C.set(intake.getDeviceTemp().getValueAsDouble());
    }

    public void intake() {
        intakeOutput = (Robot.scoreCoral) ? coralIntakeSpeed : 0.0;
        intakeTopOutput = (Robot.scoreCoral) ? coralIntakeTopSpeed : algaeIntakeSpeed;
        intaking = true;
        outtaking = false;
        Dashboard.intaking.set(true);
    }

    public void outtake() {
        intakeOutput = (Robot.scoreCoral) ? coralOuttakeSpeed : 0.0;
        intakeTopOutput = (Robot.scoreCoral) ? coralOuttakeTopSpeed : algaeOuttakeSpeed;
        intaking = false;
        outtaking = true;
        Dashboard.outtaking.set(true);
    }

    public static void stop() {
        intakeOutput = 0.0;
        intakeTopOutput = 0.0;
        intaking = false;
        outtaking = false;
        Dashboard.intaking.set(false);
        Dashboard.outtaking.set(false);
    }

    public static void hold() {
        if (Robot.scoreCoral) {
            intakeOutput = 0.0;
            intakeTopOutput = 0.0;
        } else {
            intakeOutput = 0.0;
            intakeTopOutput = 1.0;
        }
        intaking = false;
        outtaking = false;
        Dashboard.intaking.set(false);
        Dashboard.outtaking.set(false);
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
        ActuatorInterlocks.testActuatorInterlocks(intakeTop, "Intake_Top_(p)", intakeTopOutput);
    }
}
