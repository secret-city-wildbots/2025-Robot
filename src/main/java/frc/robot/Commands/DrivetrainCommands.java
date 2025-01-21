package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drivetrain;

public class DrivetrainCommands {
    private DrivetrainCommands() {}

    /**
     * Field relative drive command using two joysticks (controlling linear and
     * angular velocities).
     */
    public static Command drive(
            Drivetrain driveSystem,
            XboxController driveController,
            boolean isAutonomous,
            double period_ms) {
        return Commands.run(
                () -> {
                    driveSystem.driveTeleop(
                            driveController,
                            isAutonomous,
                            period_ms);
                },
                driveSystem);
    }
}
