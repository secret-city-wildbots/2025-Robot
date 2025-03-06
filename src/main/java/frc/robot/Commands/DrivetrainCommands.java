package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
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
            XboxController manipController) {
        return Commands.run(
                () -> {
                    driveSystem.driveTeleop(
                            manipController);
                },
                driveSystem);
    }
    
    private static Pose2d storedGoalPose = new Pose2d();

    public static Command strafeAssistScoreLeft(Drivetrain drivetrain) {
      return Commands.sequence(
        Commands.runOnce(() -> {
          Robot.scoreRight = false;
          storedGoalPose = drivetrain.determineGoalPose();
        }, drivetrain),
        drivetrain.getFinalStrafeCorrectionCommand()
      );
    }
}
