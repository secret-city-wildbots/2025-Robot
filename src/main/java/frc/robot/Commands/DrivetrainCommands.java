package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Dashboard;
import frc.robot.Robot;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;

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

    public static Command strafeAssistScoreLeft(Drivetrain drivetrain) {
      return Commands.sequence(
        Commands.runOnce(() -> {
          Robot.scoreCoral0 = Robot.scoreCoral;
          Robot.scoreCoral = true;
          Dashboard.scoreCoral.set(Robot.scoreCoral);
          Robot.scoreRight = false;
        }, drivetrain),
        drivetrain.getAutoStrafeCorrectionCommand(false)
      );
    }

    public static Command strafeAssistScoreRight(Drivetrain drivetrain) {
      return Commands.sequence(
        Commands.runOnce(() -> {
          Robot.scoreCoral0 = Robot.scoreCoral;
          Robot.scoreCoral = true;
          Dashboard.scoreCoral.set(Robot.scoreCoral);
          Robot.scoreRight = true;
        }, drivetrain),
        drivetrain.getAutoStrafeCorrectionCommand(false)
      );
    }

    public static Command pickupFeeder(Drivetrain drivetrain, Arm arm, Intake intake) {
      return 
        Commands.parallel(
          drivetrain.getAutoStrafeCorrectionCommand(true),
          
          Commands.sequence(
            arm.pickupFeeder().until(() -> arm.closeEnough()),
            ArmCommands.autoIntake(intake, arm).until(() -> Intake.hasPiece),
            ArmCommands.stop(),
            Commands.runOnce(() -> arm.updatePivot(Rotation2d.fromDegrees((Robot.scoreCoral) ? 0:-10)), arm),
            Commands.waitUntil(() -> arm.closeEnough()),
            Commands.runOnce(() -> arm.updateWrist(Rotation2d.fromDegrees(25)), arm)
          )
      ).handleInterrupt(() -> {
          arm.updatePivot(Rotation2d.fromDegrees((Robot.scoreCoral) ? 0:-10));
          arm.updateWrist(Rotation2d.fromDegrees(25));
      });
    }
}
