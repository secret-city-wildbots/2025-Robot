package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Gripper;

public class ArmCommands {
    private ArmCommands() {}

    /**
     * 
     */
    public static Command score(
            Arm arm) {
        return Commands.runOnce(()->arm.score(), arm);
    }

    public static Command pickup(
            Arm arm) {
            return Commands.runOnce(()->arm.pickup(), arm);
        }

    private static Command hold(
            Gripper gripper) {
        return Commands.runOnce(gripper::hold, gripper);
    }

    private static Command stop(
            Gripper gripper) {
        return Commands.runOnce(gripper::stop, gripper);
    }

    public static Command intake(
            Gripper gripper,
            Arm arm, 
        XboxController driverController) {
        return Commands.sequence(
            Commands.runOnce(() -> gripper.intake(), gripper),
            Commands.waitSeconds(0.5),
            Commands.waitUntil(() -> (gripper.hasPiece() || (driverController.getLeftTriggerAxis() < 0.7))),
            Commands.either(hold(gripper), stop(gripper), gripper::hasPiece),
            Commands.runOnce(() -> arm.stow(), arm)
        );
    }

    public static Command outtake(
            Gripper gripper,
            Arm arm,
        XboxController driverController) {
            return Commands.sequence(
                Commands.runOnce(() -> gripper.outtake(), gripper),
                Commands.waitSeconds(0.5),
                Commands.waitUntil(() -> 
                    (!driverController.getRightBumperButton() && 
                    (driverController.getRightTriggerAxis() < 0.7))),
                stop(gripper),
                Commands.runOnce(() -> arm.autoStow(), arm)
            );
        }
}
