package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;

public class ArmCommands {
    private ArmCommands() {}

    /**
     * Sends the arm to the stored scoring height
     * @param arm
     * @return
     */
    public static Command score(
            Arm arm) {
        return Commands.runOnce(()->arm.score(), arm);
    }

    /**
     * Sends the arm to the stored pickup height (always feeder(1) for coral)
     * @param arm
     * @return
     */
    public static Command pickup(
            Arm arm) {
            return Commands.runOnce(()->arm.pickup(), arm);
        }

    /**
     * Commands the intake to pull constantly to hold pieces
     * @param intake
     * @return
     */
    private static Command hold(
            Intake intake) {
        return Commands.runOnce(intake::hold, intake);
    }

    /**
     * Commands the intake to not run the motor whatsoever
     * @param intake
     * @return
     */
    private static Command stop(
            Intake intake) {
        return Commands.runOnce(intake::stop, intake);
    }

    /**
     * Intakes a piece until driver stops holding left trigger or a game piece is acquired
     * Then stows the arm
     * Only holds piece in with constant intake if intake has a piece
     * @param intake
     * @param arm
     * @param driverController
     * @return
     */
    public static Command intake(
            Intake intake,
            Arm arm, 
        XboxController driverController) {
        return Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> intake.intake(), intake),
                    Commands.waitSeconds(0.5),
                    Commands.waitUntil(() -> (intake.hasPiece() || (driverController.getLeftTriggerAxis() < 0.7))),
                    Commands.either(hold(intake), stop(intake), intake::hasPiece),
                    Commands.runOnce(() -> arm.stow(), arm)
                ),

                Commands.sequence(
                    Commands.runOnce(() -> intake.intake(), intake),
                    Commands.waitSeconds(0.5),
                    Commands.waitUntil(() -> driverController.getLeftTriggerAxis() < 0.7),
                    Commands.either(hold(intake), stop(intake), intake::hasPiece)
                ),

                () -> !intake.hasPiece()
        );
    }

    /**
     * 
     * @param intake
     * @param arm
     * @param driverController
     * @return
     */
    public static Command outtake(
            Intake intake,
            Arm arm,
        XboxController driverController) {
            return Commands.sequence(
                        Commands.runOnce(() -> intake.outtake(), intake),
                        Commands.waitSeconds(0.5),
                        Commands.waitUntil(() -> 
                            (!driverController.getRightBumperButton() && 
                            (driverController.getRightTriggerAxis() < 0.7))),
                        stop(intake),
                        Commands.runOnce(() -> arm.autoStow(), arm)
                    );
        }
}
