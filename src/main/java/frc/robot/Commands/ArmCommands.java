package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.Robot.MasterStates;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Drivetrain;
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
            return Commands.sequence(
                Commands.runOnce(() -> arm.pickupFeeder_init(), arm),
                Commands.waitUntil(() -> arm.closeEnough()).withTimeout(1),
                Commands.runOnce(()->arm.pickup(), arm)
            );
        }

    public static Command stow(Arm arm) {
        return Commands.runOnce(() -> arm.drivingStow());
        // return Commands.print("Oh no! We're tipping!!!! or this is broken, idk?");
    }

    public static Command climb(
        Arm arm) {
            return Commands.sequence(
                Commands.runOnce(arm::climbInit, arm),
                Commands.parallel(
                    Commands.waitUntil(() -> Robot.driverController.getAButtonPressed()),
                    Commands.waitUntil(arm::hasArrived)
                ),
                Commands.runOnce(arm::climbLift, arm)
            );
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
    public static Command stop(
            Intake intake) {
        return Commands.runOnce(intake::stop, intake);
    }

    public static Command groundPickup(Arm arm) {
        return Commands.sequence(
            Commands.runOnce(() -> Robot.scoreCoral = false),
            Commands.runOnce(arm::scoringStow, arm),
            Commands.waitUntil(() -> arm.closeEnough()),
            Commands.runOnce(arm::groundPickup, arm)
        );
    }

    /**
     * Intakes a piece until driver stops holding left trigger or a game piece is acquired
     * Then stows the arm
     * Only holds piece in with constant intake if intake has a piece
     * @param intake
     * @param arm
     * @return
     */
    public static Command intake(
            Intake intake,
            Arm arm) {
        return
                Commands.sequence(
                    Commands.runOnce(() -> intake.intake(), intake),
                    Commands.parallel(
                        Commands.waitSeconds(1),
                        Commands.waitUntil(() -> Robot.driverController.getLeftTriggerAxis() < 0.7)
                    ),
                    Commands.either(hold(intake), stop(intake), intake::hasPiece),
                    Commands.either(
                        Commands.runOnce(() -> Robot.masterState = MasterStates.STOW),
                        Commands.none(),
                        () -> intake.hasPiece() && 
                        (Robot.masterState.equals(MasterStates.STOW) || 
                        Robot.masterState.equals(MasterStates.FEED))
                    )
                );
    }

    public static Command autoIntake(
            Intake intake,
            Arm arm,
            double timeout) {
        return
                Commands.sequence(
                    Commands.runOnce(() -> intake.intake(), intake),
                    Commands.waitSeconds(timeout),
                    Commands.either(hold(intake), stop(intake), intake::hasPiece),
                    Commands.either(
                        Commands.runOnce(() -> Robot.masterState = MasterStates.STOW),
                        Commands.none(),
                        () -> intake.hasPiece() && 
                        (Robot.masterState.equals(MasterStates.STOW) || 
                        Robot.masterState.equals(MasterStates.FEED))
                    )
                );
    }

    /**
     * 
     * @param intake
     * @param arm
     * @return
     */
    public static Command outtake(
            Intake intake,
            Arm arm) {
            return Commands.sequence(
                        Commands.waitUntil(() -> arm.hasArrived() || Robot.driverController.getRightBumperButton()),
                        Commands.runOnce(() -> intake.outtake(), intake),
                        Commands.waitSeconds(0.5),
                        Commands.waitUntil(() -> 
                            (!Robot.driverController.getRightBumperButton() && 
                            (Robot.driverController.getRightTriggerAxis() < 0.7))),
                        stop(intake),
                        Commands.runOnce(() -> Robot.masterState = MasterStates.STOW)
                    );
        }

    public static Command scoreL4(
            Arm arm,
            Intake intake,
            Drivetrain drivetrain) {
                return Commands.sequence(
                    Commands.runOnce(() -> {Robot.scoreCoral = true; Arm.scoreHeight = 4;}),
                    Commands.sequence(
                        Commands.runOnce(() -> 
                            {arm.updatePivot(Rotation2d.fromDegrees(-5));
                            arm.updateExtender(Units.inchesToMeters(37.1));}),
                        Commands.waitUntil(() -> arm.closeEnough()),
                        Commands.runOnce(() -> arm.updateWrist(Rotation2d.fromDegrees(67)))
                    ),
                    Commands.race(
                        Commands.sequence(
                            Commands.waitUntil(() -> drivetrain.poseAccuracyGetter()),
                            Commands.waitSeconds(0.5)
                        ),
                        Commands.waitSeconds(3)
                    ),
                    outtake(intake, arm),
                    Commands.sequence(
                        Commands.runOnce(() -> {
                            arm.updatePivot(Rotation2d.fromDegrees(-25));
                            arm.updateWrist(Rotation2d.fromDegrees(25));
                        }, arm),
                        Commands.waitUntil(() -> arm.closeEnough()),
                        Commands.runOnce(() -> arm.updateExtender(0.0))
                    )
                );
            }
}
