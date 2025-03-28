package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Dashboard;
import frc.robot.LED;
import frc.robot.Robot;
import frc.robot.Robot.MasterStates;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;

public class ArmCommands {
    private ArmCommands() {}

    public static Command score(Arm arm) {
        return Commands.runOnce(() -> arm.score().schedule());
    }

    public static Command stow(Arm arm) {
        return Commands.runOnce(() -> arm.drivingStow().schedule());
        // return Commands.print("Oh no! We're tipping!!!! or this is broken, idk?");
    }

    public static Command climb(
        Arm arm) {
            return 
            Commands.sequence(
                arm.climbInit(),
                Commands.parallel(
                    Commands.waitUntil(() -> Robot.driverController.getAButtonPressed()),
                    Commands.waitUntil(arm::hasArrived)
                ),
                arm.climb2(),
                Commands.parallel(
                    Commands.waitUntil(() -> Robot.driverController.getAButtonPressed()),
                    Commands.waitUntil(arm::hasArrived)
                ),
                arm.climbLift()
            );
    }

    public static Command pickup(Arm arm) {
        return Commands.runOnce(() -> arm.pickup().schedule());
    }

    /**
     * Commands the intake to pull constantly to hold pieces
     * @param intake
     * @return
     */
    private static Command hold() {
        return Commands.runOnce(Intake::hold);
    }

    /**
     * Commands the intake to not run the motor whatsoever
     * @param intake
     * @return
     */
    public static Command stop() {
        return Commands.runOnce(Intake::stop);
    }

    public static Command groundPickup(Arm arm) {
        return 
        Commands.sequence(
            Commands.race(
                Commands.either(
                    arm.groundPickupCoral().until(() -> Intake.hasPiece && 
                        Robot.driverController.getLeftTriggerAxis() < 0.7),
                    arm.groundPickupAlgae().until(() -> Intake.hasPiece && 
                        Robot.driverController.getLeftTriggerAxis() < 0.7),
                    () -> Robot.scoreCoral
                ),
                Commands.sequence(
                    Commands.waitUntil(() -> Robot.driverController.getLeftTriggerAxis() > 0.7),
                    Commands.waitUntil(() -> Robot.driverController.getLeftTriggerAxis() < 0.7)
                )
            ),
            Commands.either(
                LED.hasPieceBlink(), //blink LEDs when robot has piece
                Commands.startEnd(() -> {}, () -> {}),
                () -> Intake.hasPiece
            ),
            Commands.runOnce(() -> Intake.hold()),
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> {
                            Robot.masterState0 = Robot.masterState;
                            Robot.masterState = MasterStates.STOW;
                        }),    
                    arm.scoringStow()
                ),
                Commands.runOnce(() -> {
                        Robot.scoreCoral = true;
                        Dashboard.scoreCoral.set(Robot.scoreCoral);
                        Robot.masterState0 = Robot.masterState;
                        Robot.masterState = MasterStates.FEED;
                    }),
                () -> Intake.hasPiece
            )
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
                    Commands.either(hold(), stop(), intake::hasPiece),
                    Commands.either(
                        Commands.runOnce(() -> {Robot.masterState0 = Robot.masterState;
                                                Robot.masterState = MasterStates.STOW;}),
                        Commands.none(),
                        () -> intake.hasPiece() && 
                        (Robot.masterState.equals(MasterStates.STOW) || 
                        Robot.masterState.equals(MasterStates.FEED))
                    )
                );
    }

    public static Command autoIntake(
            Intake intake,
            Arm arm) {
        return
                Commands.sequence(
                    Commands.runOnce(() -> intake.intake(), intake),
                    Commands.waitUntil(() -> Drivetrain.poseAccuracyGetter()).withTimeout(4),
                    Commands.waitSeconds(1),
                    Commands.either(hold(), stop(), intake::hasPiece)
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
                        Commands.waitUntil(() -> arm.hasArrived() && Drivetrain.poseAccuracyGetter() || Robot.driverController.getRightBumperButton()),
                        Commands.runOnce(() -> intake.outtake(), intake),
                        Commands.waitSeconds(0.5),
                        Commands.waitUntil(() -> 
                            (!Robot.driverController.getRightBumperButton() && 
                            (Robot.driverController.getRightTriggerAxis() < 0.7))),
                        stop(),
                        Commands.runOnce(() -> {Robot.masterState0 = Robot.masterState;
                                                Robot.masterState = MasterStates.STOW;})
                    );
        }

    public static Command scoreL4(
            Arm arm,
            Intake intake,
            Drivetrain drivetrain) {
                return Commands.sequence(
                    Commands.runOnce(() -> {Robot.scoreCoral = true; Dashboard.scoreCoral.set(Robot.scoreCoral); Arm.scoreHeight = 4;}),
                    Commands.sequence(
                        Commands.runOnce(() -> 
                            {arm.updatePivot(Rotation2d.fromDegrees(-5));
                            arm.updateExtender(Units.inchesToMeters(37.1));}),
                        Commands.waitUntil(() -> arm.closeEnough()),
                        Commands.runOnce(() -> arm.updateWrist(Rotation2d.fromDegrees(67)))
                    ),
                    Commands.race(
                        Commands.sequence(
                            Commands.waitUntil(() -> Drivetrain.poseAccuracyGetter()),
                            Commands.waitSeconds(0.5)
                        ),
                        Commands.waitSeconds(4)
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
