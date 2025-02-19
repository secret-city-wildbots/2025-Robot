package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.Arm;

public class ArmCommands {
    private ArmCommands() {}

    /**
     * 
     */
    public static Command score(
            Arm arm) {
        return Commands.startEnd(
                () -> arm.score(),
                arm);
    }

    public static Command pickup(
        Arm arm) {
            return new FunctionalCommand(() -> arm.pickup(), () -> {}, () -> arm.stow(), , arm);
        }
}
