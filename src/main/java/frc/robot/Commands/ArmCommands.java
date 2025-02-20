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
        return Commands.runOnce(()->arm.score(), arm);
    }

    public static Command pickup(
        Arm arm) {
            return Commands.runOnce(()->arm.pickup(), arm);
        }

    
}
