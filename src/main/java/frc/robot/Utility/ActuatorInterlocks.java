package frc.robot.Utility;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Dashboard;

public class ActuatorInterlocks {
    private static String testingActuator;
    private static double testingPeriod;
    private static double testingValue;

    /**
     * Fetches whether or not an actuator is being tested from the Dashboard
     * 
     * @return Whether the dashboard is testing an actuator
     */
    public static boolean isTesting() {
        testingActuator = Dashboard.testActuatorName.get();
        return !testingActuator.equals("No_Test");
    }

    /**
     * Actuator testing:
     * <ul>
     * <li>If the Dashboard is not testing anything, set normal output as a
     * DutyCycleOut command
     * <li>If the Dashboard is testing this actuator:
     * <ul>
     * <li>Output the amplitude from the Dashboard as a DutyCycleOut command
     * <li>Output a time based sinusoid with amplitude from the Dashboard as a
     * DutyCycleOut command is period is given
     * </ul>
     * <li>If the Dashboard is testing a different actuator, set DutyCycleOut
     * command of 0
     * </ul>
     * 
     * This sets DutyCycleOut commands for TalonFX motors in actuator testing
     * 
     * @param motor        The TalonFX motor object to output to
     * @param actuatorName The name of the actuator matching the list in Robot.java
     * @param normalOutput The output between -1 and 1 that would go to the motor
     *                     while not in testing mode
     */
    public static void TAI_TalonFX_Power(TalonFX motor, String actuatorName, double normalOutput) {
        // Get Dashboard testing values
        testingActuator = Dashboard.testActuatorName.get();
        testingPeriod = Dashboard.testActuatorPeriod.get();
        testingValue = Dashboard.testActuatorValue.get();

        // If nothing is being tested, return the normal output. Otherwise, uses
        // dashboard amplitude as double output or periodic sine output
        if (testingActuator.equals("No_Test")) {
            motor.set(normalOutput);
        } else if (testingActuator.equals(actuatorName)) {
            if (testingPeriod < 0.001) {
                motor.set(testingValue);
            } else {
                motor.set(
                        testingValue * Math.sin((double) System.currentTimeMillis() * 0.001 * Math.PI / testingPeriod));
            }
        } else {
            motor.set(0);
        }
    }

    /**
     * Actuator testing:
     * <ul>
     * <li>If the Dashboard is not testing anything, set normal output as a
     * PositionDutyCycle command
     * <li>If the Dashboard is testing this actuator:
     * <ul>
     * <li>Output the amplitude from the Dashboard as a DutyCycleOut command
     * <li>Output a time based sinusoid with amplitude from the Dashboard as a
     * DutyCycleOut command is period is given
     * </ul>
     * <li>If the Dashboard is testing a different actuator, set DutyCycleOut
     * command of 0
     * </ul>
     * 
     * This sets DutyCycleOut commands for TalonFX motors in actuator testing
     * 
     * @param motor        The TalonFX motor object to output to
     * @param actuatorName The name of the actuator matching the list in Robot.java
     * @param normalOutput The output in rotations that would go to the motor while
     *                     not in testing mode
     * @param normalFF     The output for a feed forward (constant added power) that
     *                     would go to the motor while not in testing mode
     */
    public static void TAI_TalonFX_Position(TalonFX motor, String actuatorName, double normalOutput, double normalFF) {
        // Get Dashboard testing values
        testingActuator = Dashboard.testActuatorName.get();
        testingPeriod = Dashboard.testActuatorPeriod.get();
        testingValue = Dashboard.testActuatorValue.get();

        // If nothing is being tested, return the normal output. Otherwise, uses
        // dashboard amplitude as double output or periodic sine output
        if (testingActuator.equals("No_Test")) {
            PositionDutyCycle controlRequest = new PositionDutyCycle(normalOutput);
            controlRequest.FeedForward = normalFF;
            motor.setControl(controlRequest);
        } else if (testingActuator.equals(actuatorName)) {
            if (testingPeriod < 0.001) {
                motor.set(testingValue);
            } else {
                motor.set(
                        testingValue * Math.sin((double) System.currentTimeMillis() * 0.001 * Math.PI / testingPeriod));
            }
        } else {
            motor.set(0);
        }
    }

    /**
     * Actuator testing:
     * <ul>
     * <li>If the Dashboard is not testing anything, set normal output as a power
     * command
     * <li>If the Dashboard is testing this actuator:
     * <ul>
     * <li>Output the amplitude from the Dashboard as a power command
     * <li>Output a time based sinusoid with amplitude from the Dashboard as a power
     * command is period is given
     * </ul>
     * <li>If the Dashboard is testing a different actuator, set power command of 0
     * </ul>
     * 
     * This sets power commands for SparkMAX motors in actuator testing
     * 
     * @param motor        The SparkMAX motor object to output to
     * @param actuatorName The name of the actuator matching the list in Robot.java
     * @param normalOutput The output between -1 and 1 that would go to the motor
     *                     while not in testing mode
     */
    public static void TAI_SparkMAX_Power(CANSparkMax motor, String actuatorName, double normalOutput) {
        // Get Dashboard testing values
        testingActuator = Dashboard.testActuatorName.get();
        testingPeriod = Dashboard.testActuatorPeriod.get();
        testingValue = Dashboard.testActuatorValue.get();

        // If nothing is being tested, return the normal output. Otherwise, uses
        // dashboard amplitude as double output or periodic sine output
        if (testingActuator.equals("No_Test")) {
            motor.set(normalOutput);
        } else if (testingActuator.equals(actuatorName)) {
            if (testingPeriod < 0.001) {
                motor.set(testingValue);
            } else {
                motor.set(
                        testingValue * Math.sin((double) System.currentTimeMillis() * 0.001 * Math.PI / testingPeriod));
            }
        } else {
            motor.set(0);
        }
    }

    /**
     * Actuator testing:
     * <ul>
     * <li>If the Dashboard is not testing anything, set normal output as a
     * PositionDutyCycle command
     * <li>If the Dashboard is testing this actuator:
     * <ul>
     * <li>Output the amplitude from the Dashboard as a DutyCycleOut command
     * <li>Output a time based sinusoid with amplitude from the Dashboard as a
     * DutyCycleOut command is period is given
     * </ul>
     * <li>If the Dashboard is testing a different actuator, set DutyCycleOut
     * command of 0
     * </ul>
     * 
     * This sets DutyCycleOut commands for TalonFX motors in actuator testing
     * 
     * @param motor        The TalonFX motor object to output to
     * @param actuatorName The name of the actuator matching the list in Robot.java
     * @param normalOutput The output in rotations that would go to the motor while
     *                     not in testing mode
     */
    public static void TAI_SparkMAX_Position(CANSparkMax motor, SparkPIDController pidController, String actuatorName,
            double normalOutput, double normalFF) {
        // Get Dashboard testing values
        testingActuator = Dashboard.testActuatorName.get();
        testingPeriod = Dashboard.testActuatorPeriod.get();
        testingValue = Dashboard.testActuatorValue.get();

        // If nothing is being tested, return the normal output. Otherwise, uses
        // dashboard amplitude as double output or periodic sine output
        if (testingActuator.equals("No_Test")) {
            pidController.setReference(normalOutput, CANSparkBase.ControlType.kPosition);
            pidController.setFF(normalFF);
        } else if (testingActuator.equals(actuatorName)) {
            if (testingPeriod < 0.001) {
                motor.set(testingValue);
            } else {
                motor.set(
                        testingValue * Math.sin((double) System.currentTimeMillis() * 0.001 * Math.PI / testingPeriod));
            }
        } else {
            motor.set(0);
        }
    }

    /**
     * Actuator testing:
     * <ul>
     * <li>If the Dashboard is not testing anything, set normal output (true is
     * forward, false is reverse)
     * <li>If the Dashboard is testing this actuator, set true or false if the
     * Dashboard amplitude is 0 or 1 respectively
     * <li>If the Dashboard is testing a different actuator, set 0 (false / reverse)
     * </ul>
     * <br>
     * 
     * This sets a boolean output for a solenoid/piston
     * 
     * @param solenoid     The solenoid to output to
     * @param actuatorName The name of the actuator matching the list in Robot.java
     * @param normalOutput The output (true or false) that would go to the solenoid
     *                     while not in testing mode
     */
    public static void TAI_Solenoids(DoubleSolenoid solenoid, String actuatorName, boolean normalOutput) {
        // Get dashboard test values
        testingActuator = Dashboard.testActuatorName.get();
        testingValue = Dashboard.testActuatorValue.get();

        Value normalValue = (normalOutput) ? Value.kForward : Value.kReverse;

        // If nothing is being tested, return the normal output. Otherwise, uses
        // dashboard amplitude as 1 or 0 for an output
        if (testingActuator.equals("No_Test")) {
            solenoid.set(normalValue);
        } else if (testingActuator.equals(actuatorName)) {
            if (testingValue > 0.9) {
                solenoid.set(Value.kForward);
            } else {
                solenoid.set(Value.kReverse);
            }
        } else {
            solenoid.set(Value.kReverse);
        }
    }

}
