// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utility.FileHelpers;
import frc.robot.Utility.SwerveUtils;
import edu.wpi.first.math.util.Units;

public class Robot extends TimedRobot {

  private final String codeVersion = "2025-Robot v1.0_dev";

  public static enum MasterStates {
    STOWED,
    SHOOTING,
    AMP,
    CLIMBING,
    TRAP
  }

  public static MasterStates masterState = MasterStates.STOWED;

  public static double robotLength_m;
  public static double robotWidth_m;

  public static double loopTime = 20;

  private final Drivetrain drivetrain;
  private final XboxController driverController = new XboxController(0);
  private final XboxController manipController = new XboxController(1);
  private final Compressor compressor = new Compressor(2, PneumaticsModuleType.REVPH);
  private final LED led = new LED();
  private static double loopTime0 = System.currentTimeMillis();

  public static final String robotProfile = FileHelpers.readFile("/home/lvuser/calibrations/RobotProfile.txt");

  @SuppressWarnings("unused")
  private final Dashboard dashboard = new Dashboard();

  private final String[] actuatorNames = { "No_Test", "Compressor_(p)", "Drive_0_(p)", "Drive_1_(p)", "Drive_2_(p)", "Drive_3_(p)",
      "Azimuth_0_(p)", "Azimuth_1_(p)", "Azimuth_2_(p)", "Azimuth_3_(p)", "Swerve_0_Shifter_(b)",
      "Swerve_1_Shifter_(b)", "Swerve_2_Shifter_(b)", "Swerve_3_Shifter_(b)"};
  public static final String[] legalDrivers = { "Devin", "Reed", "Driver 3", "Driver 4", "Driver 5", "Programmers",
      "Kidz" };

  private double selectedDriver0 = 0;

  /**
   * This is called when the robot is initalized
   */
  public Robot() {
    // Initialize dashboard values
    Dashboard.legalActuatorNames.set(actuatorNames);
    Dashboard.legalDrivers.set(legalDrivers);
    switch (Robot.robotProfile) {
      case "2024_Robot":
        robotLength_m = Units.inchesToMeters(19);
        robotWidth_m = Units.inchesToMeters(23);
        break;
      case "Steve2":
        robotLength_m = Units.inchesToMeters(19);
        robotWidth_m = Units.inchesToMeters(23);
        break;
      default:
        robotLength_m = Units.inchesToMeters(19);
        robotWidth_m = Units.inchesToMeters(23);
    }
    drivetrain = new Drivetrain();
    Dashboard.robotProfile.set(robotProfile);
    Dashboard.codeVersion.set(codeVersion);
    Dashboard.currentDriverProfileSetpoints
        .set(SwerveUtils.readDriverProfiles(legalDrivers[(int) Dashboard.selectedDriver.get()]).toDoubleArray());
  }

  /**
   * This is called on every loop cycle
   */
  @Override
  public void robotPeriodic() {

    boolean[] confirmedStates = new boolean[5];
    confirmedStates[masterState.ordinal()] = true;
    Dashboard.confirmedMasterStates.set(confirmedStates);
    Dashboard.isAutonomous.set(isAutonomous());
    double selectedDriver = Dashboard.selectedDriver.get();
    if (selectedDriver != selectedDriver0) {
      selectedDriver0 = selectedDriver;
      Dashboard.currentDriverProfileSetpoints
          .set(SwerveUtils.readDriverProfiles(legalDrivers[(int) Dashboard.selectedDriver.get()]).toDoubleArray());
    }
    if (Dashboard.applyProfileSetpoints.get()) {
      double[] setpoints = Dashboard.newDriverProfileSetpoints.get();
      SwerveUtils.updateDriverProfile(setpoints);
      Dashboard.currentDriverProfileSetpoints.set(setpoints);
    }

    drivetrain.updateSensors();

    drivetrain.drive(driverController, isAutonomous(), loopTime);

    drivetrain.updateOutputs(isAutonomous());

    updateLoopTime();
    Dashboard.loopTime.set(loopTime);
  }

  /**
   * This is called every loop cycle while the robot is disabled
   */
  @Override
  public void disabledPeriodic() {
    
  }

  /**
   * This is called every loop cycle while the robot is enabled in autonomous mode
   */
  @Override
  public void autonomousPeriodic() {
    drivetrain.updateOdometry();
  }

  /**
   * This is called every loop cycle while the robot is enabled in TeleOp mode
   */
  @Override
  public void teleopPeriodic() {
    // Enable compressor
    compressor.enableAnalog(100, 120);

    // Start by updating all sensor values
    getSensors();

    // Check for state updates based on manip inputs
    updateMasterState();

    // Adjust LED color settings based on mode using driver controller
    led.updateLED(driverController, isAutonomous());

    // Update outputs for everything
    // This includes all motors, pistons, and other things
    updateOutputs();
  }

  /**
   * 
   */
  private void getSensors() {
    drivetrain.updateSensors();
    Dashboard.pressureTransducer.set(compressor.getPressure());
  }

  /**
   * 
   */
  private void updateOutputs() {
    drivetrain.updateOutputs(isAutonomous());
    led.updateOutputs();
  }

  /**
   * 
   */
  public void updateMasterState() {
    /*
     * Change master states to match these manip inputs:
     * Left Bumper: STOW
     * Right Bumper: AMP
     * Right Trigger: SHOOTING
     * Left Trigger & Start Button: CLIMBING
     */
    if (manipController.getLeftBumper()) {
      masterState = MasterStates.STOWED;
    } else if (masterState != MasterStates.CLIMBING && manipController.getRightBumper()) {
      masterState = MasterStates.AMP;
    } else if (masterState != MasterStates.CLIMBING && manipController.getRightTriggerAxis() > 0.7) {
      masterState = MasterStates.SHOOTING;
    } else if (manipController.getLeftTriggerAxis() > 0.7 && manipController.getStartButton()) {
      masterState = MasterStates.CLIMBING;
    }
  }



  public static void updateLoopTime() {
    loopTime = System.currentTimeMillis()-loopTime0;
    loopTime0 = System.currentTimeMillis();
  }
}
