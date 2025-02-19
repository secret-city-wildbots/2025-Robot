// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DrivetrainCommands;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utility.FileHelpers;
import frc.robot.Utility.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class Robot extends TimedRobot {
  // Subsystems and major objects
  private final Drivetrain drivetrain;
  private final XboxController driverController;
  private final XboxController manipController;
  private final Compressor compressor;
  private final LED led;
  private Command autonomousCommand;
  private final CommandXboxController commanddriverController;
  Command pathfinder;
    

  @SuppressWarnings("unused")
  private final Dashboard dashboard = new Dashboard();

  public static enum MasterStates {
    STOWED
  }

  public static MasterStates masterState = MasterStates.STOWED;

  // Major constants
  private final String codeVersion = "2025-Robot v1.1_dev";
  public static double robotLength_m;
  public static double robotWidth_m;
  public static double robotLengthBumpers;
  public static double robotWidthBumpers;
  public static double fieldWidth_m = 8.05;
  public static double fieldLength_m = 17.55;
  public static final String robotProfile = FileHelpers.readFile("/home/lvuser/calibrations/RobotProfile.txt");
  private final String[] actuatorNames = { "No_Test", "Compressor_(p)", "Drive_0_(p)", "Drive_1_(p)", "Drive_2_(p)",
      "Drive_3_(p)",
      "Azimuth_0_(p)", "Azimuth_1_(p)", "Azimuth_2_(p)", "Azimuth_3_(p)", "Swerve_0_Shifter_(b)",
      "Swerve_1_Shifter_(b)", "Swerve_2_Shifter_(b)", "Swerve_3_Shifter_(b)", "Drivetrain_(p)" };
  public static final String[] legalDrivers = { "Devin", "Reed", "Driver 3", "Driver 4", "Driver 5", "Programmers",
      "Kidz" };

  // Looptime tracking
  public static double loopTime_ms = 20;
  private static double loopTime0 = System.currentTimeMillis();

  // Dashboard variables
  private double selectedDriver0 = 0;

  /**
   * This is called when the robot is initalized
   */
  public Robot() {
    // Set major constants using profiles
    switch (Robot.robotProfile) {
      case "2025_Robot":
        robotLength_m = Units.inchesToMeters(23);
        robotWidth_m = Units.inchesToMeters(23);
        robotLengthBumpers = Units.inchesToMeters(35);
        robotWidthBumpers = Units.inchesToMeters(35);
        break;
      case "COTS_Testbed":
        robotLength_m = Units.inchesToMeters(23);
        robotWidth_m = Units.inchesToMeters(23);
        break;
      default:
        robotLength_m = Units.inchesToMeters(23);
        robotWidth_m = Units.inchesToMeters(23);
    }
    
     // Set up subsystems and major objects
     drivetrain = new Drivetrain();
     led = new LED();
     driverController = new XboxController(0);
     manipController = new XboxController(1);
     compressor = new Compressor(2, PneumaticsModuleType.REVPH);
     commanddriverController = new CommandXboxController(0);

    // Send major constants to the Dashboard
    Dashboard.robotProfile.set(robotProfile);
    Dashboard.codeVersion.set(codeVersion);
    Dashboard.currentDriverProfileSetpoints
        .set(SwerveUtils.readDriverProfiles(legalDrivers[(int) Dashboard.selectedDriver.get()]).toDoubleArray());
    Dashboard.legalActuatorNames.set(actuatorNames);
    Dashboard.legalDrivers.set(legalDrivers);
    Dashboard.robotLengthBumpers.set(Units.metersToInches(robotLengthBumpers));
    Dashboard.robotWidthBumpers.set(Units.metersToInches(robotWidthBumpers));
    Dashboard.fieldWidth.set(Units.metersToInches(fieldWidth_m));
    Dashboard.fieldLength.set(Units.metersToInches(fieldLength_m));
    
    // Configure compressor
    compressor.enableAnalog(100, 120);

    // Configure CommandBased
    configureDefaultCommands();

    registerNamedCommands();

    configureButtonBindings();
  }

  // Get sensors to avoid faulty calls to null objects in periodic functions
  @Override
  public void robotInit() {
    getSensors();
    FollowPathCommand.warmupCommand().schedule();
  }

  /**
   * This is called on every loop cycle
   */
  @Override
  public void robotPeriodic() {
    // Start by updating all sensor values
    getSensors();

    CommandScheduler.getInstance().run();

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

    updateLoopTime();
    Dashboard.loopTime.set(loopTime_ms);
  }

  /**
   * This is called every loop cycle while the robot is disabled
   */
  @Override
  public void disabledPeriodic() {
    updateOutputs();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = new PathPlannerAuto("Example Auto");

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /**
   * This is called every loop cycle while the robot is enabled in autonomous mode
   */
  @Override
  public void autonomousPeriodic() {
    updateOutputs();
  }

  /**
   * This is called every loop cycle while the robot is enabled in TeleOp mode
   */
  @Override
  public void teleopPeriodic() {
    pathfinder = drivetrain.getPathFindingCommand(new Pose2d(Units.inchesToMeters(134), Units.inchesToMeters(50), new Rotation2d()));
    if (driverController.getBButtonPressed()) {
      pathfinder.schedule();
      System.out.println(pathfinder);
    }


    // commanddriverController.b().onTrue(pathfinder);


    // Check for state updates based on manip inputs
    updateMasterState();

    // Adjust LED color settings based on mode using driver controller
    led.updateLED(driverController, false);

    // Update outputs for everything
    // This includes all motors, pistons, and other things
    updateOutputs();
  }

  /**
   * 
   */
  private void getSensors() {
    drivetrain.updateSensors(driverController);
    Dashboard.pressureTransducer.set(compressor.getPressure());
  }

  /**
   * 
   */
  private void updateOutputs() {
    drivetrain.updateOutputs(isAutonomous(), driverController);
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
    }
  }

  public static void updateLoopTime() {
    loopTime_ms = System.currentTimeMillis() - loopTime0;
    loopTime0 = System.currentTimeMillis();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        DrivetrainCommands.drive(
            drivetrain,
            driverController,
            isAutonomous(),
            loopTime_ms));
  }

  /**
   * Use this method to register named commands for path planner.
   */
  private void registerNamedCommands() {

  }

  /**
   * Use this method to define your button -> command mappings. Buttons can be
   * created by passing XBoxController into a new JoystickButton
   */
  private void configureButtonBindings() {

  }
}