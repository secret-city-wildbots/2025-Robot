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
import frc.robot.Commands.ArmCommands;
import frc.robot.Commands.DrivetrainCommands;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Arm;
import frc.robot.Utility.FileHelpers;
import frc.robot.Utility.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.playingwithfusion.TimeOfFlight;

public class Robot extends TimedRobot {
  // Subsystems and major objects
  private final Drivetrain drivetrain;
  private final Arm arm;
  private final Intake intake;
  private final XboxController driverController;
  private final CommandXboxController driverCommandController;
  private final XboxController manipController;
  private final CommandXboxController manipCommandController;
  private final Compressor compressor;
  private final LED led;
  private Command autonomousCommand;
  Command pathfinder;
    

  @SuppressWarnings("unused")
  private final Dashboard dashboard = new Dashboard();

  public static enum MasterStates {
    STOW,
    FEED,
    SCOR,
    CLMB
  }

  public static MasterStates masterState = MasterStates.STOW;
  public static boolean scoreRight = false;
  public static boolean scoreCoral = true;

  // Major constants
  private final String codeVersion = "2025-Robot v1.1_dev";
  /** The size of the robot in the X direction (distance between wheel centers) */
  public static double robotLength_m;
  /** The size of the robot in the Y direction (distance between wheel centers) */
  public static double robotWidth_m;
  /** The size of the robot in the X direction with bumpers */
  public static double robotLengthBumpers_m;
  /** The size of the robot in the Y direction with bumpers */
  public static double robotWidthBumpers_m;
  /** The size of the field in the Y direction */
  public static double fieldWidth_m = 8.05;
  /** The size of the field in the X direction */
  public static double fieldLength_m = 17.55;
  public static final String robotProfile = FileHelpers.readFile("/home/lvuser/calibrations/RobotProfile.txt");
  private final String[] actuatorNames = { "No_Test", "Compressor_(p)", "Drive_0_(p)", "Drive_1_(p)", "Drive_2_(p)",
      "Drive_3_(p)",
      "Azimuth_0_(p)", "Azimuth_1_(p)", "Azimuth_2_(p)", "Azimuth_3_(p)", "Swerve_0_Shifter_(b)",
      "Swerve_1_Shifter_(b)", "Swerve_2_Shifter_(b)", "Swerve_3_Shifter_(b)", "Drivetrain_(p)", "Wrist_(p)", "Pivot_(p)", "Extender_(p)", "Intake_(p)"};
  public static final String[] legalDrivers = { "Devin", "Reed", "Driver 3", "Driver 4", "Driver 5", "Programmers",
      "Kidz" };

  // Looptime tracking
  public static double loopTime_ms = 20;
  private static double loopTime0 = System.currentTimeMillis();

  // Dashboard variables
  private double selectedDriver0 = 0;

  // TOF Test
  private TimeOfFlight tofSensor;
  private final int tofSensorID = 7;

  /**
   * This is called when the robot is initalized
   */
  public Robot() {
    // Set major constants using profiles
    switch (Robot.robotProfile) {
      case "2025_Robot":
        robotLength_m = Units.inchesToMeters(23);
        robotWidth_m = Units.inchesToMeters(23);
        robotLengthBumpers_m = Units.inchesToMeters(35);
        robotWidthBumpers_m = Units.inchesToMeters(35);
        break;
      case "COTS_Testbed":
        robotLength_m = Units.inchesToMeters(23);
        robotWidth_m = Units.inchesToMeters(23);
        robotLengthBumpers_m = Units.inchesToMeters(35);
        robotWidthBumpers_m = Units.inchesToMeters(35);
        break;
      default:
        robotLength_m = Units.inchesToMeters(23);
        robotWidth_m = Units.inchesToMeters(23);
        robotLengthBumpers_m = Units.inchesToMeters(35);
        robotWidthBumpers_m = Units.inchesToMeters(35);
    }
    
     // Set up subsystems and major objects
     drivetrain = new Drivetrain();
     arm = new Arm();
     intake = new Intake();
     led = new LED();
     driverController = new XboxController(0);
     driverCommandController = new CommandXboxController(0);
     manipCommandController = new CommandXboxController(1);
     manipController = new XboxController(1);
     compressor = new Compressor(2, PneumaticsModuleType.REVPH);

    // Send major constants to the Dashboard
    Dashboard.robotProfile.set(robotProfile);
    Dashboard.codeVersion.set(codeVersion);
    Dashboard.currentDriverProfileSetpoints
        .set(SwerveUtils.readDriverProfiles(legalDrivers[(int) Dashboard.selectedDriver.get()]).toDoubleArray());
    Dashboard.legalActuatorNames.set(actuatorNames);
    Dashboard.legalDrivers.set(legalDrivers);
    Dashboard.robotLengthBumpers.set(Units.metersToInches(robotLengthBumpers_m));
    Dashboard.robotWidthBumpers.set(Units.metersToInches(robotWidthBumpers_m));
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

    tofSensor = new TimeOfFlight(tofSensorID);
  }

  /**
   * This is called on every loop cycle
   */
  @Override
  public void robotPeriodic() {
    // Start by updating all sensor values
    getSensors();

    CommandScheduler.getInstance().run();

    updateMasterState();

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
    @SuppressWarnings("unused")
      double distance = tofSensor.getRange();
      // System.out.println("Distance: " + distance + " mm");
      tofSensor.identifySensor();

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
    arm.updateSensors(manipController);
    intake.updateSensors();
    Dashboard.pressureTransducer.set(compressor.getPressure());
  }

  /**
   * 
   */
  private void updateOutputs() {
    drivetrain.updateOutputs(isAutonomous(), driverController);
    led.updateOutputs();
    arm.updateOutputs();
    intake.updateOutputs();
  }

  /**
   * 
   */
  public void updateMasterState() {
    /*
     * Change master states to match these manip inputs:
     * Left Bumper: STOW
     * Right Bumper: FEED
     * Right Trigger: SCOR
     * Up joysticks & Left Trigger: CLMB
     */
    if (manipController.getLeftBumperButtonPressed()) {
      masterState = MasterStates.STOW;
    } else if (manipController.getRightBumperButtonPressed()) {
      masterState = MasterStates.FEED;
    } else if (manipController.getRightTriggerAxis() > 0.7) {
      masterState = MasterStates.SCOR;
    } else if ((manipController.getRightY() > 0.7) && (manipController.getLeftY() > 0.7) && (manipController.getLeftTriggerAxis() > 0.7)) {
      masterState = MasterStates.CLMB;
    }

    if (manipController.getRawButtonPressed(7)) {
      scoreCoral = true;
    } else if (manipController.getRawButtonPressed(8)) {
      scoreCoral = false;
    }

    if (manipController.getRightStickButtonPressed()) {
      scoreRight = true;
    } else if (manipController.getLeftStickButtonPressed()) {
      scoreRight = false;
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
            manipController));
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
    manipCommandController.axisGreaterThan(3, 0.7).onTrue(ArmCommands.score(arm));
    manipCommandController.rightBumper().onTrue(ArmCommands.pickup(arm));
    driverCommandController.axisGreaterThan(3, 0.7).onTrue(ArmCommands.outtake(intake, arm, driverController));
    driverCommandController.rightBumper().onTrue(ArmCommands.outtake(intake, arm, driverController));
    driverCommandController.axisGreaterThan(2, 0.7).onTrue(ArmCommands.intake(intake, arm, driverController));
    driverCommandController.b().onTrue(drivetrain.getFinalStrafeCorrectionCommand(drivetrain.determineGoalPose(), driverController));

  }
}