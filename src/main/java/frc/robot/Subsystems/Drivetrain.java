package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;
import frc.robot.Robot;
import frc.robot.Robot.MasterStates;
import frc.robot.Subsystems.SwerveModule.ShiftedStates;
import frc.robot.Utility.Control;
import frc.robot.Utility.SwerveUtils;
// import frc.robot.Utility.ClassHelpers.DriverProfile;
import frc.robot.Utility.ClassHelpers.Latch;
// import frc.robot.Utility.ClassHelpers.LimitAcceleration;
import frc.robot.Utility.ClassHelpers.StickyButton;
import edu.wpi.first.math.util.Units;

public class Drivetrain extends SubsystemBase{
  public static double driveHighGearRatio;
  public static double driveLowGearRatio;
  public static double azimuthGearRatio;
  public static double maxGroundSpeed_mPs;
  public static double maxLowGearSpeed_mPs;
  public static double maxRotateSpeed_radPs;
  public static double actualWheelDiameter_m;
  public static double nominalWheelDiameter_m;
  public static boolean invertDriveDirection = false;
  public static boolean invertAzimuthDirection = false;
  public static boolean azimuthSparkEnabled;

  private final TalonFXConfiguration[] driveConfigs;
  private final TalonFXConfiguration[] azimuthConfigs;

  private SwerveModuleState[] moduleStates = new SwerveModuleState[4];

  private final SwerveModule module0;
  private final SwerveModule module1;
  private final SwerveModule module2;
  private final SwerveModule module3;

  private final Translation2d module0Location_m = new Translation2d(0.305, -0.305);
  private final Translation2d module1Location_m = new Translation2d(0.305, 0.305);
  private final Translation2d module2Location_m = new Translation2d(-0.305, 0.305);
  private final Translation2d module3Location_m = new Translation2d(-0.305, -0.305);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(module0Location_m, module1Location_m,
      module2Location_m, module3Location_m);

  private final Pigeon2 pigeon = new Pigeon2(6);

  public final SwerveDriveOdometry odometry;

  SlewRateLimiter xAccelerationLimiter = new SlewRateLimiter(4.4, -1000, 0.0);
  SlewRateLimiter yAccelerationLimiter = new SlewRateLimiter(4.4, -1000, 0.0);

  private SwerveModuleState[] moduleStateOutputs = new SwerveModuleState[4];

  // Used for modeDrivebase to check if master states changed
  private Robot.MasterStates masterState0 = Robot.masterState;

  public double swerveGroundSpeed = 0;
  public static boolean shiftingEnabled = false;
  @SuppressWarnings("unused")
  private boolean headingLocked = false;
  private final PIDController strafePID = new PIDController(0, 0, 0);

  // Variables stored for the assist heading function
  private StickyButton highSpeedSticky = new StickyButton();
  private boolean headingAssist = false;
  private Latch headingLatch = new Latch(0.0);
  private PIDController antiDriftPID = new PIDController(0, 0, 0);
  private PIDController headingAnglePID = new PIDController(0, 0, 0);
  private boolean headingLatchSignal0 = false;
  private double headingFudgeTime = System.currentTimeMillis();
  private double driverHeadingFudge0 = 0.0;
  private StickyButton noRotationSticky = new StickyButton();
  private boolean lockHeading0 = false;
  private final double headingFudgeMax = 10; // degrees

  public SwerveDriveOdometry updateOdometry() {
    odometry.update(
        pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            module0.getPosition(),
            module1.getPosition(),
            module2.getPosition(),
            module3.getPosition()
        });
    return odometry;
  }

  public Drivetrain() {
    // Check for driver profile and set constants
    switch (Robot.robotProfile) {
      case "2025_Robot":
        nominalWheelDiameter_m = Units.inchesToMeters(3);
        actualWheelDiameter_m = Units.inchesToMeters(3);
        maxGroundSpeed_mPs = Units.feetToMeters(18.8 * (actualWheelDiameter_m / nominalWheelDiameter_m));
        maxLowGearSpeed_mPs = Units.feetToMeters(9.2 * (actualWheelDiameter_m / nominalWheelDiameter_m));
        maxRotateSpeed_radPs = maxGroundSpeed_mPs
            / ((Math.sqrt(Math.pow(Robot.robotLength_m / 2, 2) + Math.pow(Robot.robotWidth_m / 2, 2))));
        driveHighGearRatio = 4.54;
        driveLowGearRatio = 10;
        azimuthGearRatio = 35.45;
        shiftingEnabled = true;
        azimuthSparkEnabled = true;
        invertDriveDirection = false;
        invertAzimuthDirection = false;
        driveConfigs = SwerveUtils.swerveModuleDriveConfigs();
        azimuthConfigs = SwerveUtils.swerveModuleAzimuthConfigs();
        break;
      case "COTS_Testbed":
        nominalWheelDiameter_m = Units.inchesToMeters(4);
        actualWheelDiameter_m = Units.inchesToMeters(4);
        maxGroundSpeed_mPs = Units.feetToMeters(17.8 * (actualWheelDiameter_m / nominalWheelDiameter_m));
        maxLowGearSpeed_mPs = Units.feetToMeters(17.8 * (actualWheelDiameter_m / nominalWheelDiameter_m));
        maxRotateSpeed_radPs = maxGroundSpeed_mPs
            / ((Math.sqrt(Math.pow(Robot.robotLength_m / 2, 2) + Math.pow(Robot.robotWidth_m / 2, 2))));
        driveHighGearRatio = 0.0; // Bc no shifting
        driveLowGearRatio = 6.12;
        azimuthGearRatio = 150 / 7;
        shiftingEnabled = false;
        azimuthSparkEnabled = false;
        invertDriveDirection = true;
        invertAzimuthDirection = false;
        driveConfigs = SwerveUtils.swerveModuleDriveConfigs();
        azimuthConfigs = SwerveUtils.swerveModuleAzimuthConfigs();
        break;
      default:
        nominalWheelDiameter_m = Units.inchesToMeters(5);
        actualWheelDiameter_m = Units.inchesToMeters(4.78);
        maxGroundSpeed_mPs = Units.feetToMeters(17.8 * (actualWheelDiameter_m / nominalWheelDiameter_m));
        maxLowGearSpeed_mPs = Units.feetToMeters(8.3 * (actualWheelDiameter_m / nominalWheelDiameter_m));
        maxRotateSpeed_radPs = maxGroundSpeed_mPs
            / ((Math.sqrt(Math.pow(Robot.robotLength_m / 2, 2) + Math.pow(Robot.robotWidth_m / 2, 2))));
        driveHighGearRatio = 6.42;
        driveLowGearRatio = 14.12;
        azimuthGearRatio = 15.6;
        shiftingEnabled = true;
        azimuthSparkEnabled = true;
        invertDriveDirection = false;
        invertAzimuthDirection = false;
        driveConfigs = SwerveUtils.swerveModuleDriveConfigs();
        azimuthConfigs = SwerveUtils.swerveModuleAzimuthConfigs();
    }

    // Defining all modules 
    module0 = new SwerveModule(driveHighGearRatio, driveLowGearRatio, azimuthGearRatio, 0, driveConfigs[0],
        azimuthConfigs[0]);
    module1 = new SwerveModule(driveHighGearRatio, driveLowGearRatio, azimuthGearRatio, 1, driveConfigs[1],
        azimuthConfigs[1]);
    module2 = new SwerveModule(driveHighGearRatio, driveLowGearRatio, azimuthGearRatio, 2, driveConfigs[2],
        azimuthConfigs[2]);
    module3 = new SwerveModule(driveHighGearRatio, driveLowGearRatio, azimuthGearRatio, 3, driveConfigs[3],
        azimuthConfigs[3]);

    // Odometry object for tracking robot position using kinematics
    odometry = new SwerveDriveOdometry(
        kinematics, pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            module0.getPosition(),
            module1.getPosition(),
            module2.getPosition(),
            module3.getPosition()
        });

    /* Setting up angle wrapping for control PIDs
       Makes it so that 2pi = 0 and automatic direct angle calculations*/ 
    antiDriftPID.enableContinuousInput(0, 2*Math.PI);
    headingAnglePID.enableContinuousInput(0, 2*Math.PI);

    // Set up initial module states for init and constructors
    for (int i = 0; i < 4; i++) {
      moduleStateOutputs[i] = new SwerveModuleState();
    }

    RobotConfig config;
    try{
      
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                      new PIDConstants(5, 0.0, 0.0) // Rotation PID constants
              ),
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  // Update all sensors on swerve modules including shifter sensors
  // Each module returns a ModuleState with current speed and position
  // Logs information to SmartDashboard
  public void updateSensors() {
    moduleStates = new SwerveModuleState[] {
        module0.updateSensors(),
        module1.updateSensors(),
        module2.updateSensors(),
        module3.updateSensors()
    };

    odometry.update(pigeon.getRotation2d(), 
    new SwerveModulePosition[] {
      module0.getPosition(),
      module1.getPosition(),
      module2.getPosition(),
      module3.getPosition()
  });

    double[] loggingState = new double[] {
        moduleStates[1].angle.getRadians(),
        moduleStates[1].speedMetersPerSecond,
        moduleStates[0].angle.getRadians(),
        moduleStates[0].speedMetersPerSecond,
        moduleStates[2].angle.getRadians(),
        moduleStates[2].speedMetersPerSecond,
        moduleStates[3].angle.getRadians(),
        moduleStates[3].speedMetersPerSecond
    };

    SmartDashboard.putNumberArray("realModuleStates", loggingState);
  }

  /**
   * gets the current pose from the swerve odometry.
   * Used for pathplanner
   * 
   * @return Pose2d
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   * Used for autoBuilder (pathplanner)
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        pigeon.getRotation2d(),
        new SwerveModulePosition[] {
          module0.getPosition(),
          module1.getPosition(),
          module2.getPosition(),
          module3.getPosition()
        },
        pose);
  }

  /**
   * Gives current robot chassis speed.
   * Used for autoBuilder (pathplanner)
   * @return current chassis speed
   */
  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(moduleStates);
  }

  /**
   * Adjusts joystick outputs based on driver profile, acceleration limits,
   * assisted and locked headings, and driving orientation
   * and stores them in the modulestates[] object
   * 
   * @param driverController
   * @param isAutonomous
   * @param period_ms           How long it has been since the last loop cycle
   */
  public void driveTeleop(XboxController driverController, boolean isAutonomous, double period_ms) {

    // Adjust strafe outputs
    double[] strafeOutputs = SwerveUtils.swerveScaleStrafe(driverController, isAutonomous);
    double limitedStrafeX = Math.signum(strafeOutputs[0]) * xAccelerationLimiter.calculate(Math.abs(strafeOutputs[0]));
    double limitedStrafeY = Math.signum(strafeOutputs[1]) * yAccelerationLimiter.calculate(Math.abs(strafeOutputs[1]));

    double[] limitedStrafe = new double[] { limitedStrafeX, limitedStrafeY };

    double[] assistedStrafe = SwerveUtils.assistStrafe(limitedStrafe, new double[] { Double.NaN, Double.NaN },
        strafePID);
    double[] orientedStrafe = SwerveUtils.fieldOrientedTransform(assistedStrafe, (-pigeon.getYaw().getValueAsDouble()) % 360);

    // Adjust rotate outputs
    double rotateOutput = SwerveUtils.swerveScaleRotate(driverController, isAutonomous);

    double assistedRotation = swerveAssistHeading(modeDrivebase(driverController), rotateOutput, limitedStrafe,
        isAutonomous, driverController);

    

    // Store information in modulestates
    System.out.println(orientedStrafe[0]);
    moduleStateOutputs = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
            orientedStrafe[0] * maxGroundSpeed_mPs, orientedStrafe[1] * maxGroundSpeed_mPs, assistedRotation * maxRotateSpeed_radPs,
            pigeon.getRotation2d()), 0.001 * period_ms));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStateOutputs, maxGroundSpeed_mPs);
  }

  /**
   * 
   * @param driverController
   * @return
   */
  private double modeDrivebase(XboxController driverController) {
    if ((masterState0 != Robot.masterState) || (driverController.getYButton())) {
      headingLocked = true;
    } else if (driverController.getXButton()) {
      headingLocked = false;
    }
    switch (Robot.masterState) {

      case STOWED: 
        headingLocked = false;
        masterState0 = MasterStates.STOWED;
        return Double.NaN;
      default:
        return Double.NaN;
    }
  }

  /**
   * Adjusts joystick outputs to help with angle drift and heading locking
   * 
   * @param lockedHeading    If the heading is locked
   * @param joystickRotation
   * @param limitedStrafe
   * @param isAutonomous
   * @param driverController
   * @return Locked or assisted heading output
   */
  private double swerveAssistHeading(double lockedHeading, double joystickRotation, double[] limitedStrafe,
      boolean isAutonomous, XboxController driverController) {
    double pigeonAngle = (-pigeon.getYaw().getValueAsDouble()) % 360;
    if (lockedHeading != lockedHeading) {
      lockHeading0 = false;
      boolean disableHeadingAssist = false;
      boolean lowSpeed = !highSpeedSticky.isStuck(swerveGroundSpeed > 1, 250);
      disableHeadingAssist |= lowSpeed; // Driving at low speed
      disableHeadingAssist |= (Math.max(Math.abs(limitedStrafe[0]), Math.abs(limitedStrafe[1]))) <= 0.001; // Driver not
                                                                                                           // driving
      disableHeadingAssist |= Math.abs(joystickRotation) >= 0.001; // Driver trying to rotate
      disableHeadingAssist |= DriverStation.isDisabled(); // Robot disable
      disableHeadingAssist |= isAutonomous; // Robot autonomous
      headingAssist = !disableHeadingAssist;

      if (!headingAssist) {
        return joystickRotation;
      }

      boolean headingLatchSignal = (noRotationSticky.isStuck(Math.abs(joystickRotation) <= 0.001, 100.0)
          && !(DriverStation.isDisabled() || isAutonomous || lowSpeed));

      antiDriftPID.setP(swerveGroundSpeed * antiDriftPID.getP()); // increase kp base on ground velocity
      double assistedRotation = antiDriftPID.calculate(
          pigeonAngle,
          headingLatch.updateLatch(pigeonAngle, pigeonAngle,
              (!headingLatchSignal0) && headingLatchSignal,
              !headingLatchSignal));
      headingLatchSignal0 = headingLatchSignal;
      return assistedRotation;
    } else {
      headingAssist = true;
      double headingFudgeDeltaT = System.currentTimeMillis() - headingFudgeTime;
      headingFudgeTime = System.currentTimeMillis();
      String currentProfile = Robot.legalDrivers[(int) Dashboard.selectedDriver.get(0.0)];
      double driverHeadingFudge;
      if (driverController.getBButton() || driverController.getYButton() || lockHeading0 == false) {
        driverHeadingFudge = 0.0;
        driverHeadingFudge0 = 0.0;
      } else {
        driverHeadingFudge = headingFudgeDeltaT * Units.radiansToDegrees(maxRotateSpeed_radPs)
            * SwerveUtils.readDriverProfiles(currentProfile).rotateMax * joystickRotation;
        driverHeadingFudge0 += driverHeadingFudge;
      }
      driverHeadingFudge0 = Control.clamp(driverHeadingFudge0, headingFudgeMax, -1 * headingFudgeMax);
      lockHeading0 = true;
      double assistedRotation = headingAnglePID.calculate(pigeonAngle, lockedHeading + driverHeadingFudge0);
      return (Math.abs(assistedRotation) > 0.02) ? assistedRotation : 0.0;
    }
  }

  private boolean[] driveFaults = new boolean[4];
  private boolean[] azimuthFaults = new boolean[4];

  /**
   * 
   * @return A boolean array with the structure:
   *         <ul>
   *         <li>drive faults
   *         <li>azimuth faults
   */
  public boolean[] getFaults() {
    boolean[] faults0 = module0.getSwerveFaults();
    boolean[] faults1 = module1.getSwerveFaults();
    boolean[] faults2 = module2.getSwerveFaults();
    boolean[] faults3 = module3.getSwerveFaults();
    driveFaults = new boolean[] { faults0[0], faults1[0], faults2[0], faults3[0] };
    azimuthFaults = new boolean[] { faults0[1], faults1[1], faults2[1], faults3[1] };
    boolean driveFault = faults0[0] || faults1[0] || faults2[0] || faults3[0];
    boolean azimuthFault = faults0[1] || faults1[1] || faults2[1] || faults3[1];
    return new boolean[] { driveFault, azimuthFault };
  }

  /**
   * Sends stored modulestate outputs to each individual module
   * 
   * @param isAutonomous
   */
  public void updateOutputs(boolean isAutonomous) {
    boolean[] faults = getFaults();
    boolean fLow = faults[0] || faults[1];
    boolean homeWheels = Dashboard.homeWheels.get();
    module0.updateOutputs(moduleStateOutputs[0], isAutonomous, fLow, driveFaults[0] || azimuthFaults[0], homeWheels);
    module1.updateOutputs(moduleStateOutputs[1], isAutonomous, fLow, driveFaults[1] || azimuthFaults[1], homeWheels);
    module2.updateOutputs(moduleStateOutputs[2], isAutonomous, fLow, driveFaults[2] || azimuthFaults[2], homeWheels);
    module3.updateOutputs(moduleStateOutputs[3], isAutonomous, fLow, driveFaults[3] || azimuthFaults[3], homeWheels);

    double[] loggingState = new double[] {
      moduleStateOutputs[1].angle.getRadians(),
      moduleStateOutputs[1].speedMetersPerSecond / maxGroundSpeed_mPs
          * ((module1.shiftedState.equals(ShiftedStates.HIGH)) ? maxGroundSpeed_mPs : maxLowGearSpeed_mPs),
      moduleStateOutputs[0].angle.getRadians(),
      moduleStateOutputs[0].speedMetersPerSecond / maxGroundSpeed_mPs
          * ((module0.shiftedState.equals(ShiftedStates.HIGH)) ? maxGroundSpeed_mPs : maxLowGearSpeed_mPs),
      moduleStateOutputs[2].angle.getRadians(),
      moduleStateOutputs[2].speedMetersPerSecond / maxGroundSpeed_mPs
          * ((module2.shiftedState.equals(ShiftedStates.HIGH)) ? maxGroundSpeed_mPs : maxLowGearSpeed_mPs),
      moduleStateOutputs[3].angle.getRadians(),
      moduleStateOutputs[3].speedMetersPerSecond / maxGroundSpeed_mPs
          * ((module3.shiftedState.equals(ShiftedStates.HIGH)) ? maxGroundSpeed_mPs : maxLowGearSpeed_mPs)
  };

  SmartDashboard.putNumberArray("modulestates", loggingState);

  // Report to the dashboard
  Dashboard.swerve0Details.set(new double[] {
      moduleStates[0].angle.getDegrees() % 360,
      module0.getTemp(),
      moduleStates[0].speedMetersPerSecond,
      (module0.shiftedState.equals(ShiftedStates.HIGH)) ? 1 : 0
  });
  Dashboard.swerve1Details.set(new double[] {
      moduleStates[1].angle.getDegrees() % 360,
      module1.getTemp(),
      moduleStates[1].speedMetersPerSecond,
      (module1.shiftedState.equals(ShiftedStates.HIGH)) ? 1 : 0
  });
  Dashboard.swerve2Details.set(new double[] {
      moduleStates[2].angle.getDegrees() % 360,
      module2.getTemp(),
      moduleStates[2].speedMetersPerSecond,
      (module2.shiftedState.equals(ShiftedStates.HIGH)) ? 1 : 0
  });
  Dashboard.swerve3Details.set(new double[] {
      moduleStates[3].angle.getDegrees() % 360,
      module3.getTemp(),
      moduleStates[3].speedMetersPerSecond,
      (module3.shiftedState.equals(ShiftedStates.HIGH)) ? 1 : 0
  });
  }

  /**
   * set module state outputs to target states that are contained within the given ChassisSpeeds.
   * Used for autoBuilder (pathplanner)
   * 
   * @param robotRelativeSpeeds chassis speeds to set desired to
   */
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    moduleStateOutputs = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(robotRelativeSpeeds, 0.001*Robot.loopTime_ms));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStateOutputs, maxGroundSpeed_mPs);
  }
}
