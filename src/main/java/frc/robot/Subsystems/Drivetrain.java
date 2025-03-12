package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Dashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Robot.MasterStates;
import frc.robot.Utility.ActuatorInterlocks;
import frc.robot.Utility.Control;
import frc.robot.Utility.SwerveUtils;
// import frc.robot.Utility.ClassHelpers.DriverProfile;
import frc.robot.Utility.ClassHelpers.Latch;
// import frc.robot.Utility.ClassHelpers.LimitAcceleration;
import frc.robot.Utility.ClassHelpers.StickyButton;
import frc.robot.Utility.ClassHelpers.Timer;
import edu.wpi.first.math.util.Units;

public class Drivetrain extends SubsystemBase {
  enum ShiftedStates {
      LOW,
      TRANS,
      HIGH
  }

  // Constants
  private final double driveHighGearRatio;
  private final double driveLowGearRatio;
  private final double azimuthGearRatio;
  public static double maxGroundSpeed_mPs;
  private final double maxLowGearSpeed_mPs;
  private final double maxRotateSpeed_radPs;
  public static double actualWheelDiameter_m;
  public static double nominalWheelDiameter_m;
  public static boolean invertDriveDirection = false;
  public static boolean invertAzimuthDirection = false;
  public static boolean azimuthSparkEnabled;
  private double[] driveManualAdjustments = new double[] { 1, 1, 1, 1 };
  private final double reefPoseX_m = 4.49;
  private final double reefPoseY_m = Robot.fieldWidth_m / 2;

  // Modules
  private final SwerveModule module0;
  private final SwerveModule module1;
  private final SwerveModule module2;
  private final SwerveModule module3;
  private final Translation2d module0Location_m = new Translation2d(0.301625, -0.276225);
  private final Translation2d module1Location_m = new Translation2d(0.301625, 0.276225);
  private final Translation2d module2Location_m = new Translation2d(-0.301625, 0.276225);
  private final Translation2d module3Location_m = new Translation2d(-0.301625, -0.276225);
  private final TalonFXConfiguration[] driveConfigs;
  private final TalonFXConfiguration[] azimuthConfigs;
  private SwerveModuleState[] moduleStates = new SwerveModuleState[4];

  // Major objects
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(module0Location_m, module1Location_m,
      module2Location_m, module3Location_m);
  private static final Pigeon2 pigeon = new Pigeon2(6);
  SlewRateLimiter xAccelerationLimiter = new SlewRateLimiter(1000, -1000, 0.0);
  SlewRateLimiter yAccelerationLimiter = new SlewRateLimiter(1000, -1000, 0.0);
  private SwerveModuleState[] moduleStateOutputs = new SwerveModuleState[4];
  public DoubleSolenoid shifter;

  // Used for modeDrivebase to check if master states changed
  public static Robot.MasterStates masterState0 = Robot.masterState;
  public static boolean shiftingEnabled = false;
  private boolean headingLocked = false;
  public static final PIDController strafePID = new PIDController(0.4, 0.001, 0);

  // Variables stored for the assist heading function
  private StickyButton highSpeedSticky = new StickyButton();
  private boolean headingAssist = false;
  private Latch headingLatch = new Latch(0.0);
  private PIDController antiDriftPID = new PIDController(0.0007, 0, 0);
  private PIDController headingAnglePID = new PIDController(1.2, 0.0, 0.06);
  // private double kp0 = 0.0;
  // private double ki0 = 0.0;
  // private double kd0 = 0.0;
  private boolean headingLatchSignal0 = false;
  private double driverHeadingFudge0_rad = 0.0;
  private StickyButton noRotationSticky = new StickyButton();
  private boolean lockHeading0 = false;
  private final double headingFudgeMax_rad = Units.degreesToRadians(5); // degrees

  public static ShiftedStates shiftedState = ShiftedStates.HIGH;

  public static final Trigger antiTipTrigger = new Trigger(() -> isTipping());

  private Timer resetIMUTimer = new Timer();
  private boolean resetIMU0 = false;

  private double currentDriveSpeed_mPs;

  public SwerveDrivePoseEstimator updateOdometry() {
    poseEstimator.update(
        getIMURotation(),
        new SwerveModulePosition[] {
            module0.getPosition(),
            module1.getPosition(),
            module2.getPosition(),
            module3.getPosition()
        });
    return poseEstimator;
  }

  public SwerveDrivePoseEstimator poseEstimator;

  public Drivetrain() {
    // Check for driver profile and set constants
    switch (Robot.robotProfile) {
      case "2025_Robot":
        nominalWheelDiameter_m = Units.inchesToMeters(3);
        actualWheelDiameter_m = Units.inchesToMeters(2.9845);
        maxGroundSpeed_mPs = Units.feetToMeters(18.8 * (actualWheelDiameter_m / nominalWheelDiameter_m));
        maxLowGearSpeed_mPs = Units.feetToMeters(9.2 * (actualWheelDiameter_m / nominalWheelDiameter_m));
        maxRotateSpeed_radPs = maxGroundSpeed_mPs
            / ((Math.hypot(Robot.robotLength_m, Robot.robotWidth_m)));
        driveHighGearRatio = 4.54;
        driveLowGearRatio = 10;
        azimuthGearRatio = 35.45;
        shiftingEnabled = true;
        azimuthSparkEnabled = true;
        invertDriveDirection = true;
        invertAzimuthDirection = false;
        driveConfigs = SwerveUtils.swerveModuleDriveConfigs();
        azimuthConfigs = SwerveUtils.swerveModuleAzimuthConfigs();
        driveManualAdjustments = new double[] { 104.0 / 107.7, 104.0 / 105.0, 104.0 / 106.0, 104.0 / 107.0 };
        break;
      case "COTS_Testbed":
        nominalWheelDiameter_m = Units.inchesToMeters(4);
        actualWheelDiameter_m = Units.inchesToMeters(3.8);
        maxGroundSpeed_mPs = Units.feetToMeters(17.8 * (actualWheelDiameter_m / nominalWheelDiameter_m));
        maxLowGearSpeed_mPs = Units.feetToMeters(17.8 * (actualWheelDiameter_m / nominalWheelDiameter_m));
        maxRotateSpeed_radPs = maxGroundSpeed_mPs
            / ((Math.hypot(Robot.robotLength_m, Robot.robotWidth_m)));
        driveHighGearRatio = 0.0; // Bc no shifting
        driveLowGearRatio = 6.12;
        azimuthGearRatio = 150.0 / 7.0;
        shiftingEnabled = false;
        azimuthSparkEnabled = false;
        invertDriveDirection = false;
        invertAzimuthDirection = false;
        driveConfigs = SwerveUtils.swerveModuleDriveConfigs();
        azimuthConfigs = SwerveUtils.swerveModuleAzimuthConfigs();
        driveManualAdjustments = new double[] { 1, 1, 1, 1 };
        break;
      default:
        nominalWheelDiameter_m = Units.inchesToMeters(3);
        actualWheelDiameter_m = Units.inchesToMeters(2.9845);
        maxGroundSpeed_mPs = Units.feetToMeters(17.8 * (actualWheelDiameter_m / nominalWheelDiameter_m));
        maxLowGearSpeed_mPs = Units.feetToMeters(8.3 * (actualWheelDiameter_m / nominalWheelDiameter_m));
        maxRotateSpeed_radPs = maxGroundSpeed_mPs
            / ((Math.hypot(Robot.robotLength_m, Robot.robotWidth_m)));
        driveHighGearRatio = 6.42;
        driveLowGearRatio = 14.12;
        azimuthGearRatio = 15.6;
        shiftingEnabled = true;
        azimuthSparkEnabled = true;
        invertDriveDirection = true;
        invertAzimuthDirection = false;
        driveConfigs = SwerveUtils.swerveModuleDriveConfigs();
        azimuthConfigs = SwerveUtils.swerveModuleAzimuthConfigs();
        driveManualAdjustments = new double[] { 1, 1, 1, 1 };
    }
        if (shiftingEnabled) {
            shifter = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 1, 0);
            shifter.set(Value.kForward);
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

    poseEstimator = 
      new SwerveDrivePoseEstimator(
        kinematics, 
        getIMURotation(), 
        new SwerveModulePosition[] {
          module0.getPosition(),
          module1.getPosition(),
          module2.getPosition(),
          module3.getPosition()
        }, 
        new Pose2d());

    /*
     * Setting up angle wrapping for control PIDs
     * Makes it so that 2pi = 0 and automatic direct angle calculations
     */
    antiDriftPID.enableContinuousInput(0, 2 * Math.PI);
    headingAnglePID.enableContinuousInput(0, 2 * Math.PI);

    // Set up initial module states for init and constructors
    for (int i = 0; i < 4; i++) {
      moduleStateOutputs[i] = new SwerveModuleState();
    }

    RobotConfig config;
    try {

      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                                    // optionally outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic drive trains
              new PIDConstants(1.5, 0.0, 0), // Translation PID constants
              new PIDConstants(3.0, 0.0, 0.0) // Rotation PID constants
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
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
    LimelightHelpers.setLimelightNTDouble("limelight-left", "Throttle", 100.0);
    LimelightHelpers.setPipelineIndex("limelight-left", 1);
    LimelightHelpers.setLimelightNTDouble("limelight-right", "Throttle", 100.0);
    LimelightHelpers.setPipelineIndex("limelight-right", 1);
  }

  public Command getPathFindingCommand(Pose2d targetPose) {
    PathConstraints constraints = new PathConstraints(maxGroundSpeed_mPs / 4, 3.0, maxRotateSpeed_radPs,
        4 * Math.PI); // The constraints for this path.
    Command pathfinder = AutoBuilder.pathfindToPose(targetPose, constraints, 0);
    return pathfinder;
  }

  private static Rotation2d imuOffset = new Rotation2d();

  public static Rotation2d getIMURotation() {
    Rotation2d imuRelativeRotation = pigeon.getRotation2d();
    return imuRelativeRotation.minus(imuOffset);
  }

  // Update all sensors on swerve modules including shifter sensors
  // Each module returns a ModuleState with current speed and position
  // Logs information to SmartDashboard
  public void updateSensors() {
    if (Robot.isEnabled) {
      LimelightHelpers.setLimelightNTDouble("limelight-left", "Throttle", 0.0);
      LimelightHelpers.setPipelineIndex("limelight-left", 0);
      LimelightHelpers.setLimelightNTDouble("limelight-right", "Throttle", 0.0);
      LimelightHelpers.setPipelineIndex("limelight-right", 0);
    } else {
      LimelightHelpers.setLimelightNTDouble("limelight-left", "Throttle", 100.0);
      LimelightHelpers.setPipelineIndex("limelight-left", 1);
      LimelightHelpers.setLimelightNTDouble("limelight-right", "Throttle", 100.0);
      LimelightHelpers.setPipelineIndex("limelight-right", 1);
    }
    moduleStates = new SwerveModuleState[] {
        module0.updateSensors(Robot.driverController),
        module1.updateSensors(Robot.driverController),
        module2.updateSensors(Robot.driverController),
        module3.updateSensors(Robot.driverController)
    };

    ChassisSpeeds currentSpeeds = kinematics.toChassisSpeeds(moduleStates);
    currentDriveSpeed_mPs = Math
        .sqrt(Math.pow(currentSpeeds.vxMetersPerSecond, 2) + Math.pow(currentSpeeds.vyMetersPerSecond, 2));

    if (Robot.driverController.getPOV() > 225 || Robot.driverController.getPOV() < 135) {
      resetIMUTimer.reset();
      resetIMU0 = false;
      poseEstimator.update(
          getIMURotation(),
          new SwerveModulePosition[] {
              module0.getPosition(),
              module1.getPosition(),
              module2.getPosition(),
              module3.getPosition()
          });
    } else if (resetIMUTimer.getTimeMillis() > 2000) {
      if (!resetIMU0) {
        imuOffset = pigeon.getRotation2d();
        poseEstimator.resetRotation(getIMURotation());
        poseEstimator.update(getIMURotation(), new SwerveModulePosition[] {
          module0.getPosition(),
          module1.getPosition(),
          module2.getPosition(),
          module3.getPosition()
        });
        poseEstimator.resetRotation(getIMURotation());
        poseEstimator.update(getIMURotation(), new SwerveModulePosition[] {
          module0.getPosition(),
          module1.getPosition(),
          module2.getPosition(),
          module3.getPosition()
        });
      }
      resetIMU0 = true;
    }

    if (Dashboard.pushRobotStart.get()) {
      double x = Units.inchesToMeters(Dashboard.manualStartX.get());
      double y = Units.inchesToMeters(Dashboard.manualStartY.get());
      Rotation2d h = new Rotation2d(Units.degreesToRadians(Dashboard.manualStartH.get()));
      imuOffset = pigeon.getRotation2d().minus(h);
      resetPose(new Pose2d(x, y, h));
    }

    LimelightHelpers.SetRobotOrientation("limelight-left", getIMURotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-right", getIMURotation().getDegrees(), 0, 0, 0, 0, 0);

    boolean leftEstimateValid = LimelightHelpers.validPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left"));
    boolean rightEstimateValid = LimelightHelpers.validPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right"));
    

    // if (leftEstimateValid && rightEstimateValid) {
    //   PoseEstimate leftPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
    //   PoseEstimate rightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");

    //   if (leftPose.avgTagDist < rightPose.avgTagDist && leftPose.avgTagDist < 3) {
    //     poseEstimator.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds, VecBuilder.fill(0.5, 0.5, 999999));
    //   } else if (rightPose.avgTagDist < 3) {
    //     poseEstimator.addVisionMeasurement(rightPose.pose, rightPose.timestampSeconds, VecBuilder.fill(0.5, 0.5, 999999));
    //   }
    // } else if (leftEstimateValid) {
    //   PoseEstimate leftPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
    //   if (leftPose.avgTagDist < 3) {
    //     poseEstimator.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds, VecBuilder.fill(0.5, 0.5, 999999));
    //   }
    // } else if (rightEstimateValid) {
    //   PoseEstimate rightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
    //   if (rightPose.avgTagDist < 3) {
    //     poseEstimator.addVisionMeasurement(rightPose.pose, rightPose.timestampSeconds, VecBuilder.fill(0.5, 0.5, 999999));
    //   }
    // }
    PoseEstimate rightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
    PoseEstimate leftPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
      if (disableLeftLimelight) {
        if (rightEstimateValid) {
          poseEstimator.addVisionMeasurement(rightPose.pose, rightPose.timestampSeconds, VecBuilder.fill(0.5, 0.5, 999999));
        }
      } else if (disableRightLimelight) {
        if (leftEstimateValid) {
          poseEstimator.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds, VecBuilder.fill(0.5, 0.5, 999999));
          System.out.println("hello");
        }
      } else {
        if (leftEstimateValid && rightEstimateValid) {
          if (leftPose.avgTagDist < rightPose.avgTagDist) {
            poseEstimator.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds, VecBuilder.fill(0.5, 0.5, 999999));
          } else {
            poseEstimator.addVisionMeasurement(rightPose.pose, rightPose.timestampSeconds, VecBuilder.fill(0.5, 0.5, 999999));
          }
        } else if (leftEstimateValid) {
          poseEstimator.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds, VecBuilder.fill(0.5, 0.5, 999999));
        } else if (rightEstimateValid) {
          poseEstimator.addVisionMeasurement(rightPose.pose, rightPose.timestampSeconds, VecBuilder.fill(0.5, 0.5, 999999));
        }
      }

    Dashboard.robotX.set(Units.metersToInches(poseEstimator.getEstimatedPosition().getX()));
    Dashboard.robotY.set(Units.metersToInches(poseEstimator.getEstimatedPosition().getY()));
    Dashboard.robotHeading.set(poseEstimator.getEstimatedPosition().getRotation().getDegrees());

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
    return poseEstimator.getEstimatedPosition();
  }

  public static boolean isTipping() {
    return (Math.abs(pigeon.getPitch().getValueAsDouble()) > 10) || (Math.abs(pigeon.getRoll().getValueAsDouble()) > 10);
  }

  /**
   * Resets the odometry to the specified pose.
   * Used for autoBuilder (pathplanner)
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    imuOffset = pigeon.getRotation2d();
    imuOffset = imuOffset.minus(pose.getRotation());
    poseEstimator.resetRotation(getIMURotation());
    poseEstimator.resetPose(pose);
    poseEstimator.resetRotation(getIMURotation());
    poseEstimator.resetPose(pose);
  }

  /**
   * Gives current robot chassis speed.
   * Used for autoBuilder (pathplanner)
   * 
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
   * @param isAutonomous
   * @param period_ms        How long it has been since the last loop cycle
   */
  public void driveTeleop(XboxController manipController) {
    boolean isAutonomous = DriverStation.isAutonomous();
    // Adjust strafe outputs
    double[] strafeOutputs = SwerveUtils.swerveScaleStrafe(isAutonomous);
    double limitedStrafeX = Control
        .clamp(Math.signum(strafeOutputs[0]) * xAccelerationLimiter.calculate(Math.abs(strafeOutputs[0])), -1, 1);
    double limitedStrafeY = Control
        .clamp(Math.signum(strafeOutputs[1]) * yAccelerationLimiter.calculate(Math.abs(strafeOutputs[1])), -1, 1);

    double[] limitedStrafe = new double[] { limitedStrafeX, limitedStrafeY };
    // PID Tuning
      // double kp = Dashboard.freeTuningkP.get();
      // double ki = Dashboard.freeTuningkI.get();
      // double kd = Dashboard.freeTuningkD.get();
      // if ((kp0 != kp) || (ki0 != ki) || (kd0 != kd)) {
      // strafePID.setP(kp);
      // strafePID.setI(ki);
      // strafePID.setD(kd);
      // kp0 = kp;
      // ki0 = ki;
      // kd0 = kd;
      // }
      // double[] lockedPosition = new double[] { 1, 1 };
      // Dashboard.pidTuningGoalActual.set(new double[] { lockedPosition[0], poseEstimator.getEstimatedPosition().getX() });
      // System.out.println(lockedPosition[1] + ", " + poseEstimator.getEstimatedPosition().getY());

    double lockedHeading = modeDrivebase(manipController);

    // Adjust rotate outputs
    double rotateOutput = SwerveUtils.swerveScaleRotate(isAutonomous);
    double assistedRotation = swerveAssistHeading(lockedHeading, rotateOutput, limitedStrafe,
        isAutonomous);

    // Store information in modulestates
    moduleStateOutputs = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
            limitedStrafe[0] * maxGroundSpeed_mPs, limitedStrafe[1] * maxGroundSpeed_mPs,
            assistedRotation * maxRotateSpeed_radPs,
            getIMURotation()), 0.001 * Robot.loopTime_ms));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStateOutputs, maxGroundSpeed_mPs);
  }


  boolean disableRightLimelight = false;
  boolean disableLeftLimelight = false;
  public Command getFinalStrafeCorrectionCommand() {
    Command outputCommand = new FunctionalCommand(
      () -> poseAccuracyFinal = determineGoalPose(),
      () -> {
        if (Robot.masterState.equals(MasterStates.STOW) || Robot.masterState.equals(MasterStates.SCOR)) {
          if (Robot.scoreRight) {disableRightLimelight = true; disableLeftLimelight = false;}
          else {disableLeftLimelight = true; disableRightLimelight = false; System.out.println("HI");}
        } else {disableRightLimelight = false; disableLeftLimelight = false;}
        double[] strafeCorrection = getStrafeCorrection(determineGoalPose());
        moduleStateOutputs = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
            strafeCorrection[0] * maxGroundSpeed_mPs, strafeCorrection[1] * maxGroundSpeed_mPs,
            strafeCorrection[2] * maxRotateSpeed_radPs,
            getIMURotation()), 0.001 * Robot.loopTime_ms));
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStateOutputs, maxGroundSpeed_mPs);
        }, 
      (interrupt) -> {disableRightLimelight = false; disableLeftLimelight = false;}, 
      () -> ((Robot.masterState.equals(MasterStates.FEED) && this.poseAccuracyGetter())||
            Robot.driverController.getXButton() || 
            Robot.driverController.getYButton() || 
            Robot.driverController.getRightTriggerAxis() > 0.7 || 
            ((Robot.masterState.equals(MasterStates.STOW)) && (masterState0 != MasterStates.STOW))), 
      this);
    return outputCommand;
  }

  public Command getAutoStrafeCorrectionCommand(boolean feederMode) {
    Command outputCommand = new FunctionalCommand(
      () -> poseAccuracyFinal = autoDetermineGoalPose(feederMode),
      () -> {
        if (Robot.masterState.equals(MasterStates.STOW) || Robot.masterState.equals(MasterStates.SCOR)) {
          if (Robot.scoreRight) {disableRightLimelight = true; disableLeftLimelight = false;}
          else {disableLeftLimelight = true; disableRightLimelight = false;}
        } else {disableRightLimelight = false; disableLeftLimelight = false;}
        double[] strafeCorrection = getStrafeCorrection(autoDetermineGoalPose(feederMode));
        moduleStateOutputs = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
            strafeCorrection[0] * maxGroundSpeed_mPs, strafeCorrection[1] * maxGroundSpeed_mPs,
            strafeCorrection[2] * maxRotateSpeed_radPs,
            getIMURotation()), 0.001 * Robot.loopTime_ms));
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStateOutputs, maxGroundSpeed_mPs);
        }, 
      (interrupt) -> {disableRightLimelight = false; disableLeftLimelight = false;}, 
      () -> (this.poseAccuracyGetter()), 
      this).withTimeout(2.5);
    return outputCommand;
  }

  Pose2d poseAccuracyFinal = new Pose2d();
  double poseAccuracyAllowedError = 1.5*0.0254; // Meters
  double rotateAccuracyAllowedError = 2; // degree

  public boolean poseAccuracyGetter() {
    Pose2d pose = poseEstimator.getEstimatedPosition();
    boolean xValid = (Math.abs(pose.getX() - poseAccuracyFinal.getX())) < poseAccuracyAllowedError;
    boolean yValid = (Math.abs(pose.getY() - poseAccuracyFinal.getY())) < poseAccuracyAllowedError;
    boolean rotateValid = (Math.abs(((pose.getRotation().getDegrees()+360)%360) - ((poseAccuracyFinal.getRotation().getDegrees()+360) % 360))) < rotateAccuracyAllowedError;
    return xValid && yValid && rotateValid;
  }

  public double[] getStrafeCorrection(Pose2d finalPose) {
    Pose2d robotPose = poseEstimator.getEstimatedPosition();
    double robotXPose = robotPose.getX();
    double robotYPose = robotPose.getY();
    double finalXPose = finalPose.getX();
    double finalYPose = finalPose.getY();
    PIDController strafePID = Drivetrain.strafePID;
    // Assist in both x and y
    double assistedX;
    double assistedY;
    double distance = Math.hypot(finalXPose - robotXPose, finalYPose - robotYPose);
    // PID Tuning
      // double kp = Dashboard.freeTuningkP.get();
      // double ki = Dashboard.freeTuningkI.get();
      // double kd = Dashboard.freeTuningkD.get();
      // if ((kp0 != kp) || (ki0 != ki) || (kd0 != kd)) {
      // strafePID.setP(kp);
      // strafePID.setI(ki);
      // strafePID.setD(kd);
      // kp0 = kp;
      // ki0 = ki;
      // kd0 = kd;
      // }
      // Dashboard.pidTuningGoalActual.set(new double[] { Units.metersToInches(finalXPose), Units.metersToInches(robotXPose) });
      // System.out.println(Units.metersToInches(finalYPose) + ", " + Units.metersToInches(robotYPose));
    double output = -strafePID.calculate(distance);
    Rotation2d angle = new Rotation2d(finalXPose - robotXPose, finalYPose - robotYPose);
    assistedX = Math.cos(angle.getRadians()) * output;
    assistedX = (Math.abs(assistedX) > 0.01) ? assistedX : 0.0;
    assistedY = Math.sin(angle.getRadians()) * output;
    assistedY = (Math.abs(assistedY) > 0.01) ? assistedY : 0.0;

    double assistedRotation = headingAnglePID.calculate(
      getIMURotation().getRadians(), finalPose.getRotation().getRadians());
    assistedRotation = (Math.abs(assistedRotation) > 0.01) ? assistedRotation : 0.0;
    return new double[] {assistedX, assistedY, assistedRotation};
  }


  public boolean driveToPose = false;

  /**
   * 
   * @return
   */
  private double modeDrivebase(XboxController manipController) {
    if (((masterState0 != Robot.masterState) && (Robot.masterState != MasterStates.STOW)) || (Robot.driverController.getYButton()) || (Robot.driverController.getBButton())) {
      headingLocked = true;
    } else if (Robot.driverController.getXButton()) {
      headingLocked = false;
    }

    Pose2d pose_m = poseEstimator.getEstimatedPosition();
    double poseX_m = pose_m.getX();
    double poseY_m = pose_m.getY();

    double lockedHeading_rad;

    switch (Robot.masterState) {
      case STOW:
        masterState0 = MasterStates.STOW;
        if (headingLocked) {
          // if (Math.hypot(reefPoseX_m-poseX_m, reefPoseY_m-poseY_m) < 2.75) {
          //   if (poseX_m < ((-Math.sqrt(3)) * Math.abs(poseY_m - reefPoseY_m) + reefPoseX_m)) {
          //     // Bottom sextant
          //     lockedHeading_rad = 0.0;
          //   } else if (poseX_m < a) {
          //     if (poseY_m < b) {
          //       // Bottom right sextant
          //         lockedHeading_rad = Units.degreesToRadians(60.0);
          //     } else {
          //       // Bottom left sextant
          //         lockedHeading_rad = Units.degreesToRadians(-60.0);
          //     }
          //   } else if (poseX_m > ((Math.sqrt(3)) * Math.abs(poseY_m - reefPoseY_m) + reefPoseX_m)) {
          //     // Top sextant
          //     lockedHeading_rad = Units.degreesToRadians(180.0);
          //   } else if (poseY_m < b) {
          //     // Top right sextant
          //     lockedHeading_rad = Units.degreesToRadians(120);
          //   } else {
          //     // Top left sextant
          //     lockedHeading_rad = Units.degreesToRadians(-120);
          //   }
          // } else {
            lockedHeading_rad = Math.atan2(reefPoseY_m-poseY_m, reefPoseX_m-poseX_m);
          // }
        } else {
        lockedHeading_rad = Double.NaN;
        }
        break;
      case FEED:
        masterState0 = MasterStates.FEED;
        if (headingLocked) {
          if (Robot.scoreCoral) {
            if (poseY_m < Robot.fieldWidth_m / 2) {
              lockedHeading_rad = Units.degreesToRadians(54);
            } else {
              lockedHeading_rad = Units.degreesToRadians(-54);
            }
          } else {
            if (poseX_m < ((-Math.sqrt(3)) * Math.abs(poseY_m - reefPoseY_m) + reefPoseX_m)) {
                  // Bottom sextant
                  lockedHeading_rad = 0.0;
                } else if (poseX_m < reefPoseX_m) {
                  if (poseY_m < reefPoseY_m) {
                    // Bottom right sextant
                      lockedHeading_rad = Units.degreesToRadians(60.0);
                  } else {
                    // Bottom left sextant
                      lockedHeading_rad = Units.degreesToRadians(-60.0);
                  }
                } else if (poseX_m > ((Math.sqrt(3)) * Math.abs(poseY_m - reefPoseY_m) + reefPoseX_m)) {
                  // Top sextant
                  lockedHeading_rad = Units.degreesToRadians(180.0);
                } else if (poseY_m < reefPoseY_m) {
                  // Top right sextant
                  lockedHeading_rad = Units.degreesToRadians(120);
                } else {
                  // Top left sextant
                  lockedHeading_rad = Units.degreesToRadians(-120);
                }
          }
        } else {
          lockedHeading_rad = Double.NaN;
        }
        break;
      case SCOR:
      masterState0 = MasterStates.SCOR;
        if (headingLocked) {
          if (poseX_m < ((-Math.sqrt(3)) * Math.abs(poseY_m - reefPoseY_m) + reefPoseX_m)) {
                // Bottom sextant
                lockedHeading_rad = 0.0;
              } else if (poseX_m < reefPoseX_m) {
                if (poseY_m < reefPoseY_m) {
                  // Bottom right sextant
                    lockedHeading_rad = Units.degreesToRadians(60.0);
                } else {
                  // Bottom left sextant
                    lockedHeading_rad = Units.degreesToRadians(-60.0);
                }
              } else if (poseX_m > ((Math.sqrt(3)) * Math.abs(poseY_m - reefPoseY_m) + reefPoseX_m)) {
                // Top sextant
                lockedHeading_rad = Units.degreesToRadians(180.0);
              } else if (poseY_m < reefPoseY_m) {
                // Top right sextant
                lockedHeading_rad = Units.degreesToRadians(120);
              } else {
                // Top left sextant
                lockedHeading_rad = Units.degreesToRadians(-120);
              }
        } else {
          lockedHeading_rad = Double.NaN;
        }
      default:
      lockedHeading_rad = Double.NaN;
    }
    return lockedHeading_rad;

  }

  double maxStrafeFudge = Units.inchesToMeters(2);
  public Pose2d autoDetermineGoalPose(boolean feederMode) {
    double reefApothem_m = Units.inchesToMeters(32.75);
    double coralLocalYOffset_m = ((Robot.scoreRight) ? -1 : 1) * Units.inchesToMeters(12.94 / 2);
    double robotSizeX_m = Robot.robotLengthBumpers_m / 2.0 + (0.0254 * 2);

    if (!Robot.scoreCoral) {
      coralLocalYOffset_m = 0.0;
    }

    Pose2d pose_m = poseEstimator.getEstimatedPosition();
    double poseX_m = pose_m.getX();
    double poseY_m = pose_m.getY();

    double lockedX_m;
    double lockedY_m;

    double theta;
    lockedX_m = reefPoseX_m;
    lockedY_m = reefPoseY_m;
    if (feederMode) {
      if (poseY_m < reefPoseY_m) {
        lockedX_m = Units.inchesToMeters(22.51);
        lockedY_m = Units.inchesToMeters(14.8);
        theta = Units.degreesToRadians(54);
        lockedX_m += (robotSizeX_m - (0.0254 * -6)) * Math.sin(theta);
        lockedY_m += (robotSizeX_m - (0.0254 * -6)) * Math.cos(theta);
        lockedX_m += maxStrafeFudge * (-Robot.driverController.getLeftX() * Math.sin(theta));
        lockedY_m -= maxStrafeFudge * (-Robot.driverController.getLeftX() * Math.cos(theta));
        // System.out.println("Locked X: " + lockedX_m + ", Locked Y:" + lockedY_m + ", Rotation:" + (Math.PI + theta));
        return new Pose2d(lockedX_m, lockedY_m, new Rotation2d(theta));
      } else {
        lockedX_m = Units.inchesToMeters(22.51);
        lockedY_m = Units.inchesToMeters(294.2);
        theta = Units.degreesToRadians(306);
        lockedX_m += (robotSizeX_m - (0.0254 * -6)) * Math.cos(theta);
        lockedY_m += (robotSizeX_m - (0.0254 * -6)) * Math.sin(theta);
        lockedX_m += maxStrafeFudge * (-Robot.driverController.getLeftX() * Math.sin(theta));
        lockedY_m -= maxStrafeFudge * (-Robot.driverController.getLeftX() * Math.cos(theta));
        // System.out.println("Locked X: " + lockedX_m + ", Locked Y:" + lockedY_m + ", Rotation:" + (Math.PI + theta));
        return new Pose2d(lockedX_m, lockedY_m, new Rotation2d(theta));
      }
    } else {
      if (poseX_m < ((-Math.sqrt(3)) * Math.abs(poseY_m - reefPoseY_m) + reefPoseX_m)) {
        // Bottom sextant
        theta = 180;
        // lockedX_m = 3.13;
        // lockedY_m = 4.02;
      } else if (poseX_m < reefPoseX_m) {
        if (poseY_m < reefPoseY_m) {
          // Bottom right sextant
          theta = 240;
          // lockedX_m = 3.788;
          // lockedY_m = 2.82;
        } else {
          // Bottom left sextant
          theta = 120;
          // lockedX_m = 3.788;
          // lockedY_m = 5.218;
        }
      } else if (poseX_m > ((Math.sqrt(3)) * Math.abs(poseY_m - reefPoseY_m) + reefPoseX_m)) {
        // Top sextant
        theta = 0;
        // lockedX_m = 5.862;
        // lockedY_m = 4.02;
      } else if (poseY_m < reefPoseY_m) {
        // Top right sextant
        theta = 300;
        // lockedX_m = 5.191;
        // lockedY_m = 2.832;
      } else {
        // Top left sextant
        theta = 60;
        // lockedX_m = 5.203;
        // lockedY_m = 5.206;
      }
      theta = Units.degreesToRadians(theta);
      lockedX_m += (reefApothem_m + robotSizeX_m + (0.5*0.0254))*Math.cos(theta); // X Position of the center of the face
      lockedX_m += coralLocalYOffset_m * Math.sin(theta); // X Position of the scoring location
      lockedX_m += maxStrafeFudge * (-Robot.driverController.getLeftX() * Math.sin(theta));
      lockedY_m += (reefApothem_m + robotSizeX_m + (0.5*0.0254))*Math.sin(theta); // Y position of the center of the face
      lockedY_m -= coralLocalYOffset_m * Math.cos(theta); // Y position of the scoring location
      lockedY_m -= maxStrafeFudge * (-Robot.driverController.getLeftX() * Math.cos(theta));
      // System.out.println("Locked X: " + lockedX_m + ", Locked Y:" + lockedY_m + ", Rotation:" + (Math.PI + theta));
      return new Pose2d(lockedX_m, lockedY_m, new Rotation2d(Math.PI + theta));
    }
  }
  public Pose2d determineGoalPose() {
    double reefApothem_m = Units.inchesToMeters(32.75);
    double coralLocalYOffset_m = ((Robot.scoreRight) ? -1 : 1) * Units.inchesToMeters(12.94 / 2);
    double robotSizeX_m = Robot.robotLengthBumpers_m / 2.0 + (0.0254 * 2);

    if (!Robot.scoreCoral) {
      coralLocalYOffset_m = 0.0;
    }

    Pose2d pose_m = poseEstimator.getEstimatedPosition();
    double poseX_m = pose_m.getX();
    double poseY_m = pose_m.getY();

    double lockedX_m;
    double lockedY_m;

    double theta;
    lockedX_m = reefPoseX_m;
    lockedY_m = reefPoseY_m;
    if (Robot.masterState.equals(Robot.MasterStates.FEED) && Arm.pickupHeight == 1 || (Robot.isAutonomous && Arm.wristRotation.getDegrees() < -110)) {
      if (poseY_m < reefPoseY_m) {
        lockedX_m = Units.inchesToMeters(22.51);
        lockedY_m = Units.inchesToMeters(14.8);
        theta = Units.degreesToRadians(54);
        lockedX_m += (robotSizeX_m - (0.0254 * -6)) * Math.sin(theta);
        lockedY_m += (robotSizeX_m - (0.0254 * -6)) * Math.cos(theta);
        lockedX_m += maxStrafeFudge * (-Robot.driverController.getLeftX() * Math.sin(theta));
        lockedY_m -= maxStrafeFudge * (-Robot.driverController.getLeftX() * Math.cos(theta));
        // System.out.println("Locked X: " + lockedX_m + ", Locked Y:" + lockedY_m + ", Rotation:" + (Math.PI + theta));
        return new Pose2d(lockedX_m, lockedY_m, new Rotation2d(theta));
      } else {
        lockedX_m = Units.inchesToMeters(30.51);
        lockedY_m = Units.inchesToMeters(288.2);
        theta = Units.degreesToRadians(306);
        lockedX_m += (robotSizeX_m - (0.0254 * -6)) * Math.cos(theta);
        lockedY_m += (robotSizeX_m - (0.0254 * -6)) * Math.sin(theta);
        lockedX_m += maxStrafeFudge * (-Robot.driverController.getLeftX() * Math.sin(theta));
        lockedY_m -= maxStrafeFudge * (-Robot.driverController.getLeftX() * Math.cos(theta));
        // System.out.println("Locked X: " + lockedX_m + ", Locked Y:" + lockedY_m + ", Rotation:" + (Math.PI + theta));
        return new Pose2d(lockedX_m, lockedY_m, new Rotation2d(theta));
      }
    } else {
      if (poseX_m < ((-Math.sqrt(3)) * Math.abs(poseY_m - reefPoseY_m) + reefPoseX_m)) {
        // Bottom sextant
        theta = 180;
        // lockedX_m = 3.13;
        // lockedY_m = 4.02;
      } else if (poseX_m < reefPoseX_m) {
        if (poseY_m < reefPoseY_m) {
          // Bottom right sextant
          theta = 240;
          // lockedX_m = 3.788;
          // lockedY_m = 2.82;
        } else {
          // Bottom left sextant
          theta = 120;
          // lockedX_m = 3.788;
          // lockedY_m = 5.218;
        }
      } else if (poseX_m > ((Math.sqrt(3)) * Math.abs(poseY_m - reefPoseY_m) + reefPoseX_m)) {
        // Top sextant
        theta = 0;
        // lockedX_m = 5.862;
        // lockedY_m = 4.02;
      } else if (poseY_m < reefPoseY_m) {
        // Top right sextant
        theta = 300;
        // lockedX_m = 5.191;
        // lockedY_m = 2.832;
      } else {
        // Top left sextant
        theta = 60;
        // lockedX_m = 5.203;
        // lockedY_m = 5.206;
      }
      theta = Units.degreesToRadians(theta);
      lockedX_m += (reefApothem_m + robotSizeX_m + (0.5*0.0254))*Math.cos(theta); // X Position of the center of the face
      lockedX_m += coralLocalYOffset_m * Math.sin(theta); // X Position of the scoring location
      lockedX_m += maxStrafeFudge * (-Robot.driverController.getLeftX() * Math.sin(theta));
      lockedY_m += (reefApothem_m + robotSizeX_m + (0.5*0.0254))*Math.sin(theta); // Y position of the center of the face
      lockedY_m -= coralLocalYOffset_m * Math.cos(theta); // Y position of the scoring location
      lockedY_m -= maxStrafeFudge * (-Robot.driverController.getLeftX() * Math.cos(theta));
      // System.out.println("Locked X: " + lockedX_m + ", Locked Y:" + lockedY_m + ", Rotation:" + (Math.PI + theta));
      return new Pose2d(lockedX_m, lockedY_m, new Rotation2d(Math.PI + theta));
    }
  }

  /**
   * Adjusts joystick outputs to help with angle drift and heading locking
   * 
   * @param lockedHeading_rad Angle to rotate to in radians (NaN if disabled)
   * @param joystickRotation
   * @param limitedStrafe
   * @param isAutonomous
   * @return Locked or assisted heading output
   */
  private double swerveAssistHeading(double lockedHeading_rad, double joystickRotation, double[] limitedStrafe,
      boolean isAutonomous) {
    Rotation2d pigeonAngle = getIMURotation();
    if (lockedHeading_rad != lockedHeading_rad) {
      lockHeading0 = false;
      boolean disableHeadingAssist = false;
      boolean lowSpeed = !highSpeedSticky.isStuck(currentDriveSpeed_mPs > 1, 250);
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

      PIDController antiDriftPID2 = new PIDController(currentDriveSpeed_mPs * antiDriftPID.getP(), antiDriftPID.getI(),
          antiDriftPID.getD());
      double assistedRotation = antiDriftPID2.calculate(
          pigeonAngle.getRadians(),
          headingLatch.updateLatch(pigeonAngle.getRadians(), pigeonAngle.getRadians(),
              (!headingLatchSignal0) && headingLatchSignal,
              !headingLatchSignal));
      headingLatchSignal0 = headingLatchSignal;
      antiDriftPID2.close();
      return assistedRotation;
    } else {
      headingAssist = true;
      String currentProfile = Robot.legalDrivers[(int) Dashboard.selectedDriver.get(0.0)];
      double driverHeadingFudge_rad;
      if (Robot.driverController.getBButton() || Robot.driverController.getYButton() || lockHeading0 == false) {
        driverHeadingFudge_rad = 0.0;
        driverHeadingFudge0_rad = 0.0;
      } else {
        driverHeadingFudge_rad = Robot.loopTime_ms * 0.001 * (maxRotateSpeed_radPs)
            * SwerveUtils.readDriverProfiles(currentProfile).rotateMax * joystickRotation;
        driverHeadingFudge0_rad += driverHeadingFudge_rad;
      }

      driverHeadingFudge0_rad = MathUtil.clamp(driverHeadingFudge0_rad, -1 * headingFudgeMax_rad, headingFudgeMax_rad);
      lockHeading0 = true;

      // PID Tuning
      // double kp = Dashboard.freeTuningkP.get();
      // double ki = Dashboard.freeTuningkI.get();
      // double kd = Dashboard.freeTuningkD.get();
      // if ((kp0 != kp) || (ki0 != ki) || (kd0 != kd)) {
      // headingAnglePID.setP(kp);
      // headingAnglePID.setI(ki);
      // headingAnglePID.setD(kd);
      // kp0 = kp;
      // ki0 = ki;
      // kd0 = kd;
      // }

      double assistedRotation = headingAnglePID.calculate(pigeonAngle.getRadians(),
          lockedHeading_rad + (driverHeadingFudge0_rad));
      Dashboard.pidTuningGoalActual.set(new double[] { Units.radiansToDegrees(lockedHeading_rad), pigeonAngle.getDegrees() });
      return (Math.abs(assistedRotation) > 0.01) ? assistedRotation : 0.0;
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
    boolean fLow = Robot.driverController.getLeftStickButton() && !(faults[0] || faults[1]);
    boolean homeWheels = Dashboard.homeWheels.get();

    moduleStateOutputs[0] = new SwerveModuleState(
        driveManualAdjustments[0] * moduleStateOutputs[0].speedMetersPerSecond,
        moduleStateOutputs[0].angle);
    moduleStateOutputs[1] = new SwerveModuleState(
        driveManualAdjustments[1] * moduleStateOutputs[1].speedMetersPerSecond,
        moduleStateOutputs[1].angle);
    moduleStateOutputs[2] = new SwerveModuleState(
        driveManualAdjustments[2] * moduleStateOutputs[2].speedMetersPerSecond,
        moduleStateOutputs[2].angle);
    moduleStateOutputs[3] = new SwerveModuleState(
        driveManualAdjustments[3] * moduleStateOutputs[3].speedMetersPerSecond,
        moduleStateOutputs[3].angle);

    module0.updateOutputs(moduleStateOutputs[0], isAutonomous, fLow, driveFaults[0] || azimuthFaults[0], homeWheels);
    module1.updateOutputs(moduleStateOutputs[1], isAutonomous, fLow, driveFaults[1] || azimuthFaults[1], homeWheels);
    module2.updateOutputs(moduleStateOutputs[2], isAutonomous, fLow, driveFaults[2] || azimuthFaults[2], homeWheels);
    module3.updateOutputs(moduleStateOutputs[3], isAutonomous, fLow, driveFaults[3] || azimuthFaults[3], homeWheels);

    shiftedState = (fLow) ? ShiftedStates.LOW:ShiftedStates.HIGH;
    ActuatorInterlocks.testActuatorInterlocks(shifter, "Swerve_Shifter_(b)",
            !fLow);

    double[] loggingState = new double[] {
        moduleStateOutputs[1].angle.getRadians(),
        moduleStateOutputs[1].speedMetersPerSecond / maxGroundSpeed_mPs
            * ((shiftedState.equals(ShiftedStates.HIGH)) ? maxGroundSpeed_mPs : maxLowGearSpeed_mPs),
        moduleStateOutputs[0].angle.getRadians(),
        moduleStateOutputs[0].speedMetersPerSecond / maxGroundSpeed_mPs
            * ((shiftedState.equals(ShiftedStates.HIGH)) ? maxGroundSpeed_mPs : maxLowGearSpeed_mPs),
        moduleStateOutputs[2].angle.getRadians(),
        moduleStateOutputs[2].speedMetersPerSecond / maxGroundSpeed_mPs
            * ((shiftedState.equals(ShiftedStates.HIGH)) ? maxGroundSpeed_mPs : maxLowGearSpeed_mPs),
        moduleStateOutputs[3].angle.getRadians(),
        moduleStateOutputs[3].speedMetersPerSecond / maxGroundSpeed_mPs
            * ((shiftedState.equals(ShiftedStates.HIGH)) ? maxGroundSpeed_mPs : maxLowGearSpeed_mPs)
    };

    SmartDashboard.putNumberArray("modulestates", loggingState);

    // Report to the dashboard
    Dashboard.swerve0Details.set(new double[] {
        moduleStates[0].angle.getDegrees() % 360,
        module0.getTemp(),
        moduleStates[0].speedMetersPerSecond,
        (shiftedState.equals(ShiftedStates.HIGH)) ? 1 : 0
    });
    Dashboard.swerve1Details.set(new double[] {
        moduleStates[1].angle.getDegrees() % 360,
        module1.getTemp(),
        moduleStates[1].speedMetersPerSecond,
        (shiftedState.equals(ShiftedStates.HIGH)) ? 1 : 0
    });
    Dashboard.swerve2Details.set(new double[] {
        moduleStates[2].angle.getDegrees() % 360,
        module2.getTemp(),
        moduleStates[2].speedMetersPerSecond,
        (shiftedState.equals(ShiftedStates.HIGH)) ? 1 : 0
    });
    Dashboard.swerve3Details.set(new double[] {
        moduleStates[3].angle.getDegrees() % 360,
        module3.getTemp(),
        moduleStates[3].speedMetersPerSecond,
        (shiftedState.equals(ShiftedStates.HIGH)) ? 1 : 0
    });
  }

  /**
   * set module state outputs to target states that are contained within the given
   * ChassisSpeeds.
   * Used for autoBuilder (pathplanner)
   * 
   * @param robotRelativeSpeeds chassis speeds to set desired to
   */
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds, DriveFeedforwards feedforwards) {
    robotRelativeSpeeds = new ChassisSpeeds(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond, robotRelativeSpeeds.omegaRadiansPerSecond);
    moduleStateOutputs = kinematics
        .toSwerveModuleStates(ChassisSpeeds.discretize(robotRelativeSpeeds, 0.001 * Robot.loopTime_ms));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStateOutputs, maxGroundSpeed_mPs);
  }
}
