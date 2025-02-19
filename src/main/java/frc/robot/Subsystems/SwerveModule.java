package frc.robot.Subsystems;

import frc.robot.Dashboard;
import frc.robot.Utility.ActuatorInterlocks;
import frc.robot.Utility.SwerveUtils;
import frc.robot.Utility.ClassHelpers.Timer;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class SwerveModule {
    enum ShiftedStates {
        LOW,
        TRANS,
        HIGH
    }

    private final double shiftToHigh_radPs = Units.rotationsPerMinuteToRadiansPerSecond(4000);
    private final double shiftToLow_radPs = Units.rotationsPerMinuteToRadiansPerSecond(500);
    private static double kWheelRadius_m;

    private final double driveHighGearRatio;
    private final double driveLowGearRatio;
    private final double azimuthRatio;
    private final int moduleNumber;

    private final TalonFX drive;
    private final TalonFX azimuthTalon;
    private Slot0Configs azimuthPIDConfigs = new Slot0Configs();
    private SparkMax azimuthSpark;
    private SparkMaxConfig azimuthSparkConfig;
    private RelativeEncoder azimuthEncoder;
    private SparkClosedLoopController azimuthPidController;
    // private SparkLimitSwitch azimuthForwardLimit;
    // private SparkLimitSwitch azimuthReverseLimit;
    public DoubleSolenoid shifter;

    public ShiftedStates shiftedState = ShiftedStates.LOW;
    public boolean shiftingEnabled;

    private boolean azimuthSparkActive;

    // Used for update outputs, 0 indicates prior loop output
    private boolean unlockAzimuth0 = false;
    private boolean shifterOutput0 = false;
    private boolean unlockDrive0 = false;
    private Timer shiftThreshold = new Timer();
    private Timer robotDisabled = new Timer();

    private double currentDriveSpeed_mPs = 0;
    private double azimuthAngle_rad = 0;

    // Prior loop kp, i, and d tracking
    @SuppressWarnings("unused")
    private double kp0 = 0.12;
    @SuppressWarnings("unused")
    private double ki0 = 0;
    @SuppressWarnings("unused")
    private double kd0 = 0;

    /**
     * Creates a new swerve module object
     * 
     * @param driveHighGearRatio The gear ratio between the drive motor and the
     *                           wheel in high gear (USE 0 for a drivetrain without
     *                           shifting)
     * @param driveLowGearRatio  The gear ratio between the drive motor and the
     *                           wheel in low gear (Use the only ratio here for
     *                           drivetrain without shifting)
     * @param azimuthRatio       The gear ratio between the azimuth motor and the
     *                           rotation
     *                           of the wheel
     * @param moduleNumber       The number of the module starting at 0 in the top
     *                           right
     *                           and increasing in a ccw circle
     * @param driveConfig        Configurations for the drive motor, use
     *                           SwerveUtils.swerveModuleDriveConfigs()
     * @param azimuthConfig      Configurations for the azimuth motor, use
     *                           SwerveUtils.swerveModuleAzimuthConfigs()
     * 
     */
    public SwerveModule(double driveHighGearRatio, double driveLowGearRatio, double azimuthRatio, int moduleNumber,
            TalonFXConfiguration driveConfig, TalonFXConfiguration azimuthConfig) {
        shiftingEnabled = Drivetrain.shiftingEnabled;
        if (shiftingEnabled) {
            shifter = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, moduleNumber, 15 - moduleNumber);
            shifter.set(Value.kForward);
        }

        this.driveHighGearRatio = driveHighGearRatio;
        this.driveLowGearRatio = driveLowGearRatio;
        this.azimuthRatio = azimuthRatio;
        this.moduleNumber = moduleNumber;
        kWheelRadius_m = Drivetrain.actualWheelDiameter_m / 2;

        this.drive = new TalonFX(10 + moduleNumber);
        this.azimuthTalon = new TalonFX(20 + moduleNumber);

        azimuthSparkActive = false;

        if (Drivetrain.azimuthSparkEnabled) {
            this.azimuthSpark = new SparkMax(20 + moduleNumber, MotorType.kBrushless);
            azimuthPidController = azimuthSpark.getClosedLoopController();
            azimuthSparkConfig = new SparkMaxConfig();
            azimuthSparkConfig.closedLoop.pid(0.12,0,0);
            azimuthSpark.configure(azimuthSparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            this.azimuthEncoder = azimuthSpark.getEncoder();
            azimuthSparkActive = true;
            if (shiftingEnabled) {
                // this.azimuthForwardLimit = azimuthSpark.getForwardLimitSwitch();
                // this.azimuthReverseLimit = azimuthSpark.getReverseLimitSwitch();
            }
        } else {
            this.azimuthTalon.getConfigurator().apply(azimuthConfig);
            azimuthPIDConfigs.kP = 0.12;
            azimuthPIDConfigs.kI = 0;
            azimuthPIDConfigs.kD = 0;
            this.azimuthTalon.getConfigurator().apply(azimuthPIDConfigs);
            this.azimuthTalon.setPosition(0.0);
        }

        this.drive.setPosition(0.0);
        this.drive.getConfigurator().apply(driveConfig);
    }

    public SwerveModuleState updateSensors(XboxController drivercontroller) {
        if (azimuthSparkActive) {
            azimuthAngle_rad = Units.rotationsToRadians(azimuthEncoder.getPosition() / azimuthRatio);
            if (shiftingEnabled) {
                if (drivercontroller.getLeftStickButton()) {
                    shiftedState = ShiftedStates.LOW;
                } else {
                    shiftedState = ShiftedStates.HIGH;
                }
            }
        } else {
            if (shiftingEnabled) {
                if (drivercontroller.getLeftStickButton()) {
                    shiftedState = ShiftedStates.LOW;
                } else {
                    shiftedState = ShiftedStates.HIGH;
                }
            }
            azimuthAngle_rad = Units
                    .rotationsToRadians(azimuthTalon.getRotorPosition().getValueAsDouble() / azimuthRatio);
        }

        if (!shiftingEnabled) {
            shiftedState = ShiftedStates.LOW;
        }



        currentDriveSpeed_mPs = drive.getVelocity().getValueAsDouble()
                / ((shiftedState.equals(ShiftedStates.HIGH)) ? driveHighGearRatio : driveLowGearRatio) * 2 * Math.PI
                * kWheelRadius_m;

        return new SwerveModuleState(currentDriveSpeed_mPs, new Rotation2d(azimuthAngle_rad));
    }

    /**
     * Returns the position of the drive and azimuth motors
     * 
     * @return A SwerveModulePosition object
     */
    public SwerveModulePosition getPosition() {
        Rotation2d rotation;
        // Decide between using spark and talon
        if (azimuthSparkActive) {
            rotation = new Rotation2d(Units.rotationsToRadians(azimuthEncoder.getPosition()) / azimuthRatio);
        } else {
            rotation = new Rotation2d(
                    (Units.rotationsToRadians(azimuthTalon.getRotorPosition().getValueAsDouble() / azimuthRatio)));
        }
        return new SwerveModulePosition(
                (drive.getRotorPosition().getValueAsDouble()
                        / ((shiftedState.equals(ShiftedStates.HIGH)) ? driveHighGearRatio : driveLowGearRatio))
                        * (2 * Math.PI * kWheelRadius_m),
                rotation);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState();
    }

    /**
     * Gets any faults from the drive and azimuth motors
     * 
     * @return A boolean array with the structure:
     *         <ul>
     *         <li>drive fault,
     *         <li>azimuth fault
     */
    public boolean[] getSwerveFaults() {
        // Decide between using spark and talon
        boolean azimuthFault;
        if (azimuthSparkActive) {
            azimuthFault = (short) 0 != azimuthSpark.getFaults().rawBits;
        } else {
            azimuthFault = 0 != azimuthTalon.getFaultField().getValueAsDouble();
        }
        boolean driveFault = drive.getFaultField().getValueAsDouble() != 0;
        return new boolean[] { driveFault, azimuthFault };
    }

    /**
     * Sends modulestate outputs to the drive, azimuth, and shifter
     * 
     * @param moduleState
     * @param isAutonomous
     */
    public void updateOutputs(SwerveModuleState moduleState, boolean isAutonomous, boolean fLow,
            boolean moduleFailure, boolean homeWheels) {
        // Decide shifter output
        if (shiftingEnabled) {
            if (fLow) {
                shifterOutput0 = false;
            } else {
                // FOR AUTO-SHIFTING ONLY
                if (Dashboard.testActuatorName.get().equals("Drivetrain_(p)")) {
                    if (isAutonomous) {
                        shifterOutput0 = true;
                    } else {
                        if (shifterOutput0) {
                            // Currently commanded to high gear
                            if (Units.rotationsToRadians(
                                    Math.abs(drive.getVelocity().getValueAsDouble())) > shiftToLow_radPs) {
                                shiftThreshold.reset();
                            }
                            shifterOutput0 = shiftThreshold.getTimeMillis() < 150;
                        } else {
                            // Currently commanded to low gear
                            if (Units
                                    .rotationsToRadians(
                                            Math.abs(drive.getVelocity().getValueAsDouble())) < shiftToHigh_radPs) {
                                shiftThreshold.reset();
                            }
                            shifterOutput0 = shiftThreshold.getTimeMillis() > 150;
                        }
                    }
                } else {
                shifterOutput0 = true;
                }
            }
            // Output to shifter
            ActuatorInterlocks.TAI_Solenoids(shifter, "Swerve_" + ((Integer) moduleNumber).toString() + "_Shifter_(b)",
                    shifterOutput0);
        } else {
            shifterOutput0 = false;
        }

        // Output to azimuth
        moduleState.optimize(new Rotation2d(azimuthAngle_rad));

        double normalAzimuthOutput_rad = ((homeWheels) ? 0 : moduleState.angle.getRadians());

        // Wrapping the angle to allow for "continuous input"
        double minDistance = MathUtil.angleModulus(normalAzimuthOutput_rad - azimuthAngle_rad);
        double normalAzimuthOutput_rot = Units.radiansToRotations(azimuthAngle_rad + minDistance) * azimuthRatio;

        if (azimuthSparkActive) {
            ActuatorInterlocks.TAI_SparkMAX_Position(azimuthSpark, azimuthPidController,
                    "Azimuth_" + ((Integer) moduleNumber).toString() + "_(p)",
                    normalAzimuthOutput_rot, 0.0);
            if (Dashboard.calibrateWheels.get()) {
                azimuthEncoder.setPosition(0);
            }
        } else {
            /* PID tuning code START */
            // double kp = Dashboard.freeTuningkP.get();
            // double ki = Dashboard.freeTuningkI.get();
            // double kd = Dashboard.freeTuningkD.get();
            // if ((kp0 != kp) || (ki0 != ki) || (kd0 != kd)) {
            // azimuthPIDConfigs.kP = kp;
            // azimuthPIDConfigs.kI = ki;
            // azimuthPIDConfigs.kD = kd;
            // this.azimuthTalon.getConfigurator().apply(azimuthPIDConfigs);
            // kp0 = kp;
            // ki0 = ki;
            // kd0 = kd;
            // }
            /* PID tuning code END */

            if (Dashboard.calibrateWheels.get()) {
                azimuthTalon.setPosition(0.0);
            }

            ActuatorInterlocks.TAI_TalonFX_Position(azimuthTalon,
                    "Azimuth_" + ((Integer) moduleNumber).toString() + "_(p)",
                    normalAzimuthOutput_rot, 0.0);
        }

        // Decide whether to put azimuth in coast mode
        boolean unlockAzimuth = Dashboard.unlockAzimuth.get();
        if (((unlockAzimuth || moduleFailure) != unlockAzimuth0)) {
            if (unlockAzimuth || moduleFailure) {
                if (azimuthSparkActive) {
                    SparkMaxConfig config = new SparkMaxConfig();
                    config.idleMode(IdleMode.kCoast);
                    azimuthSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                } else {
                    azimuthTalon.setNeutralMode(NeutralModeValue.Coast);
                }
            } else {
                if (azimuthSparkActive) {
                    SparkMaxConfig config = new SparkMaxConfig();
                    config.idleMode(IdleMode.kBrake);
                    azimuthSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                } else {
                    azimuthTalon.setNeutralMode(NeutralModeValue.Brake);
                }
            }
        }
        unlockAzimuth0 = (unlockAzimuth || moduleFailure);

        // Output drive
        double driveOutput = (shiftingEnabled) ? SwerveUtils.driveCommandToPower(moduleState, shifterOutput0)
                : moduleState.speedMetersPerSecond / Drivetrain.maxGroundSpeed_mPs;

        driveOutput *= moduleState.angle.minus(new Rotation2d(azimuthAngle_rad)).getCos();

        ActuatorInterlocks.TAI_TalonFX_Power(drive, "Drive_" + ((Integer) moduleNumber).toString() + "_(p)",
        driveOutput);

        // unlock drive motor if robot is disabled for more than 7 seconds or module
        // fails
        if (DriverStation.isEnabled()) {
            robotDisabled.reset();
        }

        boolean unlockDrive = (robotDisabled.getTimeMillis() > 7000) || moduleFailure;

        if (unlockDrive != unlockDrive0) {
            if (unlockDrive) {
            drive.setNeutralMode(NeutralModeValue.Coast);
            } else {
                drive.setNeutralMode(NeutralModeValue.Brake);
            }
        }
        unlockDrive0 = unlockDrive;
    }

    /**
     * Temperature of drive motor
     * 
     * <ul>
     * <li><b>Minimum Value:</b> 0.0
     * <li><b>Maximum Value:</b> 255.0
     * <li><b>Default Value:</b> 0
     * <li><b>Units:</b> ℃
     * </ul>
     * 
     * @return Double temperature in degrees Celcius
     */
    public double getTemp() {
        return drive.getDeviceTemp().getValueAsDouble();
    }
}