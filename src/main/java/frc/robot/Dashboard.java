package frc.robot;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;

public class Dashboard {
    @SuppressWarnings("unused")
    private NetworkTableInstance inst;
    @SuppressWarnings("unused")
    private NetworkTable table;

    // Array publishers
    public static StringArrayPublisher legalActuatorNames;
    public static StringArrayPublisher legalDrivers;
    public static DoubleArrayPublisher swerve0Details;
    public static DoubleArrayPublisher swerve1Details;
    public static DoubleArrayPublisher swerve2Details;
    public static DoubleArrayPublisher swerve3Details;
    public static DoubleArrayPublisher currentDriverProfileSetpoints;
    public static BooleanArrayPublisher confirmedMasterStates;

    // Publishers
    public static StringPublisher robotProfile;
    public static StringPublisher codeVersion;
    public static DoublePublisher pressureTransducer;

    public static BooleanPublisher isAutonomous;
    public static DoublePublisher loopTime;

    // Array Subscribers
    public static DoubleArraySubscriber newDriverProfileSetpoints;

    // Subscribers
    public static StringSubscriber testActuatorName;
    public static DoubleSubscriber testActuatorValue;
    public static DoubleSubscriber testActuatorPeriod;
    public static DoubleSubscriber selectedDriver;
    public static DoubleSubscriber freeTuningVariable;
    public static DoubleSubscriber freeTuningkP;
    public static DoubleSubscriber freeTuningkI;
    public static DoubleSubscriber freeTuningkD;
    public static BooleanSubscriber unlockAzimuth;
    public static BooleanSubscriber applyProfileSetpoints;
    public static BooleanSubscriber homeWheels;
    public static BooleanSubscriber calibrateWheels;

    /**
     * Creates an object for storing dashboard publishers and subscribers
     */
    public Dashboard() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("SmartDashboard");

        // Drivebase subsystem
        swerve0Details = table.getDoubleArrayTopic("Swerve_0_Details").publish();
        swerve1Details = table.getDoubleArrayTopic("Swerve_1_Details").publish();
        swerve2Details = table.getDoubleArrayTopic("Swerve_2_Details").publish();
        swerve3Details = table.getDoubleArrayTopic("Swerve_3_Details").publish();
        currentDriverProfileSetpoints = table.getDoubleArrayTopic("Current_Driver_Profile_Setpoints").publish();
        legalDrivers = table.getStringArrayTopic("Legal_Drivers").publish();
        newDriverProfileSetpoints = table.getDoubleArrayTopic("New_Driver_Profile_Setpoints")
                .subscribe(new double[] { 0.08, 1.8, 1, 0.15, 2.5, 1 });

        selectedDriver = table.getDoubleTopic("Selected_Driver").subscribe(0.0);
        unlockAzimuth = table.getBooleanTopic("Unlock_Azimuth").subscribe(false);
        homeWheels = table.getBooleanTopic("Home_Wheels").subscribe(false);
        calibrateWheels = table.getBooleanTopic("Calibrate_Wheels").subscribe(false);
        applyProfileSetpoints = table.getBooleanTopic("Apply_Driver_Profile_Setpoints").subscribe(false);

        
        // Other testing
        legalActuatorNames = table.getStringArrayTopic("Legal_Actuator_Names").publish();
        confirmedMasterStates = table.getBooleanArrayTopic("Confirmed_States").publish();

        robotProfile = table.getStringTopic("Robot_Profile").publish();
        codeVersion = table.getStringTopic("Robot_Code_Version").publish();
        pressureTransducer = table.getDoubleTopic("Pressure_Transducer").publish();
        loopTime = table.getDoubleTopic("Control_Loop_Time").publish();
        isAutonomous = table.getBooleanTopic("Robot_is_Autonomous").publish();
        testActuatorName = table.getStringTopic("Test_Actuator_Name").subscribe("");
        testActuatorValue = table.getDoubleTopic("Test_Actuator_Value").subscribe(0.0);
        testActuatorPeriod = table.getDoubleTopic("Test_Actuator_Period").subscribe(0.0);
        freeTuningVariable = table.getDoubleTopic("Free_Tuning_Variable").subscribe(0);
        freeTuningkP = table.getDoubleTopic("Free_Tuning_PID_P").subscribe(0);
        freeTuningkI = table.getDoubleTopic("Free_Tuning_PID_I").subscribe(0);
        freeTuningkD = table.getDoubleTopic("Free_Tuning_PID_D").subscribe(0);
    }
}
