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

    // Drivebase Subsystem
    public static DoubleArrayPublisher swerve0Details;
    public static DoubleArrayPublisher swerve1Details;
    public static DoubleArrayPublisher swerve2Details;
    public static DoubleArrayPublisher swerve3Details;
    public static DoubleArrayPublisher currentDriverProfileSetpoints;
    public static StringArrayPublisher legalDrivers;
    
    public static DoublePublisher robotX;
    public static DoublePublisher robotY;
    public static DoublePublisher robotLengthBumpers;
    public static DoublePublisher robotWidthBumpers;
    public static DoublePublisher robotHeading;    

    public static DoubleArraySubscriber newDriverProfileSetpoints;
    
    public static DoubleSubscriber selectedDriver;
    public static DoubleSubscriber manualStartX;
    public static DoubleSubscriber manualStartY;
    public static DoubleSubscriber manualStartH;
    public static BooleanSubscriber pushRobotStart;
    public static BooleanSubscriber unlockAzimuth;
    public static BooleanSubscriber homeWheels;
    public static BooleanSubscriber calibrateWheels;
    public static BooleanSubscriber applyProfileSetpoints;

    // Limelight


    // Other Testing
    public static DoubleArrayPublisher pidTuningGoalActual;
    public static StringArrayPublisher legalActuatorNames;
    public static BooleanArrayPublisher confirmedMasterStates;

    public static DoublePublisher pressureTransducer;
    public static DoublePublisher loopTime;
    public static DoublePublisher fieldWidth;
    public static DoublePublisher fieldLength;
    public static StringPublisher robotProfile;
    public static StringPublisher codeVersion;
    public static BooleanPublisher isAutonomous;
    
    public static DoubleSubscriber testActuatorValue;
    public static DoubleSubscriber testActuatorPeriod;
    public static DoubleSubscriber freeTuningVariable;
    public static DoubleSubscriber freeTuningkP;
    public static DoubleSubscriber freeTuningkI;
    public static DoubleSubscriber freeTuningkD;
    public static StringSubscriber testActuatorName;
    public static BooleanSubscriber unlockExtender;

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
        newDriverProfileSetpoints = table.getDoubleArrayTopic("New_Driver_Profile_Setpoints")
                .subscribe(new double[] { 0.08, 1.8, 1, 0.15, 2.5, 1 });
        legalDrivers = table.getStringArrayTopic("Legal_Drivers").publish();

        robotX = table.getDoubleTopic("Robot_X").publish();
        robotY = table.getDoubleTopic("Robot_Y").publish();
        robotHeading = table.getDoubleTopic("Robot_H").publish();
        robotLengthBumpers = table.getDoubleTopic("Robot_Length_Bumpers").publish();
        robotWidthBumpers = table.getDoubleTopic("Robot_Width_Bumpers").publish();

        selectedDriver = table.getDoubleTopic("Selected_Driver").subscribe(0.0);
        manualStartX = table.getDoubleTopic("Manual_Start_X").subscribe(0);
        manualStartY = table.getDoubleTopic("Manual_Start_Y").subscribe(0);
        manualStartH = table.getDoubleTopic("Manual_Start_H").subscribe(0);
        pushRobotStart = table.getBooleanTopic("Push_Robot_Start").subscribe(false);
        unlockAzimuth = table.getBooleanTopic("Unlock_Azimuth").subscribe(false);
        homeWheels = table.getBooleanTopic("Home_Wheels").subscribe(false);
        calibrateWheels = table.getBooleanTopic("Calibrate_Wheels").subscribe(false);
        applyProfileSetpoints = table.getBooleanTopic("Apply_Driver_Profile_Setpoints").subscribe(false);

        
        // Other testing
        pidTuningGoalActual = table.getDoubleArrayTopic("PID_Tuning_GoalActual").publish();
        legalActuatorNames = table.getStringArrayTopic("Legal_Actuator_Names").publish();
        confirmedMasterStates = table.getBooleanArrayTopic("Confirmed_States").publish();

        pressureTransducer = table.getDoubleTopic("Pressure_Transducer").publish();
        loopTime = table.getDoubleTopic("Control_Loop_Time").publish();
        fieldWidth = table.getDoubleTopic("Field_Width").publish();
        fieldLength = table.getDoubleTopic("Field_Length").publish();
        robotProfile = table.getStringTopic("Robot_Profile").publish();
        codeVersion = table.getStringTopic("Robot_Code_Version").publish();
        isAutonomous = table.getBooleanTopic("Robot_is_Autonomous").publish();

        testActuatorValue = table.getDoubleTopic("Test_Actuator_Value").subscribe(0.0);
        testActuatorPeriod = table.getDoubleTopic("Test_Actuator_Period").subscribe(0.0);
        freeTuningVariable = table.getDoubleTopic("Free_Tuning_Variable").subscribe(0);
        freeTuningkP = table.getDoubleTopic("Free_Tuning_PID_P").subscribe(0);
        freeTuningkI = table.getDoubleTopic("Free_Tuning_PID_I").subscribe(0);
        freeTuningkD = table.getDoubleTopic("Free_Tuning_PID_D").subscribe(0);
        testActuatorName = table.getStringTopic("Test_Actuator_Name").subscribe("");
        unlockExtender = table.getBooleanTopic("Super_Test_Mode").subscribe(false);
    }
}
