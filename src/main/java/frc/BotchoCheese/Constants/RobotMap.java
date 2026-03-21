package frc.BotchoCheese.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class RobotMap {
    //Limelight 
    // Increase these numbers to trust your model's state estimates less.
    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;

    // Increase these numbers to trust global measurements from vision less.
    public static final double kVisionStdDevX = 1;
    public static final double kVisionStdDevY = 1;
    public static final double kVisionStdDevTheta = 99999;

    public static final String LIMELIGHT_NAME = "limelight";
    public static final String LIMELIGHT_2_NAME = "limelight-two";
    public static final double DIFFERENCE_CUTOFF_THRESHOLD = 0.15; // Normalized from 1.5 for scale consistency
    public static final double[] TAG_HEIGHTS = {
    1.4859, 1.4859, /*ID: 1 - 2*/
    1.30175, /*ID: 3*/
    1.8679160000000001, 1.8679160000000001, /*ID: 4 - 5*/
    0.308102, 0.308102, 0.308102, 0.308102, 0.308102, 0.308102, /*ID: 6 - 11*/
    1.4859, 1.4859, /*ID: 12 - 13*/
    1.8679160000000001, 1.8679160000000001, /*ID: 14 - 15*/
    1.30175, /*ID: 16*/
    0.308102, 0.308102, 0.308102, 0.308102, 0.308102, 0.308102 /*ID: 17 - 22*/};

    public static final AprilTagFieldLayout WELDED_FIELD2026 = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    //Gyro
    public static final int PIGEON_ID = 30;

    //TODO
    // GLOBAL SPEED
    public static final double GLOBAL_SPEED = 0.1;

    // Climber
    // Climber motor IDs
    // public static final int LEFT_CLIMBER_MOTOR_ID = 27;
    // public static final int RIGHT_CLIMBER_MOTOR_ID = 28;

    // // Climber PID Values
    // // TODO
    // public static final double CLIMBER_P_VALUE = 0.3;
    // public static final double CLIMBER_I_VALUE = 0.25;
    // public static final double CLIMBER_D_VALUE = 0.4;

    // // Climber speeds
    // public static final double CLIMBER_UP_SPEED = -0.6;
    // public static final double CLIMBER_DOWN_SPEED = 0.6;

    // // TODO: Comfirm the new Climber Extension Limit
    // public static final double CLIMBER_EXTENSION_LIMIT = 200; 
    
    // Intake motor IDs
    public static final int INTAKE_MOTOR_ID = 22;
    public static final int LEFT_PIVOT_MOTOR_ID = 20;
    public static final int RIGHT_PIVOT_MOTOR_ID = 21;
    // public static final int CAN_RANGE_ID = 23;

    // Pivot motor values
    // TODO Edit values
    public static final double PIVOT_UP_POSITION = 0;
    public static final double PIVOT_DOWN_POSITION = 17.2;

    public static final double PIVOT_CURRENT_STALL_THRESHOLD = 4.0;

    // Intake PID Values
    // TODO
    public static final double INTAKE_P_VALUE = 0.4;
    public static final double INTAKE_I_VALUE = 0.3;
    public static final double INTAKE_D_VALUE = 0.1;

    // Pivot PID Values
    // TODO
    public static final double PIVOT_S_VALUE = 0.25;
    public static final double PIVOT_V_VALUE = 0.12;
    public static final double PIVOT_A_VALUE = 0.01;
    public static final double PIVOT_P_VALUE = 0.4;
    public static final double PIVOT_I_VALUE = 0.3;
    public static final double PIVOT_D_VALUE = 0.1;

    // Intake Motion Magic Values

    public static final double PIVOT_CRUISE_VELOCITY = 10; // Target cruise velocity of 20 rps
    public static final double PIVOT_ACCELERATION = 10; // Target acceleration of 20 rps^2 (1 seconds)
    public static final double PIVOT_JERK = 100; // Target jerk

    // Intake speeds
    // Verify if values are correct and go in the right directions
    public static final double INTAKE_SPEED = -0.83;
    // TODO: Comfirm the threshold.
    public static final double CAN_RANGE_DISTANCE_THRESHOLD = 0.5;
    
    // Feeder Motor ID and Speed
    // TODO PID
    public static final int FEEDER_MOTOR_L_ID = 23;
    public static final int FEEDER_MOTOR_R_ID = 29;
    public static final double FEEDER_P_VALUE = 0.12;
    public static final double FEEDER_I_VALUE = 0.0;
    public static final double FEEDER_D_VALUE = 0.001;
    public static final double FEEDER_SPEED = -1.0; 

    // Motion Magic is currently disabled for the feeder.
    // public static final double FEEDER_CRUISE_VELOCITY = 14;
    // public static final double FEEDER_ACCELERATION = 5;
    // public static final double FEEDER_JERK = 50;

    // Shooter Motor IDs
    public static final int LEFT_SHOOTER_MOTOR_ID = 24;
    public static final int MIDDLE_SHOOTER_MOTOR_ID = 25;
    public static final int RIGHT_SHOOTER_MOTOR_ID = 26;
    
    // // Hood Motor IDs
    // public static final int HOOD_MOTOR_ID = 17;
    // public static final int HOOD_THROUGHBORE_DIO = 0;

    // public static final double HOOD_P_VALUE = 0.4;
    // public static final double HOOD_I_VALUE = 0.0;
    // public static final double HOOD_D_VALUE = 0.2;
    // public static final double HOOD_SPEED = .1;

    // // Hood Motion Magic Values
    
    // public static final double HOOD_CRUISE_VELOCITY = 40;
    
    // public static final double HOOD_ACCELERATION = 80;
    // public static final double HOOD_JERK = 800;

    // //TODO
    // public static double HOOD_SPEED_REGRESSION_SLOPE = 1.0;
    // public static double HOOD_SPEED_REGRESSION_Y_INTERCEPT = 1.0;

    // Shooter speeds

      // Notice: Will be dynamically modified in the Shooter's updateSpeed function
   
    public static final double SHOOTER_MULTIPLIER = 0.5;
    public static double SHOOTER_SPEED_1 = 0.67; // Notice: Will be dynamically modified in the Shooter's updateSpeed function
    public static double SHOOTER_SPEED_2 = 0.77;
    public static double SHOOTER_SPEED_3 = 0.87;
    
    public static final double SHOOTER_ACCELERATION = 160.0 * SHOOTER_MULTIPLIER;
    public static final double SHOOTER_JERK = 1600.0 * SHOOTER_MULTIPLIER;

    // Notice 1: Will be dynamically modified in the Shooter's updateSpeed function
    // Notice 2: This apparently does not control anything
    public static double SHOOTER_SPEED = 0.83;
    
    //TODO
    public static double SHOOTER_SPEED_REGRESSION_SLOPE = 1.0;
    public static double SHOOTER_SPEED_REGRESSION_Y_INTERCEPT = 1.0;

    //If the distance between the robot and tag is less than threshold, the distance is "short"
    public static final double SHORT_DISTANCE_THRESHOLD = 5.0;
    public static final double MEDIUM_DISTANCE_THRESHOLD = 10.0;

    // Shooter PID Values
    // TODO
    public static final double SHOOTER_P_VALUE = 0.5; //Good enough
    public static final double SHOOTER_I_VALUE = 0.0;  //Formerly 0.15
    public static final double SHOOTER_D_VALUE = 0.0; //Formerly 0.2
    public static final double SHOOTER_S_VALUE = 0.2; //Recommended range: 0.2--0.3 (formerly 0.25)
    public static final double SHOOTER_V_VALUE = 0.05; //TODO: Important! Must be measured (formerly 0.12).
    public static final double SHOOTER_A_VALUE = 0.01; //Recommended range: Small (<0.05)

    // Indexer ID
    public static final int INDEXER_MOTOR_ID = 18;

    // Indexer speeds
    public static final double INDEXER_MOTOR_SPEED =.4;

    // CANdle Constants
    public static final int CANDLE_LED_START_INDEX = 0;
    public static final int CANDLE_LED_COUNT = 0;
    public static final int CANDLE_CAN_ID = 40;
    public static final String CANDLE_CAN_BUS = "Canivore1515";
    public static final double CANDLE_BRIGHTNESS = 0.5;
}
