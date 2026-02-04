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
    public static final double DIFFERENCE_CUTOFF_THRESHOLD = 1.5; // Max difference between vision and odometry pose
    public static final double[] TAG_HEIGHTS = {
    1.4859, 1.4859, /*ID: 1 - 2*/
    1.30175, /*ID: 3*/
    1.8679160000000001, 1.8679160000000001, /*ID: 4 - 5*/
    0.308102, 0.308102, 0.308102, 0.308102, 0.308102, 0.308102, /*ID: 6 - 11*/
    1.4859, 1.4859, /*ID: 12 - 13*/
    1.8679160000000001, 1.8679160000000001, /*ID: 14 - 15*/
    1.30175, /*ID: 16*/
    0.308102, 0.308102, 0.308102, 0.308102, 0.308102, 0.308102 /*ID: 17 - 22*/};

    public static final AprilTagFieldLayout ANDYMARK_FIELD2025 = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public static final AprilTagFieldLayout WELDED_FIELD2025 = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    //Gyro
    public static final int PIGEON_ID = 30;

    // Climber
    // Climber motor IDs (fill with actual CAN IDs for your robot hardware)
    public static final int LEFT_CLIMBER_MOTOR_ID = 10;
    public static final int RIGHT_CLIMBER_MOTOR_ID = 11;
    public static final int LEFT_CLIMBER_TWO_MOTOR_ID = 12;
    public static final int RIGHT_CLIMBER_TWO_MOTOR_ID = 13;

    // Climber PID Values
    public static final double CLIMBER_P_VALUE = 0.3;
    public static final double CLIMBER_I_VALUE = 0.25;
    public static final double CLIMBER_D_VALUE = 0.4;

    // Climber speeds
    // TODO: Check whether new motors (l2Climber and r2Climber) speeds are required in some way
    public static final double TEST_MOTOR_LEFT_UP_SPEED = 0.2;
    public static final double TEST_MOTOR_LEFT_DOWN_SPEED = -0.2;
    public static final double TEST_MOTOR_RIGHT_UP_SPEED = 0.2;
    public static final double TEST_MOTOR_RIGHT_DOWN_SPEED = -0.2;

    public static final double TEST_MOTOR_LEFT_TWO_UP_SPEED = 0.2;
    public static final double TEST_MOTOR_LEFT_TWO_DOWN_SPEED = -0.2;
    public static final double TEST_MOTOR_RIGHT_TWO_UP_SPEED = 0.2;
    public static final double TEST_MOTOR_RIGHT_TWO_DOWN_SPEED = -0.2;

    // TODO: Comfirm the new Climber Extension Limit
    public static final double CLIMBER_EXTENSION_LIMIT = 30.0;
    
    // Intake motor IDs
    public static final int TOP_INTAKE_MOTOR_ID = 20;
    public static final int BOTTOM_INTAKE_MOTOR_ID = 21;
    public static final int INSIDE_INTAKE_MOTOR_ID = 22;

    // Intake PID Values
    public static final double INTAKE_P_VALUE = 0.4;
    public static final double INTAKE_I_VALUE = 0.3;
    public static final double INTAKE_D_VALUE = 0.1;

    // Intake speeds
    // Verify if values are correct and go in the right directions
    public static final double TEST_TOP_MOTOR_IN_SPEED = 0.05;
    public static final double TEST_BOTTOM_MOTOR_IN_SPEED = 0.05;
    public static final double TEST_INSIDE_MOTOR_IN_SPEED = 0.05;

    public static final double TEST_TOP_MOTOR_OUT_SPEED = -0.05;
    public static final double TEST_BOTTOM_MOTOR_OUT_SPEED = -0.05;
    public static final double TEST_INSIDE_MOTOR_OUT_SPEED = -0.05;
    
    // Shooter Motor IDs
    public static final int TOP_SHOOTER_ONE_MOTOR_ID = 20;
    public static final int BOTTOM_SHOOTER_ONE_MOTOR_ID = 31;

    public static final int TOP_SHOOTER_TWO_MOTOR_ID = 32;
    public static final int BOTTOM_SHOOTER_TWO_MOTOR_ID = 33;

    // Shooter speeds
    public static final double TEST_MOTOR_SHOOTER_TOP_OUT_SPEED = 0.2;
    public static final double TEST_MOTOR_SHOOTER_BOTTOM_OUT_SPEED = 0.2;

    public static final double TEST_MOTOR_SHOOTER_TOP_IN_SPEED = -0.2;
    public static final double TEST_MOTOR_SHOOTER_BOTTOM_IN_SPEED = -0.2;

    public static final double SHOOTER_SPEED = 3;

    // Shooter PID Valuess
    public static final double SHOOTER_P_VALUE = 0.5;
    public static final double SHOOTER_I_VALUE = 0.15;
    public static final double SHOOTER_D_VALUE = 0.2;

    // Indexer ID
    public static final int INDEXER_MOTOR_ID = 40;

    // Indexer speeds
    public static final double INDEXER_MOTOR_SPEED = 0.5;

    // 
}
