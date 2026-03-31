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
    
    // Intake motor IDs
    public static final int INTAKE_MOTOR_ID = 22;

    // Pivot motor IDs
    public static final int LEFT_PIVOT_MOTOR_ID = 20;
    public static final int RIGHT_PIVOT_MOTOR_ID = 21;
  
  ; // Shooter motor IDs
    public static final int BACK_LEFT_SHOOTER_MOTOR_ID = 24;
    public static final int BACK_RIGHT_SHOOTER_MOTOR_ID = 25;
    public static final int FRONT_SHOOTER_MOTOR_ID = 26;

    // Indexer ID
    public static final int INDEXER_MOTOR_ID = 18;

    // Feeder ID
    public static final int FEEDER_MOTOR_ID = 23;

    // CANdle hardware mapping
    public static final String CANIVORE_CAN_BUS = "1515Canivore";
    public static final int CANDLE_CAN_ID = 40;
}
