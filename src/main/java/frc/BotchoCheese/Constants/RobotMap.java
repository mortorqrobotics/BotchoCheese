package frc.BotchoCheese.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class RobotMap {
    // Pose estimator tuning for drivetrain odometry.
    // Larger values tell the estimator to trust its own position estimate less.
    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;

    // Pose estimator tuning for vision measurements.
    // Larger values tell the estimator to trust external global updates less.
    public static final double kVisionStdDevX = 2.5;
    public static final double kVisionStdDevY = 2.5;
    public static final double kVisionStdDevTheta = 180.0;

    // Single Limelight instance used for drivetrain pose updates.
    public static final String LIMELIGHT_NAME = "limelight-1515-1";

    // AprilTag field map used to bound/validate global pose estimates.
    public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    // AprilTag heights indexed by tag ID minus 1.
    public static final double[] TAG_HEIGHTS = {
    1.4859, 1.4859, /*ID: 1 - 2*/
    1.30175, /*ID: 3*/
    1.8679160000000001, 1.8679160000000001, /*ID: 4 - 5*/
    0.308102, 0.308102, 0.308102, 0.308102, 0.308102, 0.308102, /*ID: 6 - 11*/
    1.4859, 1.4859, /*ID: 12 - 13*/
    1.8679160000000001, 1.8679160000000001, /*ID: 14 - 15*/
    1.30175, /*ID: 16*/
    0.308102, 0.308102, 0.308102, 0.308102, 0.308102, 0.308102 /*ID: 17 - 22*/};

    // Device ID for the pigeon used by the drivetrain.
    public static final int PIGEON_ID = 30;
    
    // Intake motor controller CAN ID.
    public static final int INTAKE_MOTOR_ID = 22;

    // Pivot motor controller CAN IDs.
    public static final int LEFT_PIVOT_MOTOR_ID = 20;
    public static final int RIGHT_PIVOT_MOTOR_ID = 21;

    // Shooter motor controller CAN IDs.
    public static final int BACK_LEFT_SHOOTER_MOTOR_ID = 24;
    public static final int BACK_RIGHT_SHOOTER_MOTOR_ID = 25;
    public static final int FRONT_SHOOTER_MOTOR_ID = 26;

    // Indexer motor controller CAN ID.
    public static final int INDEXER_MOTOR_ID = 18;

    // Feeder motor controller CAN ID.
    public static final int FEEDER_MOTOR_ID = 23;

    // CANivore bus name for drivetrain hardware and CANdle device ID.
    public static final String CANIVORE_CAN_BUS = "1515Canivore";
    public static final int CANDLE_CAN_ID = 40;
}
