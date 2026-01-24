package frc.BotchedCode.Constants;

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
}
