package frc.BotchoCheese.Constants;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class PathPlannerSetpoints {
    private PathPlannerSetpoints() {}

    /*
     * Blue-alliance reference poses copied from PathPlanner path files.
     * Red-alliance values are derived at runtime via FlippingUtil.
     */
    public static final Pose2d HUB_HOME_POSE_BLUE =
        new Pose2d(3.5150734953703706, 4.049631148726852, Rotation2d.fromDegrees(180.0));
    public static final Pose2d LEFT_HUB_SHOOTING_POSE_BLUE =
        new Pose2d(2.3411698495370374, 4.943574652777778, Rotation2d.fromDegrees(145.57026926200894));
    public static final Pose2d MIDDLE_HUB_SHOOTING_POSE_BLUE =
        new Pose2d(2.416753182870371, 4.049631148726852, Rotation2d.fromDegrees(180.0));
    public static final Pose2d RIGHT_HUB_SHOOTING_POSE_BLUE =
        new Pose2d(2.647899088541667, 3.243736979166666, Rotation2d.fromDegrees(-150.49689428205946));

    public static final PathConstraints TELEOP_SHOT_PATHFIND_CONSTRAINTS =
        new PathConstraints(2.5, 2.0, 4.0, 6.0);

    public static final PathConstraints TELEOP_LEFT_SHOT_PATHFIND_CONSTRAINTS =
        new PathConstraints(1.5, 1.25, 2.0, 3.0);
}
