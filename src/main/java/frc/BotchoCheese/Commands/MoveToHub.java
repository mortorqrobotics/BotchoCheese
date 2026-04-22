package frc.BotchoCheese.Commands;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Utils.LimelightHelpers;

public class MoveToHub extends Command {
    private static final Set<Integer> RED_HUB_TAG_IDS = Set.of(9, 10);
    private static final Set<Integer> BLUE_HUB_TAG_IDS = Set.of(25, 26);

    private static final double DEFAULT_STANDOFF_METERS = 0.0;
    private static final double DEFAULT_ARC_OFFSET_PER_METER = 0.4;
    private static final double DEFAULT_ARC_DIRECTION = 1.0;
    private static final double ANGLE_OFFSET_RADIANS = 0.0;
    private static final double FORWARD_KP = 1.0;
    private static final double LATERAL_KP = 1.0;
    private static final double ROTATION_KP = 2.0;
    private static final double MAX_TRANSLATION_SPEED_MPS = 0.75;
    private static final double MAX_ROTATION_SPEED_RAD_PER_SEC = 3.0;
    private static final double DISTANCE_TOLERANCE_METERS = 0.10;
    private static final double LATERAL_TOLERANCE_METERS = 0.10;
    private static final double ANGLE_TOLERANCE_RADIANS = 0.05;

    private final CommandSwerveDrivetrain drivetrainSubsystem;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    private final PIDController angleController = new PIDController(ROTATION_KP, 0.0, 0.0);

    private final double standoffMeters;
    private final double arcOffsetPerMeter;
    private final double arcDirection;

    private boolean validTarget = false;
    private double forwardErrorMeters = 0.0;
    private double lateralErrorMeters = 0.0;
    private double angleErrorRadians = 0.0;

    public MoveToHub(CommandSwerveDrivetrain drivetrainSubsystem) {
        this(drivetrainSubsystem, DEFAULT_STANDOFF_METERS, DEFAULT_ARC_OFFSET_PER_METER, DEFAULT_ARC_DIRECTION);
    }

    public MoveToHub(
        CommandSwerveDrivetrain drivetrainSubsystem,
        double standoffMeters,
        double arcOffsetPerMeter,
        double arcDirection
    ) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.standoffMeters = standoffMeters;
        this.arcOffsetPerMeter = arcOffsetPerMeter;
        this.arcDirection = Math.signum(arcDirection) == 0.0 ? 1.0 : Math.signum(arcDirection);

        angleController.setTolerance(ANGLE_TOLERANCE_RADIANS);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        angleController.reset();
        updateControlTargets();
    }

    @Override
    public void execute() {
        updateControlTargets();
        if (!validTarget) {
            drivetrainSubsystem.setControl(
                driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)
            );
            return;
        }

        double forwardSpeed =
            MathUtil.clamp(
                FORWARD_KP * forwardErrorMeters,
                -MAX_TRANSLATION_SPEED_MPS,
                MAX_TRANSLATION_SPEED_MPS
            );

        double lateralSpeed =
            MathUtil.clamp(
                LATERAL_KP * lateralErrorMeters,
                -MAX_TRANSLATION_SPEED_MPS,
                MAX_TRANSLATION_SPEED_MPS
            );

        double currentAngle = drivetrainSubsystem.getState().Pose.getRotation().getRadians();
        double rotationSpeed =
            MathUtil.clamp(
                angleController.calculate(currentAngle),
                -MAX_ROTATION_SPEED_RAD_PER_SEC,
                MAX_ROTATION_SPEED_RAD_PER_SEC
            );

        drivetrainSubsystem.setControl(
            driveRequest
                .withVelocityX(forwardSpeed)
                .withVelocityY(lateralSpeed)
                .withRotationalRate(rotationSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        return !validTarget
            || (Math.abs(forwardErrorMeters) <= DISTANCE_TOLERANCE_METERS
                && Math.abs(lateralErrorMeters) <= LATERAL_TOLERANCE_METERS
                && Math.abs(angleErrorRadians) <= ANGLE_TOLERANCE_RADIANS);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.setControl(new SwerveRequest.Idle());
    }

    private void updateControlTargets() {
        validTarget = LimelightHelpers.getTV(RobotMap.LIMELIGHT_NAME);
        if (!validTarget) {
            return;
        }

        int fiducialId = (int) LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_NAME);
        if (!isAllianceHubTag(fiducialId)) {
            validTarget = false;
            return;
        }

        var tagPoseOpt = RobotMap.WELDED_FIELD2026.getTagPose(fiducialId);
        if (tagPoseOpt.isEmpty()) {
            validTarget = false;
            return;
        }

        var targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(RobotMap.LIMELIGHT_NAME);
        double forwardToTagMeters = targetPoseRobotSpace.getX();
        double leftToTagMeters = targetPoseRobotSpace.getY();

        forwardErrorMeters = forwardToTagMeters - standoffMeters;

        double desiredLateralOffsetMeters = arcDirection * arcOffsetPerMeter * Math.abs(forwardErrorMeters);
        lateralErrorMeters = leftToTagMeters - desiredLateralOffsetMeters;

        double targetAngle =
            tagPoseOpt.get().getRotation().toRotation2d().getRadians() + Math.PI + ANGLE_OFFSET_RADIANS;
        angleController.setSetpoint(targetAngle);

        double currentAngle = drivetrainSubsystem.getState().Pose.getRotation().getRadians();
        angleErrorRadians = MathUtil.angleModulus(targetAngle - currentAngle);
    }

    private boolean isAllianceHubTag(int fiducialId) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }

        return alliance.get() == DriverStation.Alliance.Red
            ? RED_HUB_TAG_IDS.contains(fiducialId)
            : BLUE_HUB_TAG_IDS.contains(fiducialId);
    }
}
