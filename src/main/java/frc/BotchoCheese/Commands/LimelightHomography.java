package frc.BotchoCheese.Commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Utils.LimelightHelpers;
import frc.BotchoCheese.Utils.LimelightHelpers.PoseEstimate;

public final class LimelightHomography {
    // Start with conservative thresholds, then tune from logs.
    private static final double kMaxOmegaRpsForVision = 2.0;
    private static final double kMaxPoseDisagreementMeters = 1.0;
    private static final double kMaxHeadingDisagreementDeg = 20.0;
    private static final double kMinTagArea = 0.02;
    private static final double kMaxTagDistanceMeters = 7.0;
    private static final double kMaxPoseJumpFromOdomMeters = 2.0;

    private LimelightHomography() {}

    public static void update(CommandSwerveDrivetrain drivetrain) {
        // Step 0: read current drivetrain state (our odometry reference).
        var driveState = drivetrain.getState();
        Pose2d odomPose = driveState.Pose;
        // Use odometry heading for LL orientation sync.
        double headingDeg = odomPose.getRotation().getDegrees();
        // Convert angular velocity to rotations/sec for an easy gate threshold.
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        // Hard gate: reject all vision while spinning quickly.
        if (Math.abs(omegaRps) >= kMaxOmegaRpsForVision) {
            SmartDashboard.putString("VisionFusion/Mode", "REJECTED_HIGH_OMEGA");
            SmartDashboard.putBoolean("VisionFusion/LL1Valid", false);
            SmartDashboard.putBoolean("VisionFusion/LL2Valid", false);
            return;
        }
        
        // TODO: Check if needed to change to red
        // Feed current robot heading into both LLs to stabilize MegaTag2 orientation.
        LimelightHelpers.SetRobotOrientation(RobotMap.LIMELIGHT_NAME, headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(RobotMap.LIMELIGHT_2_NAME, headingDeg, 0, 0, 0, 0, 0);

        // Step 1: ask each camera for a MegaTag2 pose estimate.
        PoseEstimate ll1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotMap.LIMELIGHT_NAME);
        PoseEstimate ll2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotMap.LIMELIGHT_2_NAME);

        // Step 2: run hard validation gates on each estimate.
        boolean ll1Valid = isValid(ll1, odomPose);
        boolean ll2Valid = isValid(ll2, odomPose);
        // Step 2a: publish validity so we can debug in SmartDashboard.
        SmartDashboard.putBoolean("VisionFusion/LL1Valid", ll1Valid);
        SmartDashboard.putBoolean("VisionFusion/LL2Valid", ll2Valid);

        // Step 3: neither estimate is usable, skip this loop.
        if (!ll1Valid && !ll2Valid) {
            SmartDashboard.putString("VisionFusion/Mode", "NO_VALID_INPUT");
            return;
        }

        // Step 4: only camera 1 is valid, so use camera 1 directly.
        if (ll1Valid && !ll2Valid) {
            addVision(drivetrain, ll1);
            publishVisionError(ll1.pose, odomPose);
            SmartDashboard.putString("VisionFusion/Mode", "LL1_ONLY");
            return;
        }

        // Step 5: only camera 2 is valid, so use camera 2 directly.
        if (!ll1Valid && ll2Valid) {
            addVision(drivetrain, ll2);
            publishVisionError(ll2.pose, odomPose);
            SmartDashboard.putString("VisionFusion/Mode", "LL2_ONLY");
            return;
        }

        // Both valid: compare disagreement before deciding to fuse or pick one.
        Pose2d p1 = ll1.pose;
        Pose2d p2 = ll2.pose;
        // Compare translation disagreement.
        double deltaMeters = p1.getTranslation().getDistance(p2.getTranslation());
        // Compare heading disagreement.
        double deltaHeadingDeg = Math.abs(p1.getRotation().minus(p2.getRotation()).getDegrees());
        // Publish disagreement so we can tune fusion thresholds.
        SmartDashboard.putNumber("VisionFusion/DeltaMeters", deltaMeters);
        SmartDashboard.putNumber("VisionFusion/DeltaHeadingDeg", deltaHeadingDeg);

        // Step 6a: if they agree, fuse into one blended translation.
        if (deltaMeters <= kMaxPoseDisagreementMeters && deltaHeadingDeg <= kMaxHeadingDisagreementDeg) {
            Pose2d fusedPose = weightedAveragePose(p1, score(ll1), p2, score(ll2), odomPose.getRotation());
            // Use the newer of the two timestamps for the fused pose.
            double fusedTimestamp = Math.max(ll1.timestampSeconds, ll2.timestampSeconds);

            // Convert FPGA-based LL timestamp before feeding drivetrain estimator.
            drivetrain.addVisionMeasurement(fusedPose, Utils.fpgaToCurrentTime(fusedTimestamp));
            publishVisionError(fusedPose, odomPose);
            SmartDashboard.putString("VisionFusion/Mode", "FUSED_AVERAGE");
            return;
        }

        // Step 6b: if they disagree, choose the higher-scoring camera.
        PoseEstimate selected = score(ll1) >= score(ll2) ? ll1 : ll2;
        addVision(drivetrain, selected);
        publishVisionError(selected.pose, odomPose);
        SmartDashboard.putString("VisionFusion/Mode", selected == ll1 ? "LL1_SELECTED" : "LL2_SELECTED");
    }

    private static void addVision(CommandSwerveDrivetrain drivetrain, PoseEstimate est) {
        // Push one accepted vision sample into the drivetrain pose estimator.
        drivetrain.addVisionMeasurement(est.pose, Utils.fpgaToCurrentTime(est.timestampSeconds));
    }

    private static boolean isValid(PoseEstimate est, Pose2d odomPose) {
        // Basic null/checks from Limelight helper.
        if (!LimelightHelpers.validPoseEstimate(est)) {
            return false;
        }

        // Hard quality gates for raw camera solve quality.
        if (est.tagCount <= 0 || est.avgTagArea < kMinTagArea || est.avgTagDist > kMaxTagDistanceMeters) {
            return false;
        }

        Pose2d p = est.pose;
        double x = p.getX();
        double y = p.getY();
        // Field dimensions from the official 2026 field layout.
        double maxX = RobotMap.WELDED_FIELD2026.getFieldLength();
        double maxY = RobotMap.WELDED_FIELD2026.getFieldWidth();

        // Reject impossible positions outside the field (+ small margin).
        boolean inField = x >= -0.25 && x <= maxX + 0.25 && y >= -0.25 && y <= maxY + 0.25;
        if (!inField) {
            return false;
        }

        // Optional "huge jump" gate against current odometry pose.
        double jump = p.getTranslation().getDistance(odomPose.getTranslation());
        return jump <= kMaxPoseJumpFromOdomMeters;
    }

    private static double score(PoseEstimate est) {
        // Higher score means "trust this camera more".
        return (est.tagCount * 2.0) + (est.avgTagArea * 10.0) - est.avgTagDist;
    }

    private static Pose2d weightedAveragePose(
        Pose2d p1,
        double w1,
        Pose2d p2,
        double w2,
        Rotation2d trustedHeading
    ) {
        // Prevent divide-by-zero if scores are tiny or negative.
        double safeW1 = Math.max(0.001, w1);
        double safeW2 = Math.max(0.001, w2);
        double sum = safeW1 + safeW2;
        // Blend X and Y by camera quality score.
        double x = (p1.getX() * safeW1 + p2.getX() * safeW2) / sum;
        double y = (p1.getY() * safeW1 + p2.getY() * safeW2) / sum;

        x = MathUtil.clamp(x, -0.25, RobotMap.WELDED_FIELD2026.getFieldLength() + 0.25);
        y = MathUtil.clamp(y, -0.25, RobotMap.WELDED_FIELD2026.getFieldWidth() + 0.25);

        // Keep heading from odometry/gyro chain by default.
        return new Pose2d(x, y, trustedHeading);
    }

    private static void publishVisionError(Pose2d visionPose, Pose2d odomPose) {
        // Useful live metric: how far vision currently is from odometry.
        double errorMeters = visionPose.getTranslation().getDistance(odomPose.getTranslation());
        SmartDashboard.putNumber("VisionFusion/VisionOdomErrorMeters", errorMeters);
    }
}
