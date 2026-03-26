package frc.BotchoCheese.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Utils.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.BotchoCheese.Utils.LimelightHelpers.RawFiducial;

public class StrafeToTag extends Command {
    private final CommandSwerveDrivetrain drivetrainSubsystem;
    
    // We use FieldCentric so the position controller can drive to field X/Y setpoints.
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    private final PIDController xController;
    private final PIDController yController;

    private static final double offset = 0.0;
    private double xSetpoint;
    private double ySetpoint;
    private boolean lostTarget = false;

    /**
     * Align robot with the target using the Limelight camera
     *
      * @param drivetrainSubsystem drivetrain to command
     */
    public StrafeToTag(CommandSwerveDrivetrain drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;

        xController = new PIDController(1, 0, 0);
        yController = new PIDController(1, 0, 0);

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);

        addRequirements(drivetrainSubsystem);
    }
    @Override
    public void initialize() {
        lostTarget = false;

        RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(RobotMap.LIMELIGHT_NAME);
        Pose2d robotPose = drivetrainSubsystem.getState().Pose;

        if (rawFiducials.length == 0 || !LimelightHelpers.getTV(RobotMap.LIMELIGHT_NAME)) {
            lostTarget = true;
            xSetpoint = robotPose.getX();
            ySetpoint = robotPose.getY();
        } else {
            int trackedId = (int) LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_NAME);

            boolean trackedIdSeen = false;
            for (RawFiducial fiducial : rawFiducials) {
                if (fiducial.id == trackedId) {
                    trackedIdSeen = true;
                    break;
                }
            }

            if (!trackedIdSeen) {
                lostTarget = true;
                xSetpoint = robotPose.getX();
                ySetpoint = robotPose.getY();
            } else {
                Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(RobotMap.LIMELIGHT_NAME);

                double forwardToTagMeters = targetPoseRobotSpace.getX();
                double leftToTagMeters = targetPoseRobotSpace.getY();

                double forwardDeltaMeters = forwardToTagMeters - offset;
                double leftDeltaMeters = leftToTagMeters;

                Translation2d fieldDelta = new Translation2d(forwardDeltaMeters, leftDeltaMeters)
                    .rotateBy(robotPose.getRotation());

                xSetpoint = robotPose.getX() + fieldDelta.getX();
                ySetpoint = robotPose.getY() + fieldDelta.getY();
            }
        }

        xController.reset();
        yController.reset();
        xController.setSetpoint(xSetpoint);
        yController.setSetpoint(ySetpoint);

        System.out.println("StrafeToTag initialized");
    }


    @Override
    public void execute() {
        if (lostTarget) {
            drivetrainSubsystem.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        var robotPose = drivetrainSubsystem.getState().Pose;
        double xSpeed = xController.calculate(robotPose.getX());
        double ySpeed = yController.calculate(robotPose.getY());

        xSpeed = Math.max(-0.25, Math.min(0.25, xSpeed)); 
        ySpeed = Math.max(-0.25, Math.min(0.25, ySpeed));

        drivetrainSubsystem.setControl(
            driveRequest
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(0.0)
        );
    }

    @Override
    public boolean isFinished() {
        return lostTarget || (xController.atSetpoint() && yController.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends or the driver lets go of the button
        drivetrainSubsystem.setControl(new SwerveRequest.Idle());
        System.out.println("StrafeToTag Ended");
    }
}
