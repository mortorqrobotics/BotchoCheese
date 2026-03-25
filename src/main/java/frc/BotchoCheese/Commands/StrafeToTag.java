package frc.BotchoCheese.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Utils.LimelightHelpers;

public class StrafeToTag extends Command {
    private final CommandSwerveDrivetrain drivetrainSubsystem;
    
    // We use FieldCentric so the position controller can drive to field X/Y setpoints.
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    private final PIDController xController;
    private final PIDController yController;

    private final double offset;
    private double xSetpoint;
    private double ySetpoint;
    private boolean lostTarget = false;

    /**
     * Align robot with the target using the Limelight camera
     *
      * @param drivetrainSubsystem drivetrain to command
     * @param offset desired offset from the AprilTag in meters
     */
    public StrafeToTag(CommandSwerveDrivetrain drivetrainSubsystem, double offset) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.offset = offset;

        xController = new PIDController(1, 0, 0);
        yController = new PIDController(1, 0, 0);

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        lostTarget = false;

        if (!LimelightHelpers.getTV(RobotMap.LIMELIGHT_NAME)) {
            lostTarget = true;
            xSetpoint = drivetrainSubsystem.getState().Pose.getX();
            ySetpoint = drivetrainSubsystem.getState().Pose.getY();
        } else {
            int fid = (int) LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_NAME);
            var optionalTagPose = RobotMap.WELDED_FIELD2026.getTagPose(fid);

            if (optionalTagPose.isEmpty()) {
                lostTarget = true;
                xSetpoint = drivetrainSubsystem.getState().Pose.getX();
                ySetpoint = drivetrainSubsystem.getState().Pose.getY();
            } else {
                var tagPose = optionalTagPose.get();
                double angle = drivetrainSubsystem.getState().Pose.getRotation().getRadians();

                xSetpoint = tagPose.getX() - offset * Math.cos(angle);
                ySetpoint = tagPose.getY() - offset * Math.sin(angle);
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
