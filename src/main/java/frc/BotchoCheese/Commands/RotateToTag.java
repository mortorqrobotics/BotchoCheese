package frc.BotchoCheese.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Utils.LimelightHelpers;

public class RotateToTag extends Command {
    private CommandSwerveDrivetrain drivetrainSubsystem;
    private PIDController angleController;
    private double angleSetpoint; 
    private double angleOffset;
    private boolean validTarget = false;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();


    /**
     * Align robot with the target using the limelight
     * 
     * @param drivetrainSubsystem
     * @param limelight
     */
    public RotateToTag(CommandSwerveDrivetrain drivetrainSubsystem, double angleOffset) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.angleOffset = angleOffset;
        angleController = new PIDController(2, 0, 0);
        angleController.setTolerance(0.025);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrainSubsystem);
    }

    // Gets the specific position of the AprilTag
    @Override
    public void initialize() {
        // var tagPose = RobotMap.WELDED_FIELD2026.getTagPose((int) LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_NAME)).get();
        // angleSetpoint = tagPose.getRotation().getAngle() + Math.PI + angleOffset;
        // angleController.setSetpoint(angleSetpoint);
        // System.out.println("Tag Pose: " + tagPose);
        // System.out.println("Angle Setpoint: " + angleSetpoint);
        validTarget = LimelightHelpers.getTV(RobotMap.LIMELIGHT_NAME);
        if (!validTarget) return;

        int fid = (int) LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_NAME);
        var tagPoseOpt = RobotMap.WELDED_FIELD2026.getTagPose(fid);
        if (tagPoseOpt.isEmpty()) {
            validTarget = false;
            return;
        }

        var tagPose = tagPoseOpt.get();
        angleSetpoint = tagPose.getRotation().toRotation2d().getRadians() + Math.PI + angleOffset;
        System.out.println("Angle Setpoint (Degrees): " + tagPose.getRotation().toRotation2d());
        System.out.println("Angle Setpoint " + angleSetpoint);
        angleController.reset();
        angleController.setSetpoint(angleSetpoint);
    }

    @Override
    public void execute() {
        validTarget = LimelightHelpers.getTV(RobotMap.LIMELIGHT_NAME);
        if (!validTarget) {
            drivetrainSubsystem.setControl(
                driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
            );
            return;
        }

        double currentAngle = drivetrainSubsystem.getState().Pose.getRotation().getRadians();
        double rotation = angleController.calculate(currentAngle, angleSetpoint);
        System.out.println("Target Rotation (Before Max) " + rotation);

        rotation = Math.max(-3.0, Math.min(3.0, rotation));

        drivetrainSubsystem.setControl(
            driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(rotation)
        );
    }

    @Override
    public boolean isFinished() {
        boolean finished = angleController.atSetpoint();
        return finished;
    }

    @Override
    public void end(boolean finished) {
        drivetrainSubsystem.setControl(new SwerveRequest.Idle());
    }

}