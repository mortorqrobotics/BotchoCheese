package frc.BotchoCheese.Commands;

import java.util.Arrays;
import java.util.stream.Collectors;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Utils.LimelightHelpers;

public class StrafeToTag extends Command {
    private CommandSwerveDrivetrain drivetrainSubsystem;
    private PIDController xController;
    private PIDController yController;
    private double xSetpoint;
    private double ySetpoint;
    private double offset;

    // Purpose: replaces the broken RobotContainer reflection approach with a reliable request object.
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    /**
     * Align robot with the target using the limelight
     *
     * @param drivetrainSubsystem drivetrain to command
     * @param offset desired offset distance from the tag target point
     */
    public StrafeToTag(CommandSwerveDrivetrain drivetrainSubsystem, double offset) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.offset = offset;

        xController = new PIDController(1, 3, 0);
        // TODO tune PID and tolerance
        xController.setTolerance(0.01);

        yController = new PIDController(5, 7, 0);
        // TODO tune PID and tolerance
        yController.setTolerance(0.01);

        addRequirements(drivetrainSubsystem);
        System.out.println("StrafeToTag constructed");
    }

    @Override

    public void initialize() {
        var results = LimelightHelpers.getRawFiducials(RobotMap.LIMELIGHT_2_NAME);
        var blah = Arrays.stream(results).map(c -> Integer.toString(c.id)).collect(Collectors.joining(","));
        System.out.println("all results " + blah + " blah");
        System.out.println("Var Results: " + results);
        // Use Limelight "tv" to confirm a valid fiducial target exists right now.
        if (!LimelightHelpers.getTV(RobotMap.LIMELIGHT_2_NAME)) {
            System.out.println("StrafeToTag initialize: no valid target (tv=false)");
            xSetpoint = drivetrainSubsystem.getState().Pose.getX();
            ySetpoint = drivetrainSubsystem.getState().Pose.getY();
            System.out.println("Setpoint: (" + xSetpoint + ", " + ySetpoint + ")");
        } else {
            int fid = (int) LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_2_NAME);
            var optionalTagPose = RobotMap.WELDED_FIELD2026.getTagPose(fid);
            System.out.println("Fiducial ID: " + fid);
            System.out.println("Optional Tag Pose: " + optionalTagPose);
            
            // optionalTagPose = Position of the AprilTag?
            if (optionalTagPose.isEmpty()) {
                // Target exists, but we can't map the ID to a pose in the field layout.
                System.out.println("StrafeToTag initialize: tag pose missing for fid=" + fid);
                xSetpoint = drivetrainSubsystem.getState().Pose.getX();
                ySetpoint = drivetrainSubsystem.getState().Pose.getY();
                System.out.println("Setpoint: (" + xSetpoint + ", " + ySetpoint + ")");
            } else {
                var tagPose = optionalTagPose.get();
                double angle = drivetrainSubsystem.getState().Pose.getRotation().getRadians();

                xSetpoint = tagPose.getX() - offset * Math.cos(angle);
                ySetpoint = tagPose.getY() - offset * Math.sin(angle);
                System.out.println("Tag Pose: " + tagPose);
                System.out.println("Angle: " + angle);
                System.out.println("Setpoint: (" + xSetpoint + ", " + ySetpoint + ")");
            }
        }
    // Sets the new speeds from the previous one
    xController.reset();
    yController.reset();
    xController.setSetpoint(xSetpoint);
    yController.setSetpoint(ySetpoint);

    System.out.println("StrafeToTag initialized\nControllers reset");
}

    @Override
    public void execute() {
        var robotPos = drivetrainSubsystem.getState().Pose;
        System.out.println("Robot Pose: " + robotPos);

        double xSpeed = xController.calculate(robotPos.getX());
        double ySpeed = yController.calculate(robotPos.getY());
        System.out.println("xSpeed: " + xSpeed);
        System.out.println("ySpeed: " + ySpeed);

        drivetrainSubsystem.setControl(
            driveRequest
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(0.0)
        );
    }

    //Checks whether it's at the target position
    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.setControl(new SwerveRequest.Idle());
    }
}