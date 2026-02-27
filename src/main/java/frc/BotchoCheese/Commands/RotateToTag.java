// DON'T touch file until StrafeToTag code is resolved

package frc.BotchoCheese.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchoCheese.RobotContainer;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Utils.LimelightHelpers;

public class RotateToTag extends Command {
    private CommandSwerveDrivetrain drivetrainSubsystem;
    // l
    private PIDController angleController;
    private double angleSetpoint; 
    private double angleOffset; 
    

    /**
     * Align robot with the target using the limelight
     * 
     * @param drivetrainSubsystem
     * @param limelight
     */
    public RotateToTag(CommandSwerveDrivetrain drivetrainSubsystem, double angleOffset) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.angleOffset = angleOffset;
        angleController = new PIDController(10, 0, 0);
        // TODO tune PID and tolerance
        angleController.setTolerance(0.025);
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setSetpoint(angleSetpoint);

        addRequirements(drivetrainSubsystem);
    }

    // Gets the specific position of the AprilTag
    @Override
    public void initialize(){
        var tagPose = RobotMap.WELDED_FIELD2026.getTagPose((int) LimelightHelpers.getFiducialID("limelight")).get();
        angleSetpoint = tagPose.getRotation().getAngle() + Math.PI + angleOffset;
        System.out.println("Tag Pose: " + tagPose);
        System.out.println("Angle Setpoint: " + angleSetpoint);
    }

    @Override
    public void execute() {
        double rotation = angleController.calculate(drivetrainSubsystem.getState().Pose.getRotation().getRadians(), angleSetpoint);
        System.out.println("Rotation: " + rotation);
        drivetrainSubsystem.setControl( //Does it need to move to rotate (drive with x/y 0)?
            RobotContainer.drive.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(rotation)
        ); // Drive counterclockwise with negative X (left)
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint();
    }
}