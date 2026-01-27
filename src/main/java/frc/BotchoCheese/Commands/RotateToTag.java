package frc.BotchedCode.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.RobotContainer;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;
import frc.BotchedCode.Utils.LimelightHelpers;

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

    @Override
    public void initialize(){
        var tagPose = RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID("limelight")).get();
        angleSetpoint = tagPose.getRotation().getAngle() + Math.PI + angleOffset;
    }

    @Override
    public void execute() {
        double rotation = angleController.calculate(drivetrainSubsystem.getState().Pose.getRotation().getRadians(), angleSetpoint);
                
        drivetrainSubsystem.setControl(
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