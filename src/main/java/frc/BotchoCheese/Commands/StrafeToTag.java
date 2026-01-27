package frc.BotchedCode.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.RobotContainer;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;
import frc.BotchedCode.Utils.LimelightHelpers;

public class StrafeToTag extends Command {
    private CommandSwerveDrivetrain drivetrainSubsystem;
    private PIDController xController;
    private PIDController yController;
    private double xSetpoint; 
    private double ySetpoint;
    private double offset;
    

    /**
     * Align robot with the target using the limelight
     * 
     * @param drivetrainSubsystem
     * @param limelight
     */
    public StrafeToTag(CommandSwerveDrivetrain drivetrainSubsystem, double offset) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.offset = offset;
        xController = new PIDController(1, 3, 0);
        // TODO tune PID and tolerance
        xController.setTolerance(0.01);
        xController.setSetpoint(xSetpoint);

        yController = new PIDController(5, 7, 0);
        // TODO tune PID and tolerance
        yController.setTolerance(0.01);
        yController.setSetpoint(ySetpoint);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        var tagPose = RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID("limelight")).get();
        double angle = RobotContainer.drivetrain.getState().Pose.getRotation().getRadians();
        xSetpoint = tagPose.getX() - offset*Math.cos(angle);
        ySetpoint = tagPose.getY() - offset*Math.sin(angle);
    }

    @Override
    public void execute() {
        //double floorDist = tagHeight/Math.tan(Units.degreesToRadians(LimelightHelpers.getTY(RobotMap.LIMELIGHT_NAME) + 30));
        //double tx = LimelightHelpers.getTX(RobotMap.LIMELIGHT_NAME);
        //System.out.println("FloorDist: " + floorDist + "   TX: " + tx);
        //double xSpeed = xController.calculate(floorDist, xOffset);
        //double ySpeed = yController.calculate(tx, 0);
        var robotPos = RobotContainer.drivetrain.getState().Pose;

        double xSpeed = xController.calculate(robotPos.getX(), xSetpoint);
        double ySpeed = xController.calculate(robotPos.getY(), ySetpoint);

        System.out.println("XPose: " + robotPos.getX() + "   YPose: " + robotPos.getY());
        System.out.println("FXPose: " + xSetpoint + "   FYPose: " + ySetpoint);
        System.out.println("xSpeed: " + xSpeed + "   ySpeed: " + ySpeed);
                
        drivetrainSubsystem.setControl(
            RobotContainer.drive.withVelocityX(xSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(ySpeed) // Drive left with negative X (left)
            .withRotationalRate(0)
        ); // Drive counterclockwise with negative X (left)
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint();
    }
}