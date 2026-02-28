// package frc.BotchoCheese.Commands;

// import java.util.Arrays;
// import java.util.stream.Collectors;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.BotchoCheese.Constants.RobotMap;
// import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
// import frc.BotchoCheese.Utils.LimelightHelpers;

// public class StrafeToTag extends Command {
//     private CommandSwerveDrivetrain drivetrainSubsystem;
//     private PIDController xController;
//     private PIDController yController;
//     private double xSetpoint;
//     private double ySetpoint;
//     private double offset;

//     // Purpose: replaces the broken RobotContainer reflection approach with a reliable request object.
//     private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

//     /**
//      * Align robot with the target using the limelight
//      *
//      * @param drivetrainSubsystem drivetrain to command
//      * @param offset desired offset distance from the tag target point
//      */
//     public StrafeToTag(CommandSwerveDrivetrain drivetrainSubsystem, double offset) {
//         this.drivetrainSubsystem = drivetrainSubsystem;
//         this.offset = offset;

//         xController = new PIDController(1, 3, 0);
//         // TODO tune PID and tolerance
//         xController.setTolerance(0.01);

//         yController = new PIDController(5, 7, 0);
//         // TODO tune PID and tolerance
//         yController.setTolerance(0.01);

//         addRequirements(drivetrainSubsystem);
//         System.out.println("StrafeToTag constructed");
//     }

//     @Override

//     public void initialize() {
//         var results = LimelightHelpers.getRawFiducials(RobotMap.LIMELIGHT_NAME);
//         var blah = Arrays.stream(results).map(c -> Integer.toString(c.id)).collect(Collectors.joining(","));
//         System.out.println("all results " + blah + " blah");
//         System.out.println("Var Results: " + results);
//         // Use Limelight "tv" to confirm a valid fiducial target exists right now.
//         if (!LimelightHelpers.getTV(RobotMap.LIMELIGHT_NAME)) {
//             System.out.println("StrafeToTag initialize: no valid target (tv=false)");
//             xSetpoint = drivetrainSubsystem.getState().Pose.getX();
//             ySetpoint = drivetrainSubsystem.getState().Pose.getY();
//             System.out.println("Setpoint: (" + xSetpoint + ", " + ySetpoint + ")");
//         } else {
//             int fid = (int) LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_NAME);
//             var optionalTagPose = RobotMap.WELDED_FIELD2026.getTagPose(fid);
//             System.out.println("Fiducial ID: " + fid);
//             System.out.println("Optional Tag Pose: " + optionalTagPose);
            
//             // optionalTagPose = Position of the AprilTag?
//             if (optionalTagPose.isEmpty()) {
//                 // Target exists, but we can't map the ID to a pose in the field layout.
//                 System.out.println("StrafeToTag initialize: tag pose missing for fid=" + fid);
//                 xSetpoint = drivetrainSubsystem.getState().Pose.getX();
//                 ySetpoint = drivetrainSubsystem.getState().Pose.getY();
//                 System.out.println("Setpoint: (" + xSetpoint + ", " + ySetpoint + ")");
//             } else {
//                 var tagPose = optionalTagPose.get();
//                 double angle = drivetrainSubsystem.getState().Pose.getRotation().getRadians();

//                 xSetpoint = tagPose.getX() - offset * Math.cos(angle);
//                 ySetpoint = tagPose.getY() - offset * Math.sin(angle);
//                 System.out.println("Tag Pose: " + tagPose);
//                 System.out.println("Angle: " + angle);
//                 System.out.println("Setpoint: (" + xSetpoint + ", " + ySetpoint + ")");
//             }
//         }
//     // Sets the new speeds from the previous one
//     xController.reset();
//     yController.reset();
//     xController.setSetpoint(xSetpoint);
//     yController.setSetpoint(ySetpoint);

//     System.out.println("StrafeToTag initialized\nControllers reset");
// }

//     @Override
//     public void execute() {
//         var robotPos = drivetrainSubsystem.getState().Pose;
//         System.out.println("Robot Pose: " + robotPos);

//         double xSpeed = xController.calculate(robotPos.getX()) * 0.2;
//         double ySpeed = yController.calculate(robotPos.getY()) * 0.2;
//         System.out.println("xSpeed: " + xSpeed);
//         System.out.println("ySpeed: " + ySpeed);

//         drivetrainSubsystem.setControl(
//             driveRequest
//                 .withVelocityX(xSpeed)
//                 .withVelocityY(ySpeed)
//                 .withRotationalRate(0.0)
//         );
//     }

//     //Checks whether it's at the target position
//     @Override
//     public boolean isFinished() {
//         return xController.atSetpoint() && yController.atSetpoint();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrainSubsystem.setControl(new SwerveRequest.Idle());
//     }
// }
package frc.BotchoCheese.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Subsystems.CommandSwerveDrivetrain;
import frc.BotchoCheese.Utils.LimelightHelpers;

public class StrafeToTag extends Command {
    private final CommandSwerveDrivetrain drivetrainSubsystem;
    
    // We use RobotCentric so X is always "Robot Forward" and Y is always "Robot Left"
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    // PID for moving forward/backward (X axis) based on Limelight 'ty'
    private final PIDController forwardController;
    // PID for strafing left/right (Y axis) based on Limelight 'tx'
    private final PIDController strafeController;

    private final double targetTy;

    /**
     * Align robot with the target using the Limelight camera
     *
     * @param drivetrainSubsystem drivetrain to command
     * @param desiredTyOffset The target 'ty' (pitch) angle in degrees. Controls how far away the robot stops.
     */
    public StrafeToTag(CommandSwerveDrivetrain drivetrainSubsystem, double desiredTyOffset) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.targetTy = desiredTyOffset; // Replaces your old odometry offset

        // Standard vision PID starting points. 
        // 0.05 means for every 1 degree of error, the robot moves at 0.05 m/s.
        forwardController = new PIDController(0.05, 0, 0);
        strafeController = new PIDController(0.05, 0, 0);

        // Tolerance is now in Limelight degrees, not field meters
        forwardController.setTolerance(1.0);
        strafeController.setTolerance(1.0);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        forwardController.reset();
        strafeController.reset();
        
        // We want 'tx' to be exactly 0 (centered horizontally on the screen)
        strafeController.setSetpoint(0.0);
        // We want 'ty' to match our target distance
        forwardController.setSetpoint(targetTy);
        
        System.out.println("StrafeToTag Initialized in RobotCentric Vision Mode");
    }

    @Override
    public void execute() {
        // SAFETY CHECK: If the Limelight loses the target, stop moving immediately.
        if (!LimelightHelpers.getTV(RobotMap.LIMELIGHT_NAME)) {
            drivetrainSubsystem.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        // Get live vision data
        double tx = LimelightHelpers.getTX(RobotMap.LIMELIGHT_NAME);
        double ty = LimelightHelpers.getTY(RobotMap.LIMELIGHT_NAME);

        // --- THE MATH ---
        // Limelight ty is positive when target is HIGH (farther). 
        // We need to drive FORWARD (Positive X velocity) to get closer.
        //NOTE: Ty should go with xSpeed and tx should go with ySpeed
        double xSpeed = forwardController.calculate(ty);
        
        // Limelight tx is positive when target is RIGHT. 
        // We need to strafe RIGHT (Negative Y velocity) to center it. 
        // Notice the negative sign!
        double ySpeed = -strafeController.calculate(tx); 

        // Cap the maximum speeds so the robot doesn't fly out of control during tuning (Max 1.5 m/s)
        // TODO Change the speed caps
        xSpeed = Math.max(-0.25, Math.min(0.25, xSpeed)); 
        ySpeed = Math.max(-0.25, Math.min(0.25, ySpeed));

        // Send the command to the Swerve drivetrain
        drivetrainSubsystem.setControl(
            driveRequest
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(0.0) // Keeps the robot locked facing straight forward
        );
    }

    @Override
    public boolean isFinished() {
        // Command finishes when the Limelight crosshair is resting on the setpoints
        return forwardController.atSetpoint() && strafeController.atSetpoint();
    }

    // TODO Figure out how to make it stop
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends or the driver lets go of the button
        drivetrainSubsystem.setControl(new SwerveRequest.Idle());
        System.out.println("StrafeToTag Ended");
    }
    // @Override
    // public void end(boolean interrupted) {
    //     drivetrainSubsystem.setControl(
    //         driveRequest
    //             .withVelocityX(0.0)
    //             .withVelocityY(0.0)
    //             .withRotationalRate(0.0)
    //     );
    //     System.out.println("StrafeToTag Ended");
    // }
}