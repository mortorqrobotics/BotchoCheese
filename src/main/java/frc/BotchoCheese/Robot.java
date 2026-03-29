// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchoCheese;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.BotchoCheese.Commands.LimelightHomography;
import frc.BotchoCheese.Utils.LimelightHelpers;


public class Robot extends TimedRobot {

  public static LimelightHelpers limelight;
  public static LimelightHelpers limelightTwo;

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;

  private StructPublisher<Pose2d> publisher;

  private final Field2d m_field = new Field2d();

  public Robot() {
    m_robotContainer = new RobotContainer();

    publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct)
    .publish();

    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(RobotContainer.drivetrain.getState().Pose);
    DataLogManager.start(); 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //Module Offsets
    SmartDashboard.putNumber("Mod0 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("Mod1 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(1).getEncoder().getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("Mod2 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(2).getEncoder().getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("Mod3 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(3).getEncoder().getAbsolutePosition().getValueAsDouble()));

    SmartDashboard.putNumber("Mod0 Drive Speed", RobotContainer.drivetrain.getModule(0).getEncoder().getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Mod1 Drive Speed", RobotContainer.drivetrain.getModule(1).getEncoder().getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Mod2 Drive Speed", RobotContainer.drivetrain.getModule(2).getEncoder().getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Mod3 Drive Speed", RobotContainer.drivetrain.getModule(3).getEncoder().getVelocity().getValueAsDouble());

    // SmartDashboard.putNumber("Mod0 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble()));
    // SmartDashboard.putNumber("Mod1 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(1).getEncoder().getAbsolutePosition().getValueAsDouble()));
    // SmartDashboard.putNumber("Mod2 new Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(2).getEncoder().getAbsolutePosition().getValueAsDouble()));
    // SmartDashboard.putNumber("Mod3 new Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(3).getEncoder().getAbsolutePosition().getValueAsDouble()));
    
    SmartDashboard.putNumber("PoseX", RobotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("PoseY", RobotContainer.drivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("Yaw", RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());

    if (kUseLimelight) {
      LimelightHomography.update(RobotContainer.drivetrain);
    }
  
  //AdvantageScope simulation
  Pose2d poseA = RobotContainer.drivetrain.getState().Pose;
  //System.out.println("Current pose: " + poseA);
  publisher.set(poseA);
  
}


@Override
public void disabledInit() {

}


  @Override
  public void disabledPeriodic() {
     NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(100);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    try {  
      RobotContainer.gyro.setYaw(DriverStation.getAlliance().get() == Alliance.Blue ? Math.PI: 0); // this is esentually directly from the external IMU since we barely trust vision angle
      RobotContainer.drivetrain.resetRotation(new Rotation2d(DriverStation.getAlliance().get() == Alliance.Blue ? Math.PI: 0));
    } 
    catch (Exception e) {
      System.out.print(e);
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(0);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
