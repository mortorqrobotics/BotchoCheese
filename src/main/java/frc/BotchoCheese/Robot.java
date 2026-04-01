// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchoCheese;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private static final long LIMELIGHT_IDLE_THROTTLE = 100;

  public static LimelightHelpers limelight;
  public static LimelightHelpers limelightTwo;

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;
  private final Field2d m_field = new Field2d();

  public Robot() {
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(RobotContainer.drivetrain.getState().Pose);
  }

  @Override
  public void robotInit() {
    setLimelightThrottle(LIMELIGHT_IDLE_THROTTLE);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // SmartDashboard numeric telemetry is temporarily disabled.
    // publishDashboardData();

    if (kUseLimelight) {
      LimelightHomography.update(RobotContainer.drivetrain);
    }
  }

  private void setLimelightThrottle(long throttleValue) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(throttleValue);
  }


@Override
public void disabledInit() {
  m_robotContainer.pivot.disableBrakeMode();
  setLimelightThrottle(LIMELIGHT_IDLE_THROTTLE);
}


  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.pivot.enableBrakeMode();
    applyAllianceHeadingReference();

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
    m_robotContainer.pivot.enableBrakeMode();
    m_robotContainer.seedPoseFromSelectedAuto();
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
    m_robotContainer.pivot.enableBrakeMode();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  private void applyAllianceHeadingReference() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    double headingRad = alliance == Alliance.Red ? Math.PI : 0.0;
    double headingDeg = Math.toDegrees(headingRad);

    RobotContainer.gyro.setYaw(headingDeg);
    RobotContainer.drivetrain.resetRotation(new Rotation2d(headingRad));
  }
}
