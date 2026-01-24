// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchedCode;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.Utils.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;

  private CANdle candle = new CANdle(0, "1515Canivore");

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //Module Offsets
    SmartDashboard.putNumber("Mod0 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("Mod1 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(1).getEncoder().getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("Mod2 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(2).getEncoder().getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("Mod3 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(3).getEncoder().getAbsolutePosition().getValueAsDouble()));
    
    SmartDashboard.putNumber("PoseX", RobotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("PoseY", RobotContainer.drivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("Yaw", RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());

    if (kUseLimelight) {
      var driveState = RobotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees(); // this is esentually directly from the external IMU since we barely trust vision angle
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      //assuming limelight starts facing red wall (MUST KNOW STARTING ANGLE) TODO
      LimelightHelpers.SetRobotOrientation(RobotMap.LIMELIGHT_NAME, headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotMap.LIMELIGHT_NAME);
      if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
        RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = true;
    config.stripType = LEDStripType.GRB;
    config.v5Enabled = true;
    config.vBatOutputMode = CANdle.VBatOutputMode.Modulated;
    config.brightnessScalar = 1;
    candle.configAllSettings(config, 100);
    candle.configLEDType(LEDStripType.GRB); //just added this after cd post
    candle.setLEDs(0, 0, 250, 100, 0, 10);
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