// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchoCheese;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.BotchoCheese.Commands.LimelightHomography;
import frc.BotchoCheese.Utils.DebugLog;
import frc.BotchoCheese.Utils.LimelightHelpers;


public class Robot extends TimedRobot {
  private static final long LIMELIGHT_IDLE_THROTTLE = 100;

  public static LimelightHelpers limelight;
  public static LimelightHelpers limelightTwo;

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;
  private final Field2d m_field = new Field2d();
  private double lastCanHealthLogSeconds = 0.0;
  private DoublePublisher canUtilizationPublisher;
  private IntegerPublisher canBusOffPublisher;
  private IntegerPublisher canTxFullPublisher;
  private IntegerPublisher canRxErrorPublisher;
  private IntegerPublisher canTxErrorPublisher;

  public Robot() {
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(RobotContainer.drivetrain.getState().Pose);
  }

  @Override
  public void robotInit() {
    setLimelightThrottle(LIMELIGHT_IDLE_THROTTLE);
    if (DebugLog.DEBUG) {
      SignalLogger.setPath("logs");
      SignalLogger.start();
      DebugLog.info("CTRE SignalLogger enabled (debug mode).");

      var debugTable = NetworkTableInstance.getDefault().getTable("Debug");
      canUtilizationPublisher = debugTable.getDoubleTopic("CAN/UtilizationPct").publish();
      canBusOffPublisher = debugTable.getIntegerTopic("CAN/BusOffCount").publish();
      canTxFullPublisher = debugTable.getIntegerTopic("CAN/TxFullCount").publish();
      canRxErrorPublisher = debugTable.getIntegerTopic("CAN/RxErrorCount").publish();
      canTxErrorPublisher = debugTable.getIntegerTopic("CAN/TxErrorCount").publish();

      CommandScheduler.getInstance().onCommandInitialize(
          command -> DebugLog.debug("[CMD INIT] " + command.getName()));
      CommandScheduler.getInstance().onCommandFinish(
          command -> DebugLog.debug("[CMD END] " + command.getName()));
      CommandScheduler.getInstance().onCommandInterrupt(
          command -> DebugLog.debug("[CMD INTERRUPT] " + command.getName()));
    }
    DebugLog.info("Startup complete (vision processing disabled, minimal telemetry mode).");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // SmartDashboard numeric telemetry is temporarily disabled.
    // publishDashboardData();

    if (kUseLimelight) {
      LimelightHomography.update(RobotContainer.drivetrain);
    }

    if (DebugLog.DEBUG) {
      logCanHealthSnapshot();
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
    DebugLog.info("Autonomous init");

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
    DebugLog.info("Teleop init");
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

  private void logCanHealthSnapshot() {
    double now = Timer.getFPGATimestamp();
    if (now - lastCanHealthLogSeconds < 1.0) {
      return;
    }
    lastCanHealthLogSeconds = now;

    var canStatus = RobotController.getCANStatus();
    DebugLog.debug(
        String.format(
            "[CAN] util=%.1f%% busOff=%d txFull=%d rxErr=%d txErr=%d",
            canStatus.percentBusUtilization * 100.0,
            canStatus.busOffCount,
            canStatus.txFullCount,
            canStatus.receiveErrorCount,
            canStatus.transmitErrorCount));

    if (canUtilizationPublisher != null) {
      canUtilizationPublisher.set(canStatus.percentBusUtilization * 100.0);
    }
    if (canBusOffPublisher != null) {
      canBusOffPublisher.set(canStatus.busOffCount);
    }
    if (canTxFullPublisher != null) {
      canTxFullPublisher.set(canStatus.txFullCount);
    }
    if (canRxErrorPublisher != null) {
      canRxErrorPublisher.set(canStatus.receiveErrorCount);
    }
    if (canTxErrorPublisher != null) {
      canTxErrorPublisher.set(canStatus.transmitErrorCount);
    }
  }

  private void applyAllianceHeadingReference() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    double headingRad = alliance == Alliance.Red ? Math.PI : 0.0;
    double headingDeg = Math.toDegrees(headingRad);

    RobotContainer.gyro.setYaw(headingDeg);
    RobotContainer.drivetrain.resetRotation(new Rotation2d(headingRad));
  }
}
