// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchoCheese;

import java.util.Locale;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.BotchoCheese.Constants.RobotMap;


public class Robot extends TimedRobot {
  private static final long LIMELIGHT_DISABLED_THROTTLE = 100;
  private static final double SWERVE_OFFSET_PUBLISH_INTERVAL_SECONDS = 0.5;
  private static final CANBus CANIVORE_BUS = new CANBus(RobotMap.CANIVORE_CAN_BUS);

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final TalonFX[] swerveNeutralReportMotors = {
      new TalonFX(0, CANIVORE_BUS),
      new TalonFX(1, CANIVORE_BUS),
      new TalonFX(2, CANIVORE_BUS),
      new TalonFX(3, CANIVORE_BUS),
      new TalonFX(4, CANIVORE_BUS),
      new TalonFX(5, CANIVORE_BUS),
      new TalonFX(6, CANIVORE_BUS),
      new TalonFX(7, CANIVORE_BUS)
  };

  private final Field2d m_field = new Field2d();
  private double lastSwerveOffsetPublishSeconds = Double.NEGATIVE_INFINITY;
  private DoublePublisher poseXPublisher;
  private DoublePublisher poseYPublisher;
  private DoublePublisher poseHeadingDegPublisher;

  public Robot() {
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(RobotContainer.drivetrain.getPose());
  }

  @Override
  public void robotInit() {
    // Host dashboard layout files from /home/lvuser/deploy so Elastic can load them directly from the robot.
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    setLimelightThrottle(LIMELIGHT_DISABLED_THROTTLE);

    var poseTable = NetworkTableInstance.getDefault().getTable("Pose");
    poseXPublisher = poseTable.getDoubleTopic("X").publish();
    poseYPublisher = poseTable.getDoubleTopic("Y").publish();
    poseHeadingDegPublisher = poseTable.getDoubleTopic("HeadingDeg").publish();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    publishDashboardData();
    RobotContainer.drivetrain.visionUpdateFromLimelight();

    publishPoseData();
  }

  private void publishDashboardData() {
    if (!DriverStation.isDisabled()) {
      return;
    }

    double now = Timer.getFPGATimestamp();
    if (now - lastSwerveOffsetPublishSeconds < SWERVE_OFFSET_PUBLISH_INTERVAL_SECONDS) {
      return;
    }
    lastSwerveOffsetPublishSeconds = now;

    // Option 1 calibration flow (code-side offsets):
    // Keep CANcoder magnet offsets at zero and copy these values directly into
    // TunerConstants k*EncoderOffset as Rotations.of(<value>).
    double frontLeftAbsRot = RobotContainer.drivetrain.getModule(0).getEncoder().getAbsolutePosition().refresh().getValueAsDouble();
    double frontRightAbsRot = RobotContainer.drivetrain.getModule(1).getEncoder().getAbsolutePosition().refresh().getValueAsDouble();
    double backLeftAbsRot = RobotContainer.drivetrain.getModule(2).getEncoder().getAbsolutePosition().refresh().getValueAsDouble();
    double backRightAbsRot = RobotContainer.drivetrain.getModule(3).getEncoder().getAbsolutePosition().refresh().getValueAsDouble();

    SmartDashboard.putNumber("SwerveCal/FrontLeft OffsetToPaste (rot)", frontLeftAbsRot);
    SmartDashboard.putNumber("SwerveCal/FrontRight OffsetToPaste (rot)", frontRightAbsRot);
    SmartDashboard.putNumber("SwerveCal/BackLeft OffsetToPaste (rot)", backLeftAbsRot);
    SmartDashboard.putNumber("SwerveCal/BackRight OffsetToPaste (rot)", backRightAbsRot);
    SmartDashboard.putString(
        "SwerveCal/PasteLine FrontLeft",
        String.format(
            Locale.US,
            "private static final Angle kFrontLeftEncoderOffset = Rotations.of(%.12f);",
            frontLeftAbsRot));
    SmartDashboard.putString(
        "SwerveCal/PasteLine FrontRight",
        String.format(
            Locale.US,
            "private static final Angle kFrontRightEncoderOffset = Rotations.of(%.12f);",
            frontRightAbsRot));
    SmartDashboard.putString(
        "SwerveCal/PasteLine BackLeft",
        String.format(
            Locale.US,
            "private static final Angle kBackLeftEncoderOffset = Rotations.of(%.12f);",
            backLeftAbsRot));
    SmartDashboard.putString(
        "SwerveCal/PasteLine BackRight",
        String.format(
            Locale.US,
            "private static final Angle kBackRightEncoderOffset = Rotations.of(%.12f);",
            backRightAbsRot));
    SmartDashboard.putString(
        "SwerveCal/PasteBlock",
        String.format(
            Locale.US,
            "private static final Angle kFrontLeftEncoderOffset = Rotations.of(%.12f);%n"
                + "private static final Angle kFrontRightEncoderOffset = Rotations.of(%.12f);%n"
                + "private static final Angle kBackLeftEncoderOffset = Rotations.of(%.12f);%n"
                + "private static final Angle kBackRightEncoderOffset = Rotations.of(%.12f);",
            frontLeftAbsRot,
            frontRightAbsRot,
            backLeftAbsRot,
            backRightAbsRot));
    SmartDashboard.putString(
        "SwerveCal/Instruction",
        "Point wheels forward, then copy SwerveCal/* OffsetToPaste (rot) into TunerConstants k*EncoderOffset");

  }
  private void publishPoseData() {
    if (poseXPublisher == null || poseYPublisher == null || poseHeadingDegPublisher == null) {
      return;
    }

    var pose = RobotContainer.drivetrain.getPose();
    poseXPublisher.set(pose.getX());
    poseYPublisher.set(pose.getY());
    poseHeadingDegPublisher.set(pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Pose/OdomX", pose.getX());
    SmartDashboard.putNumber("Pose/OdomY", pose.getY());
    SmartDashboard.putNumber("Pose/OdomHeadingDeg", pose.getRotation().getDegrees());
  }

  private void setLimelightThrottle(long throttleValue) {
    NetworkTableInstance.getDefault()
        .getTable(RobotMap.LIMELIGHT_NAME)
        .getEntry("throttle_set")
        .setNumber(throttleValue);
  }

  private void setSwerveNeutralMode(NeutralModeValue neutralMode) {
    for (TalonFX motor : swerveNeutralReportMotors) {
      motor.setNeutralMode(neutralMode);
    }
  }


@Override
public void disabledInit() {
  setSwerveNeutralMode(NeutralModeValue.Coast);
  m_robotContainer.pivot.disableBrakeMode();
  setLimelightThrottle(LIMELIGHT_DISABLED_THROTTLE);
}


  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    setSwerveNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.pivot.enableBrakeMode();
    m_robotContainer.seedPoseFromSelectedAuto();

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
    setSwerveNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.pivot.enableBrakeMode();
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
    setSwerveNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.pivot.enableBrakeMode();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
