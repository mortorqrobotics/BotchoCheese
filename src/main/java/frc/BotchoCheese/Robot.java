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
  private static final double MATCH_PUBLISH_INTERVAL_SECONDS = 0.1;
  private static final String MATCH_TIME_SECONDS_KEY = "Match/TimeSeconds";
  private static final String MATCH_MODE_KEY = "Match/Mode";
  private static final String MATCH_ALLIANCE_KEY = "Match/Alliance";
  private static final String MATCH_STATION_KEY = "Match/Station";
  private static final String MATCH_TYPE_KEY = "Match/Type";
  private static final String MATCH_NUMBER_KEY = "Match/Number";
  private static final String MATCH_REPLAY_NUMBER_KEY = "Match/Replay";
  private static final String MATCH_EVENT_NAME_KEY = "Match/EventName";
  private static final String MATCH_GAME_DATA_KEY = "Match/GameData";
  private static final String MATCH_REBUILT_SHIFT_KEY = "Match/RebuiltShift";
  private static final String MATCH_REBUILT_NEXT_SHIFT_KEY = "Match/RebuiltNextShift";
  private static final String MATCH_REBUILT_ACTIVE_FOR_US_KEY = "Match/RebuiltActiveForUs";
  private static final String MATCH_REBUILT_SHIFT_TIME_LEFT_KEY = "Match/RebuiltShiftTimeLeftSeconds";
  private static final String MATCH_COACH_SUMMARY_KEY = "Match/CoachSummary";
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
  private double lastMatchPublishSeconds = Double.NEGATIVE_INFINITY;
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

    publishMatchData();
    publishDashboardData();
    RobotContainer.drivetrain.visionUpdateFromLimelight();

    publishPoseData();
  }

  private void publishDashboardData() {
    m_robotContainer.updateAutoSelectionDashboard();
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

  private void publishMatchData() {
    double now = Timer.getFPGATimestamp();
    if (now - lastMatchPublishSeconds < MATCH_PUBLISH_INTERVAL_SECONDS) {
      return;
    }
    lastMatchPublishSeconds = now;

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    SmartDashboard.putNumber(MATCH_TIME_SECONDS_KEY, matchTime);
    SmartDashboard.putString(MATCH_MODE_KEY, getRobotModeSummary());
    SmartDashboard.putString(
        MATCH_ALLIANCE_KEY,
        DriverStation.getAlliance().map(alliance -> alliance.name()).orElse("Unknown"));
    SmartDashboard.putString(
        MATCH_STATION_KEY,
        DriverStation.getLocation().isPresent()
            ? Integer.toString(DriverStation.getLocation().getAsInt())
            : "Unknown");
    SmartDashboard.putString(MATCH_TYPE_KEY, DriverStation.getMatchType().name());
    SmartDashboard.putNumber(MATCH_NUMBER_KEY, DriverStation.getMatchNumber());
    SmartDashboard.putNumber(MATCH_REPLAY_NUMBER_KEY, DriverStation.getReplayNumber());
    SmartDashboard.putString(MATCH_EVENT_NAME_KEY, DriverStation.getEventName());
    SmartDashboard.putString(MATCH_GAME_DATA_KEY, gameData);
    String shiftLabel = getRebuiltShiftLabel(matchTime);
    String nextShiftLabel = getRebuiltNextShiftLabel(matchTime);
    boolean activeForUs = isRebuiltShiftActiveForOurAlliance(matchTime, gameData);
    double shiftTimeLeftSeconds = getRebuiltShiftTimeLeftSeconds(matchTime);

    SmartDashboard.putString(MATCH_REBUILT_SHIFT_KEY, shiftLabel);
    SmartDashboard.putString(MATCH_REBUILT_NEXT_SHIFT_KEY, nextShiftLabel);
    SmartDashboard.putBoolean(MATCH_REBUILT_ACTIVE_FOR_US_KEY, activeForUs);
    SmartDashboard.putNumber(MATCH_REBUILT_SHIFT_TIME_LEFT_KEY, shiftTimeLeftSeconds);
    SmartDashboard.putString(
        MATCH_COACH_SUMMARY_KEY,
        String.format(
            Locale.US,
            "%s -> %s | %s | %.1fs left",
            shiftLabel,
            nextShiftLabel,
            activeForUs ? "ACTIVE" : "INACTIVE",
            shiftTimeLeftSeconds));
  }

  private String getRobotModeSummary() {
    if (DriverStation.isEStopped()) {
      return "E-STOP";
    }
    if (DriverStation.isDisabled()) {
      return "DISABLED";
    }
    if (DriverStation.isAutonomous()) {
      return "AUTO";
    }
    if (DriverStation.isTeleop()) {
      return "TELEOP";
    }
    if (DriverStation.isTest()) {
      return "TEST";
    }
    return "UNKNOWN";
  }

  private String getRebuiltShiftLabel(double matchTime) {
    if (DriverStation.isEStopped()) {
      return "E-STOP";
    }
    if (DriverStation.isDisabled()) {
      return "DISABLED";
    }
    if (DriverStation.isAutonomous()) {
      return "AUTO";
    }
    if (!DriverStation.isTeleop()) {
      return "UNKNOWN";
    }
    if (matchTime > 130.0) {
      return "TRANSITION";
    }
    if (matchTime > 105.0) {
      return "SHIFT_1";
    }
    if (matchTime > 80.0) {
      return "SHIFT_2";
    }
    if (matchTime > 55.0) {
      return "SHIFT_3";
    }
    if (matchTime > 30.0) {
      return "SHIFT_4";
    }
    return "ENDGAME";
  }

  private boolean isRebuiltShiftActiveForOurAlliance(double matchTime, String gameData) {
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }
    if (matchTime > 130.0 || matchTime <= 30.0) {
      return true;
    }
    if (gameData.isEmpty() || DriverStation.getAlliance().isEmpty()) {
      return true;
    }

    boolean redInactiveFirst;
    char marker = gameData.charAt(0);
    if (marker == 'R') {
      redInactiveFirst = true;
    } else if (marker == 'B') {
      redInactiveFirst = false;
    } else {
      return true;
    }

    boolean shift1ActiveForUs = DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        ? !redInactiveFirst
        : redInactiveFirst;
    boolean oddShift = matchTime > 105.0 || (matchTime > 55.0 && matchTime <= 80.0);
    return oddShift ? shift1ActiveForUs : !shift1ActiveForUs;
  }

  private String getRebuiltNextShiftLabel(double matchTime) {
    if (DriverStation.isEStopped() || DriverStation.isDisabled()) {
      return "N/A";
    }
    if (DriverStation.isAutonomous()) {
      return "SHIFT_1";
    }
    if (!DriverStation.isTeleop()) {
      return "UNKNOWN";
    }
    if (matchTime > 130.0) {
      return "SHIFT_1";
    }
    if (matchTime > 105.0) {
      return "SHIFT_2";
    }
    if (matchTime > 80.0) {
      return "SHIFT_3";
    }
    if (matchTime > 55.0) {
      return "SHIFT_4";
    }
    if (matchTime > 30.0) {
      return "ENDGAME";
    }
    return "MATCH_END";
  }

  private double getRebuiltShiftTimeLeftSeconds(double matchTime) {
    if (DriverStation.isDisabled() || DriverStation.isEStopped()) {
      return 0.0;
    }
    if (DriverStation.isAutonomous()) {
      return Math.max(0.0, matchTime - 130.0);
    }
    if (!DriverStation.isTeleop()) {
      return 0.0;
    }
    if (matchTime > 130.0) {
      return matchTime - 130.0;
    }
    if (matchTime > 105.0) {
      return matchTime - 105.0;
    }
    if (matchTime > 80.0) {
      return matchTime - 80.0;
    }
    if (matchTime > 55.0) {
      return matchTime - 55.0;
    }
    if (matchTime > 30.0) {
      return matchTime - 30.0;
    }
    return Math.max(0.0, matchTime);
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
