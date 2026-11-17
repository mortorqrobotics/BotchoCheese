package frc.BotchoCheese.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
// import static edu.wpi.first.units.Units.Second;
// import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Constants.TunerConstants.TunerSwerveDrivetrain;
import frc.BotchoCheese.Utils.LimelightHelpers;
import frc.BotchoCheese.Utils.LimelightHelpers.PoseEstimate;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // Simulation objects
    private VisionSystemSim visionSim;
    private PhotonCamera m_camera;
    private PhotonCameraSim cameraSim;

    /* Blue alliance sees forward as 180 degrees for this robot's current field-oriented convention */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Red alliance sees forward as 0 degrees for this robot's current field-oriented convention */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /** Minimalist AdvantageScope 2D/3D field pose publisher */
    private final StructPublisher<Pose2d> drivePose = NetworkTableInstance.getDefault()
        .getStructTopic("DriveState/Pose", Pose2d.struct).publish();

    /*
     * SysId characterization commands are intentionally disabled for production.
     * Re-enable this block when doing drivetrain system identification/tuning.
     */
    // /* Swerve requests to apply during SysId characterization */
    // private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    // private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    //
    // /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    // private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
    //         null,        // Use default timeout (10 s)
    //         null
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> setControl(m_translationCharacterization.withVolts(output)),
    //         null,
    //         this
    //     )
    // );
    //
    // /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    // private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(7), // Use dynamic voltage of 7 V
    //         null,        // Use default timeout (10 s)
    //         null
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         volts -> setControl(m_steerCharacterization.withVolts(volts)),
    //         null,
    //         this
    //     )
    // );
    //
    // /*
    //  * SysId routine for characterizing rotation.
    //  * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
    //  * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
    //  */
    // private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         /* This is in radians per second², but SysId only supports "volts per second" */
    //         Volts.of(Math.PI / 6).per(Second),
    //         /* This is in radians per second, but SysId only supports "volts" */
    //         Volts.of(Math.PI),
    //         null, // Use default timeout (10 s)
    //         null
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> {
    //             /* output is actually radians per second, but SysId only supports "volts" */
    //             setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
    //         },
    //         null,
    //         this
    //     )
    // );
    //
    // /* The SysId routine to test */
    // private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        initSimulation();
        configureAutoBuilder();
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        initSimulation();
        configureAutoBuilder();
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        initSimulation();
        configureAutoBuilder();
    }

    private void initSimulation() {
        if (!Utils.isSimulation()) return;

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(RobotMap.APRILTAG_FIELD_LAYOUT);

        m_camera = new PhotonCamera("camera");
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        cameraProp.setFPS(30);

        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
        cameraSim = new PhotonCameraSim(m_camera, cameraProp);
        visionSim.addCamera(cameraSim, robotToCam);

        startSimThread();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),
                    new PIDConstants(5, 0, 0)
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Returns the drivetrain estimator's current robot pose.
     */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /**
     * Adds a vision measurement to the drivetrain pose estimator.
     */
    public void visionUpdate(Pose2d pose, double timestampSeconds) {
        addVisionMeasurement(pose, timestampSeconds);
    }

    public void visionUpdateFromLimelight() {
        double headingDeg = getPigeon2().getYaw().getValueAsDouble();
        double yawRate = getPigeon2().getAngularVelocityZWorld().getValueAsDouble();

        LimelightHelpers.SetRobotOrientation(RobotMap.LIMELIGHT_NAME, headingDeg, yawRate, 0, 0, 0, 0);
        SmartDashboard.putBoolean("Pose/VisionAccepted", false);

        if (Math.abs(yawRate) > 360.0) {
            return; // Reject during high-speed turns
        }

        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotMap.LIMELIGHT_NAME);
        if (!LimelightHelpers.validPoseEstimate(estimate) || estimate.tagCount <= 0) {
            return;
        }

        Pose2d pose = estimate.pose;
        if (!isPoseInFieldBounds(pose)) {
            return;
        }

        // Dynamic standard deviation scaling based on distance and tag count
        double avgDistance = estimate.avgTagDist;
        double xyStdDev;

        if (estimate.tagCount >= 2) {
            xyStdDev = 0.3 + (avgDistance * 0.1);
        } else {
            if (avgDistance > 4.0) {
                return; // Reject single tag estimates beyond 4 meters
            }
            xyStdDev = 0.5 + (avgDistance * avgDistance * 0.1);
        }

        setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, Math.toRadians(9999999.0)));
        visionUpdate(pose, Utils.fpgaToCurrentTime(estimate.timestampSeconds));

        SmartDashboard.putBoolean("Pose/VisionAccepted", true);
        SmartDashboard.putNumber("Pose/VisionX", pose.getX());
        SmartDashboard.putNumber("Pose/VisionY", pose.getY());
        SmartDashboard.putNumber("Pose/VisionHeadingDeg", pose.getRotation().getDegrees());
    }

    private boolean isPoseInFieldBounds(Pose2d pose) {
        final double marginMeters = 0.25;
        double maxX = RobotMap.APRILTAG_FIELD_LAYOUT.getFieldLength();
        double maxY = RobotMap.APRILTAG_FIELD_LAYOUT.getFieldWidth();
        return pose.getX() >= -marginMeters
            && pose.getX() <= maxX + marginMeters
            && pose.getY() >= -marginMeters
            && pose.getY() <= maxY + marginMeters;
    }

    // /**
    //  * Runs the SysId Quasistatic test in the given direction for the routine
    //  * specified by {@link #m_sysIdRoutineToApply}.
    //  *
    //  * @param direction Direction of the SysId Quasistatic test
    //  * @return Command to run
    //  */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutineToApply.quasistatic(direction);
    // }
    //
    // /**
    //  * Runs the SysId Dynamic test in the given direction for the routine
    //  * specified by {@link #m_sysIdRoutineToApply}.
    //  *
    //  * @param direction Direction of the SysId Dynamic test
    //  * @return Command to run
    //  */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutineToApply.dynamic(direction);
    // }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        visionUpdateFromLimelight();
        drivePose.set(getPose());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());

            if (visionSim != null) {
                visionSim.update(getState().Pose);
            }
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}