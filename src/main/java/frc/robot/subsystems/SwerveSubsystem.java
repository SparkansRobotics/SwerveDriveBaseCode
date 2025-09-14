package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;



public class SwerveSubsystem extends SubsystemBase {

    
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kFrontLeftCANCoderId,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kFrontRightCANCoderId,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kBackLeftCANCoderId,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kBackRightCANCoderId,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);




    public final GenericEntry sb_gyro, sb_alliance;
    public final Pigeon2 pidgey = new Pigeon2(17, "*");

    public final SwerveDrivePoseEstimator m_poseEstimator;
    public StructPublisher<Pose2d> publisher;
    public StructPublisher<Pose2d> limelightPublisher;

    private final Field2d m_field = new Field2d();

    public SwerveSubsystem() {
        // Create pose estimator
        m_poseEstimator = new SwerveDrivePoseEstimator(
            Constants.DriveConstants.kDriveKinematics, getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }, new Pose2d(0, 0, new Rotation2d()));

        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        // Path planner auto builder
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder
            AutoBuilder.configure(
                this::getPose, 
                this::resetPose, 
                this::getRobotRelativeSpeeds, 
                this::driveRobotRelative, 
                new PPHolonomicDriveController(
                    new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent() && alliance.get() != null) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }

        // AdvantageScope
        publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Robot Pose", Pose2d.struct).publish();

        limelightPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Robot Pose from Limelight", Pose2d.struct).publish();

        sb_gyro = Shuffleboard.getTab("Driver")
            .add("Gyro",0.0)
            .withPosition(0, 0)
            .withSize(4,3)
            .getEntry();

        sb_alliance = Shuffleboard.getTab("Driver")
            .add("Alliance OK", DriverStation.getAlliance().get() != null)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 6)
            .withSize(3,1)
            .getEntry();

        Shuffleboard.getTab("Driver")
            .add("Field", m_field)
            .withWidget(BuiltInWidgets.kField)
            .withPosition(11,0)
            .withSize(7,4);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public double getHeading() {
        // float angle = m_gyro.getPitch();
        // double dAngle = angle;
        return Math.IEEEremainder(DriveConstants.kGyroReversed ? -pidgey.getYaw(true).getValueAsDouble() : pidgey.getYaw(true).getValueAsDouble(), 360);
        // return (DriveConstants.kGyroReversed ? dAngle * -1 : dAngle);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPose(pose);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
            }
        );
    }

    public void driveRobotRelative(ChassisSpeeds speed ) {
        //ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speed, 0.02); is this needed?
        SwerveModuleState states[] = DriveConstants.kDriveKinematics.toSwerveModuleStates(speed);
        setModuleStates(states);
    }

    public void periodic(){
        // Update pose estimator
        m_poseEstimator.update(getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        });

        // Limelight w/ megatag 2
        LimelightHelpers.SetRobotOrientation("limelight-a", getPose().getRotation().getDegrees(), 0,0,0,0,0);
        LimelightHelpers.PoseEstimate mt2a = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-a");

        // Limelight A
        boolean doRejectUpdate = false;
        if (Math.abs(pidgey.getAngularVelocityYWorld().getValueAsDouble()) > 3000) {
            doRejectUpdate = true;
        }
        if (mt2a != null) {
            if (mt2a.tagCount == 0 || mt2a.avgTagDist > 3.5) {
                doRejectUpdate = true;
            }
        } else {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5 * mt2a.avgTagDist, 0.5 * mt2a.avgTagDist,9999999));
            m_poseEstimator.addVisionMeasurement(mt2a.pose, mt2a.timestampSeconds, VecBuilder.fill(0.0005* mt2a.avgTagDist * mt2a.avgTagDist, 0.0005 * mt2a.avgTagDist * mt2a.avgTagDist,9999999));
            limelightPublisher.set(mt2a.pose);
        }
        
        sb_gyro.setDouble(getHeading());
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        // Update advantagescope
        publisher.set(m_poseEstimator.getEstimatedPosition());
    }

    public Command zeroHeading() {
        System.out.println("Gyro Reset");
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().get() == null) {
                System.out.println("========== HEADING - ALLIANCE DETECTED AS NULL ==========");
                sb_alliance.setBoolean(false);
                return;
            }
            int angleAdjustment = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 0 : 180;
            // pidgey.reset();
            pidgey.setYaw(angleAdjustment);
        }); // Returns a command to be used on button press
    }

    public Command zeroCoords() {
        System.out.println("===== Coords Reset =====");
        return Commands.runOnce(() -> resetPose(new Pose2d(0,0,getRotation2d())));
    }

    public Command zeroEverything() {
        System.out.println("===== Zeroed Everything =====");
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().get() == null) {
                System.out.println("========== EVERYTHING - ALLIANCE DETECTED AS NULL ==========");
                sb_alliance.setBoolean(false);
                return;
            }
            int angleAdjustment = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 0 : 180;
            // pidgey.reset();
            pidgey.setYaw(angleAdjustment);
            m_poseEstimator.resetRotation(getRotation2d());
        });
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] newModuleStates = {
            frontLeft.getModuleState(),
            frontRight.getModuleState(),
            backLeft.getModuleState(),
            backRight.getModuleState()
        };
        return newModuleStates;
    }
}
