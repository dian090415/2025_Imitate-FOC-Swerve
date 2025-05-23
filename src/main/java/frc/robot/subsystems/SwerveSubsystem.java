package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SwerveConstants;
import frc.robot.lib.helpers.IDashboardProvider;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.lib.swervesetpoint.ModuleLimits;
import frc.robot.lib.swervesetpoint.SwerveSetpoint;
import frc.robot.lib.swervesetpoint.SwerveSetpointGenerator;
import frc.robot.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase implements IDashboardProvider {


    private final SwerveDriveKinematics kinematics =
    new SwerveDriveKinematics(SwerveConstants.moduleLocations);

    private final SwerveSetpointGenerator swerveSetpointGenerator;


    private SwerveSetpoint currentSetpoint =
    new SwerveSetpoint(
        new ChassisSpeeds(),
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        });

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final AHRS gyro;
    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;

    // Publishers for tracking swerve drive poses
    private final StructPublisher<Pose2d> swerveNow = NetworkTableInstance.getDefault()
        .getStructTopic("AdvantageScope/RealSwervePose", Pose2d.struct).publish();

    /**
     * Initializes the SwerveSubsystem with modules, odometry, and pose estimator,
     * and configures dashboard data for swerve module positions and velocities.
     **/
    public SwerveSubsystem() {

        swerveSetpointGenerator =
        new SwerveSetpointGenerator(kinematics, SwerveConstants.moduleLocations);

        this.registerDashboard();
        this.frontLeft = new SwerveModule(
            2, 1, 9,
            false, true,
            "frontLeft"
        );
        this.frontRight = new SwerveModule(
            4, 3, 10,
            true, true,
            "frontRight"
        );
        this.backLeft = new SwerveModule(
            6, 5, 11,
            false, true,
            "backLeft"
        );
        this.backRight = new SwerveModule(
            8, 7, 12,
            true, true,
            "backRight"
        );
        this.gyro = new AHRS(NavXComType.kUSB1);
        this.odometry = new SwerveDriveOdometry(
            SwerveConstants.swerveDriveKinematics, this.getHeading(), this.getModulePosition()
        );
        this.field = new Field2d();
        this.poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.swerveDriveKinematics, Rotation2d.fromDegrees(-this.getGyroAngle()), this.getModulePosition(), new Pose2d());
        this.gyro.reset();

        // Put swerve to Elastic board.
        SmartDashboard.putData("ElasticSwerve/", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("FrontLeft Position", () -> frontLeft.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("FrontLeft Velocity", () -> frontLeft.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("FrontRight Position", () -> frontRight.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("FrontRight Velocity", () -> frontRight.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("BackLeft Position", () -> backLeft.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("BackLeft Velocity", () -> backLeft.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("BackRight Position", () -> backRight.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("BackRight Velocity", () -> backRight.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Heading", () -> getRotation().getRadians(), null);
            }
        });
    }

    /**
     * Periodic update method called regularly by the scheduler.
     * Updates odometry and pose estimator, publishes current pose and field info.
     */
    @Override
    public void periodic() {
        this.field.setRobotPose(this.getPose());
        this.swerveNow.set(this.getPose());
        this.odometry.update(this.getHeading(), this.getModulePosition());
        this.poseEstimator.update(this.getRotation(), this.getModulePosition());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot    Angular rate of the robot.
     * @param field  Whether the provided x and y speeds are relative to the
     *               field.
     */
    public void driveSwerve(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
            ChassisSpeeds targetSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.getHeading())
                : new ChassisSpeeds(xSpeed, ySpeed, rotation);
        
            // 直接呼叫 runVelocity，裡面會處理所有模組狀態計算與馬達控制
            this.runVelocity(targetSpeeds);
        }

        public void runVelocity(ChassisSpeeds speeds) {
            // (1) 離散化速度：把速度調整成固定頻率 (讓控制更平滑、避免跳動太快)
            ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, SwerveConstants.loopPeriodSecs);
        
            // (2) 計算未優化的模組狀態（四個輪子的速度跟角度）
            SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);
        
            // (3) 用 SwerveSetpointGenerator 生成平滑、符合限制的目標狀態
            currentSetpoint = swerveSetpointGenerator.generateSetpoint(
                SwerveConstants.moduleLimitsFree,
                currentSetpoint,
                discreteSpeeds,
                SwerveConstants.loopPeriodSecs
            );
            SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

            setModuleState(setpointStates);
        }
        

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleState(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED);
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    /**
     * Returns the current position of the swerve.
     *
     * @return The current position of the swerve.
     */
    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    /**
     * Returns the current state of the swerve.
     *
     * @return The current state of the swerve.
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            this.frontLeft.getState(),
            this.frontRight.getState(),
            this.backLeft.getState(),
            this.backRight.getState()
        };
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        if (Robot.isReal()) {
            return this.odometry.getPoseMeters();
        } else {
            return new Pose2d();  // 在非實機狀況可改成初始Pose或其他適當值
        }
    }

    public Pose2d getRobotToFieldPose(Translation2d robotToTagPose, Translation2d tagToFieldPose) {
        robotToTagPose = robotToTagPose.rotateBy(this.getHeading());
        Translation2d translation = tagToFieldPose.minus(robotToTagPose);
        return new Pose2d(translation, this.getHeading());
    }

    public void resetPoseFromAprilTag(Pose2d pose) {
        this.odometry.resetPose(pose);
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetPose(Pose2d pose) {
        if (Robot.isReal()) {
            this.gyro.reset();
            this.odometry.resetPosition(this.getHeading(), this.getModulePosition(), pose);
        }
    }

    public boolean isGyroConnected() {
        return this.gyro.isConnected();
    }

    public double getGyroAngle() {
        return -this.gyro.getAngle();
    }

    public Rotation2d getHeading() {
        return new Rotation2d(Units.degreesToRadians(this.getGyroAngle()));
    }

    public Rotation2d getRotation() {
        double angle = this.getGyroAngle();

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            angle += 180.0;
        }

        return new Rotation2d(Units.degreesToRadians(angle));
    }

    /**
     * Returns the chassis speed of the swerve
     * 
     * @return The chassis speed of the swerve
     */
    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.swerveDriveKinematics.toChassisSpeeds(this.getModuleStates());
    }

    /**
     * Resets the gyro to its default state.
     */
    public void resetGyro() {
        this.gyro.reset();
    }

    /**
     * Stops all swerve drive modules.
     */
    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putString("SwervePose", this.getPose().toString());
        SmartDashboard.putData("Field", this.field);
        SmartDashboard.putBoolean("Gyro Connect", this.isGyroConnected());
        SmartDashboard.putNumber("GyroAngle", this.getGyroAngle());
    }

    public void test(){
        
    }
}
