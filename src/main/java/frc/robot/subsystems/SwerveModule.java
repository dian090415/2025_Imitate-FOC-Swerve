package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveConstants;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.lib.motor.SwerveTalon;
import frc.robot.lib.swerve.TurnEncoder;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import frc.robot.SwerveConstants;

public class SwerveModule implements IDashboardProvider {
    private final SwerveTalon driveMotor;
    private final SwerveTalon turnMotor;

    private final TurnEncoder turnEncoder;

    private final PIDController turnPid;

    private final String motorName;

    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);// TODO

    private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);// TODO

    /**
     * Constructs a SwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * Swerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public SwerveModule(
            int driveMotorPort, int turnMotorPort, int turnEncoderPort,
            boolean driveMotorReverse, boolean turnMotorReverse,
            String motorName) {
        this.registerDashboard();

        this.driveMotor = new SwerveTalon(driveMotorPort, driveMotorReverse, SwerveConstants.DRIVE_GEAR_RATIO);
        this.turnMotor = new SwerveTalon(turnMotorPort, turnMotorReverse, SwerveConstants.TURN_GEAR_RATIO);

        turnMotor.config_kP(0, 1.0);
        turnMotor.config_kI(0, 0.0);
        turnMotor.config_kD(0, 0.1);

        driveMotor.config_kP(0, 1.0);
        driveMotor.config_kI(0, 0.0);
        driveMotor.config_kD(0, 0.1);

        this.turnEncoder = new TurnEncoder(turnEncoderPort);

        this.turnPid = new PIDController(2.35, 0.0, 0.0);
        // Angle use -0.5 to 0.5, Radian use -Math.PI to Math.PI.
        this.turnPid.enableContinuousInput(-0.5, 0.5);

        this.motorName = motorName;
        this.resetEncoders();
    }

    public void resetEncoders() {
        this.driveMotor.setPosition(0.0);
        this.turnMotor.setPosition(this.turnEncoder.getAbsolutePositionRotations());
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                this.driveMotor.getMotorVelocity(),
                Rotation2d.fromRotations(this.turnEncoder.getAbsolutePositionRotations()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                this.driveMotor.getMotorPosition(),
                Rotation2d.fromRotations(this.turnEncoder.getAbsolutePositionRotations()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }

        // 優化方向（避免不必要的大旋轉）
        desiredState = SwerveModuleState.optimize(desiredState, this.getState().angle);

        gettick();

        double wheelRPS = desiredState.speedMetersPerSecond / (2 * Math.PI * SwerveConstants.WHEEL_RADIUS);
        double motorRPS = wheelRPS * SwerveConstants.DRIVE_GEAR_RATIO;
        double targetVelocityRPS = motorRPS * 2048 / 10.0;
        double ffVolts = driveFF.calculate(desiredState.speedMetersPerSecond);

        // --- Turn: 使用 TalonFX 的 Position 模式控制 ---
        double targetRotations = desiredState.angle.getRotations(); // 0~1 range
        double targetTicks = targetRotations * 2048.0;

        driveTalon.set(ControlMode.Velocity, targetVelocityRPS,
                DemandType.ArbitraryFeedForward, ffVolts / 12.0);
        this.turnMotor.set(
                ControlMode.Position,
                targetTicks,
                DemandType.ArbitraryFeedForward,
                ffVolts / 12.0);
    }

    // Put Drive Velocity and Turn Position to SmartDashboard.
    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("SwerveState/" + this.motorName + " Drive Vel", this.driveMotor.getMotorVelocity());
        SmartDashboard.putNumber("SwerveState/" + this.motorName + " Drive Pos", this.driveMotor.getMotorPosition());
        SmartDashboard.putNumber("SwerveState/" + this.motorName + " Turn Pos",
                this.turnEncoder.getAbsolutePositionRotations());
    }

    // Stop module.
    public void stop() {
        this.driveMotor.set(0.0);
        this.turnMotor.set(0.0);
    }

    public void gettick() {
        double absoluteTicks = turnEncoder.getAbsolutePositionTicks(); // 例如 90 度
        this.turnMotor.setSelectedSensorPosition(absoluteTicks);
    }
}
