package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SwerveConstants;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.lib.motor.SwerveTalon;
import frc.robot.lib.swerve.TurnEncoder;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import frc.robot.SwerveConstants;


import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SwerveModule implements IDashboardProvider {
    private final SwerveTalon driveMotor;
    private final SwerveTalon turnMotor;

    private final TurnEncoder turnEncoder;

    private final VelocityDutyCycle driveRequest = new VelocityDutyCycle(0);
    private final PositionDutyCycle turnRequest = new PositionDutyCycle(0);

    private final PIDController turnPid;

    private final String motorName;

    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);// TODO

    private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);// TODO


    private final VoltageOut voltagRequire = new VoltageOut(0.0);
    private final SysIdRoutine sysIdRoutine;
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

        turnMotor.config_kP(0);
        turnMotor.config_kI(0);
        turnMotor.config_kD(0);

        driveMotor.config_kP(0);
        driveMotor.config_kI(0);
        driveMotor.config_kD(0);

        this.turnEncoder = new TurnEncoder(turnEncoderPort);

        this.turnPid = new PIDController(2.35, 0.0, 0.0);
        // Angle use -0.5 to 0.5, Radian use -Math.PI to Math.PI.
        this.turnPid.enableContinuousInput(-0.5, 0.5);

        this.motorName = motorName;
        this.resetEncoders();

        double absoluteTicks = turnEncoder.getAbsolutePositionTicks();
        this.turnMotor.setSelectedSensorPosition(absoluteTicks);

        sysIdRoutine =  new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(2).per(Second), Volts.of(5),
                    null, (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> {
                        this.turnMotor.setControl(voltagRequire.withOutput(volts.in(Volts)));
                    },
                    null,
                    null));
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
        desiredState.optimize(this.getState().angle);


        double wheelRPS = desiredState.speedMetersPerSecond / (2 * Math.PI * SwerveConstants.WHEEL_RADIUS);
        double motorRPS = wheelRPS * SwerveConstants.DRIVE_GEAR_RATIO;
    
        // Feedforward 計算（用線速度）
        double ffVolts = driveFF.calculate(desiredState.speedMetersPerSecond);
    
        // 計算轉向角度目標（轉換成編碼器 ticks）
        double targetRotations = desiredState.angle.getRotations(); // 0 ~ 1
        double targetTicks = targetRotations * 2048; // 常數定義為 2048
    
        // 使用 Phoenix 6 控制器命令發送器
        driveMotor.setControl(
            driveRequest
                .withVelocity(motorRPS)          // RPS（Phoenix 6 native）
                .withFeedForward(ffVolts / 12.0) // 相對電壓（0~1）
        );
    
        turnMotor.setControl(
            turnRequest
                .withPosition(targetTicks)       // 位置目標（ticks）
                .withFeedForward(ffVolts / 12.0)
        );
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.dynamic(direction);
    }

    public Command sysIdTest() {
        return Commands.sequence(
                this.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .raceWith(new WaitUntilCommand(2)),
                new WaitCommand(2),

                this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                        .raceWith(new WaitUntilCommand(2)),
                new WaitCommand(2),

                this.sysIdDynamic(SysIdRoutine.Direction.kForward)
                        .raceWith(new WaitUntilCommand(2)),
                new WaitCommand(2),

                this.sysIdDynamic(SysIdRoutine.Direction.kReverse)
                        .raceWith(new WaitUntilCommand(2)));
    }
}