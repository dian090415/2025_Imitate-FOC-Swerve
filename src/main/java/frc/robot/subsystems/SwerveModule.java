package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class SwerveModule extends SubsystemBase implements IDashboardProvider {
    private final SwerveTalon driveMotor;
    private final SwerveTalon turnMotor;

    private final TurnEncoder turnEncoder;

    private final VelocityDutyCycle driveRequest = new VelocityDutyCycle(0);
    private final PositionDutyCycle turnRequest = new PositionDutyCycle(0);

    private final PIDController turnPid;

    private final String motorName;

    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);// TODO

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

        this.turnEncoder = new TurnEncoder(turnEncoderPort);

        this.turnPid = new PIDController(2.35, 0.0, 0.0);
        // Angle use -0.5 to 0.5, Radian use -Math.PI to Math.PI.
        this.turnPid.enableContinuousInput(-0.5, 0.5);

        this.motorName = motorName;
        this.resetEncoders();

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(2).per(Second), Volts.of(5),
                        null, (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> {
                            this.driveMotor.setControl(voltagRequire.withOutput(volts.in(Volts)));
                        },
                        null,
                        this));

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

        desiredState.optimize(this.getState().angle);

        // ---- Drive 馬達 ---- //
        double velocitySetpoint = desiredState.speedMetersPerSecond;

        // 計算 drive motor 的 feedforward（以 m/s 為輸入）
        double driveFFVolts = driveFF.calculate(velocitySetpoint);

        // 轉為比例輸出（假設你用 PercentOutput 模式）
        double driveFFOutput = driveFFVolts / 12.0;

        // 速度除以最大速度 → 做 PID 控制（若有）或直接用比例控制
        double driveOutput = velocitySetpoint / SwerveConstants.MAX_SPEED;

        // 合併 feedforward 與輸出（若只用比例控制就加總）
        // this.driveMotor.set(driveOutput + driveFFOutput);

        double turnOutput = this.turnPid.calculate(
            this.turnEncoder.getAbsolutePositionRotations(), desiredState.angle.getRotations());

            SmartDashboard.putNumber("desiredState.angle.getRotations()", desiredState.angle.getRotations());
            SmartDashboard.putNumber("encoder", this.turnEncoder.getAbsolutePositionRotations());
            

        // this.turnMotor.set(turnOutput);
    }


    // Put Drive Velocity and Turn Position to SmartDashboard.
    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("SwerveState/" + this.motorName + " Drive Vel",
                this.driveMotor.getMotorVelocity());
        SmartDashboard.putNumber("SwerveState/" + this.motorName + " Drive Pos",
                this.driveMotor.getMotorPosition());
        SmartDashboard.putNumber("SwerveState/" + this.motorName + " Turn Pos",
                this.turnEncoder.getAbsolutePositionRotations());
        SmartDashboard.putNumber("turnEncoder", this.turnEncoder.getAbsolutePositionRotations());
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