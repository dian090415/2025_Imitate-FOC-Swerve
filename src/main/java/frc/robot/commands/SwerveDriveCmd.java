package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpeed, ySpeed, rotation;

    public SwerveDriveCmd(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotation
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotation = rotation;
        this.addRequirements(this.swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        this.swerveSubsystem.driveSwerve(this.xSpeed.get(), this.ySpeed.get(), this.rotation.get(), SwerveConstants.gyroField);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}