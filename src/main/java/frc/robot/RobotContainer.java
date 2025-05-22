package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	public final Driver driver = new Driver();
	public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(new SwerveDriveCmd(
			this.swerveSubsystem,
			driver::getXDesiredSpeed, driver::getYDesiredSpeed, driver::getRotationSpeed));
	}

	public void configButtonStationBindings() {
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
