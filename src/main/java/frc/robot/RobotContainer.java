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
			this.configBindings();
	}

	public void configButtonStationBindings() {
	}

	public void configBindings() {
		  this.driver.test()
			.onTrue(this.swerveSubsystem.testCommand());
		  this.driver.start()
			.onTrue(this.swerveSubsystem.startCommand());
		  this.driver.stop()
			.onTrue(this.swerveSubsystem.stopCommand());
	  }

	public Command getAutonomousCommand() {
		return null;
	}
}
