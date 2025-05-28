package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.auto;
// import frc.robot.subsystems.sysidtest;

public class RobotContainer {
	public final Driver driver = new Driver();
	public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	// public final sysidtest sysidtest = new sysidtest();
	public final auto auto = new auto();

	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(new SwerveDriveCmd(
			this.swerveSubsystem,
			driver::getXDesiredSpeed, driver::getYDesiredSpeed, driver::getRotationSpeed));
			this.configBindings();
	}

	public void configButtonStationBindings() {
	// 	this.driver.test()
	// 	.onTrue(this.sysidtest.sysIdTest());
	//   this.driver.start()
	// 	.onTrue(this.sysidtest.startCommand());
	//   this.driver.stop()
	// 	.onTrue(this.sysidtest.stopCommand());
	}

	public void configBindings() {
	  }

	public Command getAutonomousCommand() {
		return this.auto.testauto();
	}
}
