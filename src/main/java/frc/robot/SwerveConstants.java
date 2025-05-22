package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.swervesetpoint.ModuleLimits;

public final class SwerveConstants {
	public static final double TRACK_WIDTH = Units.inchesToMeters(20.75);
	public static final double TRACK_LENGTH = Units.inchesToMeters(20.75);
	public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

	public static final double MAX_SPEED = 2.0;
	public static final double MAX_ACCELERATION = 5.0;
	public static final double MAX_ANGULAR = 90.0;
	public static final double MAX_ANGULAR_ACCELERATION = 9.0;
	public static final double DRIVE_GEAR_RATIO = 300.0 / 49.0;
	public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
	public static final int MAX_VOLTAGE = 20;

	public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
		new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2),
		new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2),
		new Translation2d(-TRACK_LENGTH / 2,TRACK_WIDTH / 2),
		new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2)
	);

	public static final double DEAD_BAND = 0.05;
	public static final boolean gyroField = true;

	public static final Translation2d[] moduleLocations = new Translation2d[] {
		new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2),
		new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2),
		new Translation2d(-TRACK_LENGTH / 2,TRACK_WIDTH / 2),
		new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2)
	};

	public static final ModuleLimits moduleLimitsFree =
    new ModuleLimits(MAX_SPEED , MAX_ACCELERATION, Units.degreesToRadians(MAX_ANGULAR));

	public static final double loopPeriodSecs = 0.02;
}
