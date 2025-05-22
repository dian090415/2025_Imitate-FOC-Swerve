package frc.robot.joystick;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.SwerveConstants;

public class Driver extends XboxController {
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(SwerveConstants.MAX_ACCELERATION);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(SwerveConstants.MAX_ACCELERATION);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.MAX_ANGULAR_ACCELERATION);

    public Driver() {
        super(0);
    }

    public double getXDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftY(), SwerveConstants.DEAD_BAND) * 1.0 * this.getBrake();
        return this.xLimiter.calculate(speed);
    }

    public double getYDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftX(), SwerveConstants.DEAD_BAND) * 1.0 * this.getBrake();
        return this.yLimiter.calculate(speed);
    }

    public double getRotationSpeed() {
        double speed = -MathUtil.applyDeadband(this.getRightX(), SwerveConstants.DEAD_BAND) * 0.9 * this.getBrake();
        return this.rotationLimiter.calculate(speed);
    }

    public double getBrake() {
        return 1.0 - MathUtil.applyDeadband(this.getRightTriggerAxis(), SwerveConstants.DEAD_BAND) * 0.8;
    }
}
