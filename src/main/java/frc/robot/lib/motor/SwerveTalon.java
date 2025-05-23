package frc.robot.lib.motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.SwerveConstants;

public class SwerveTalon extends TalonFX {
    private double gearRatio;

    public SwerveTalon(int motorPort, boolean reverse, double gearRatio) {
        super(motorPort);

        this.setNeutralMode(NeutralModeValue.Brake);
        this.setInverted(reverse);

        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(35.0)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(35.0)
            .withSupplyCurrentLowerLimit(40.0);

        TalonFXConfiguration FXConfig = new TalonFXConfiguration();
        FXConfig.CurrentLimits = currentConfig;
        this.getConfigurator().apply(FXConfig);

        this.gearRatio = gearRatio;
    }

    public void set(double speed) {
        super.set(speed);
    }

    public double getMotorVelocity() {
        return this.getVelocity().getValueAsDouble() * this.gearRatio * 2.0 * SwerveConstants.WHEEL_RADIUS * Math.PI;
    }

    public double getMotorPosition() {
        return this.getPosition().getValueAsDouble() * this.gearRatio * 2.0 * SwerveConstants.WHEEL_RADIUS * Math.PI;
    }

    public void setSelectedSensorPosition(double absoluteTicks) {
        this.setPosition(absoluteTicks);
    }

    public void config_kP(double kP) {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = kP;
        this.getConfigurator().apply(slot0);
    }

    public void config_kI(double kI) {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kI = kI;
        this.getConfigurator().apply(slot0);
    }

    public void config_kD(double kD) {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kD = kD;
        this.getConfigurator().apply(slot0);
    }
} 