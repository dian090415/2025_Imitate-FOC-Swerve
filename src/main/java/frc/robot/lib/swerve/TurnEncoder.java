package frc.robot.lib.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.fasterxml.jackson.core.io.schubfach.DoubleToDecimal;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class TurnEncoder extends CANcoder {
    public TurnEncoder(int port) {
        super(port);
    }

    public double getAbsolutePositionDegrees() {
        double position = Units.rotationsToDegrees(super.getAbsolutePosition().getValueAsDouble());
        position %= 360.0;
        return position > 180 ? position - 360 : position;
    }

    public double getAbsolutePositionRotations() {
        double position = super.getAbsolutePosition().getValueAsDouble();
        position %= 1.0;
        return position > 0.5 ? position - 1.0 : position;
        // return position;
    }
    public double getAbsolutePositionTicks(){
        double absolutePositionDegrees = super.getAbsolutePosition().getValueAsDouble(); // 例如 90 度
        double absoluteRotations = absolutePositionDegrees / 360.0;      // 0.25 轉
        double absoluteTicks = absoluteRotations * 2048.0;
        return absoluteTicks;

    }
}
