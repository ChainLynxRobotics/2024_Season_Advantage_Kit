package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public MutableMeasure<Voltage> topFlywheelVoltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Voltage> bottomFlywheelVoltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Current> topFlywheelCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Current> bottomFlywheelCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Temperature> topFlywheelTemperature;
        public MutableMeasure<Temperature> bottomFlywheelTemperature;
        public MutableMeasure<Velocity<Angle>> topFlywheelVelocityRPM = MutableMeasure.zero(Units.RPM);
        public MutableMeasure<Velocity<Angle>> bottomFlywheelVelocityRPM = MutableMeasure.zero(Units.RPM);
        public boolean topFlywheelMotorConnected = true;
        public boolean bottomFlywheelMotorConnected = true;
        public double pGain;
        public double iGain;
        public double dGain;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVoltage(MutableMeasure<Voltage> topAppliedVolts, MutableMeasure<Voltage> bottomAppliedVolts) {}

    public default void setPID(double p, double i, double d) {}

    public default void setRPM(MutableMeasure<Velocity<Angle>> topTargetRPM, MutableMeasure<Velocity<Angle>> bottomTargetRPM) {}
}
