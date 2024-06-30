package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public MutableMeasure<Voltage> topVoltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Voltage> bottomVoltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Current> topCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Current> bottomCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Temperature> topTemperature;
        public MutableMeasure<Temperature> bottomTemperature;
        public MutableMeasure<Velocity<Angle>> topVelocityRPM = MutableMeasure.zero(Units.RPM);
        public MutableMeasure<Velocity<Angle>> bottomVelocityRPM = MutableMeasure.zero(Units.RPM);
        public boolean topMotorConnected = true;
        public boolean bottomMotorConnected = true;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setVoltage(MutableMeasure<Voltage> topVoltage, MutableMeasure<Voltage> bottomVoltage) {}

}
