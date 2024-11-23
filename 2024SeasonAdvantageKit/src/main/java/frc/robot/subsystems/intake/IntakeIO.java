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
        public MutableMeasure<Voltage> voltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Current> current = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Temperature> temperature;
        public MutableMeasure<Velocity<Angle>> velocityRPM = MutableMeasure.zero(Units.RPM);
        public boolean motorConnected = true;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setVoltage(MutableMeasure<Voltage> voltage) {}

}
