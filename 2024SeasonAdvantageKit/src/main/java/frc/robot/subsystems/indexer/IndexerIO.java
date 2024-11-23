package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public MutableMeasure<Velocity<Angle>> indexerVelocity;
        public MutableMeasure<Voltage> motorVoltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Current> motorCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Temperature> motorTemp;
        public boolean indexerMotorConnected = true;
        public boolean isLinebreakSensor = false;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setVoltage(MutableMeasure<Voltage> voltage) {}
}
