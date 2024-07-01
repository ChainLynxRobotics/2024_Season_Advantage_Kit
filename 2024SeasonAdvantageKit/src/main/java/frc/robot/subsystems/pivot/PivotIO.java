package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface PivotIO {
    
    //need stuff for leader and follower motors, absolute encoder
    @AutoLog
    public static class PivotIOInputs {
        public MutableMeasure<Voltage> leaderVoltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Voltage> followerVoltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Current> leaderCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Current> followerCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Temperature> leaderTemperature;
        public MutableMeasure<Temperature> followerTemperature;
        public MutableMeasure<Velocity<Angle>> leaderVelocityRPM = MutableMeasure.zero(Units.RPM);
        public MutableMeasure<Velocity<Angle>> followerVelocityRPM = MutableMeasure.zero(Units.RPM);
        public MutableMeasure<Angle> absEncoderPosition = MutableMeasure.zero(Units.Radian);
        public boolean leaderMotorConnected = true;
        public boolean followerMotorConnected = true;
        public boolean absEncoderConnected = true;
        public double pGain;
        public double iGain;
        public double dGain;
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setSetpoint(MutableMeasure<Angle> setpoint) {}

    public default void setZeroPosition(MutableMeasure<Angle> position) {}

    public default void setPID(double p, double i, double d) {}

    public default void setFF(double ff) {}

    public default void disableSoftLimits() {}
}
