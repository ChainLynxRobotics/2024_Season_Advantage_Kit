package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public MutableMeasure<Voltage> climberLeaderVoltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Voltage> climberFollowerVoltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Current> climberLeaderCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Current> climberFollowerCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Temperature> climberLeaderTemperature;
        public MutableMeasure<Temperature> climberFollowerTemperature;
        public MutableMeasure<Angle> climberLeaderPosition = MutableMeasure.zero(Units.Rotations);
        public MutableMeasure<Angle> climberFollowerPosition = MutableMeasure.zero(Units.Rotations);
        public MutableMeasure<Velocity<Angle>> climberLeaderVelocity = MutableMeasure.zero(Units.RotationsPerSecond);
        public MutableMeasure<Velocity<Angle>> climberFollowerVelocity = MutableMeasure.zero(Units.RotationsPerSecond);
        public boolean leaderConnected = true;
        public boolean followerConnected = true;
    }

    default void updateInputs(ClimberIOInputs inputs) {}

    default void setSetpoint(Measure<Angle> setpoint) {}

    default void zeroEncoders() {}

    default void setPercentageMaxSpeed(double percentage, boolean followerInverted) {}

    default void disableSoftLimits() {}

}
