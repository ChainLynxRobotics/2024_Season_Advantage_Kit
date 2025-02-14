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
        public MutableMeasure<Voltage> angleMotorLeaderVoltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Voltage> angleMotorFollowerVoltage = MutableMeasure.zero(Units.Volts);
        public MutableMeasure<Current> topFlywheelCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Current> bottomFlywheelCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Current> angleMotorLeaderCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Current> angleMotorFollowerCurrent = MutableMeasure.zero(Units.Amps);
        public MutableMeasure<Temperature> topFlywheelTemperature;
        public MutableMeasure<Temperature> bottomFlywheelTemperature;
        public MutableMeasure<Temperature> angleMotorLeaderTemperature;
        public MutableMeasure<Temperature> angleMotorFollowerTemperature;
        public MutableMeasure<Velocity<Angle>> topFlywheelVelocityRPM = MutableMeasure.zero(Units.RPM);
        public MutableMeasure<Velocity<Angle>> bottomFlywheelVelocityRPM = MutableMeasure.zero(Units.RPM);
        public MutableMeasure<Angle> pivotAngle = MutableMeasure.zero(Units.Radians);
        public boolean atAngleSetpoint = false;
        public boolean atFlywheelSetpoint = false;
        public boolean topFlywheelMotorConnected = true;
        public boolean bottomFlywheelMotorConnected = true;
        public boolean angleMotorLeaderConnected = true;
        public boolean angleMotorFollowerConnected = true;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setRPM(MutableMeasure<Velocity<Angle>> target) {}

    public default void setAngle(MutableMeasure<Angle> angle) {}
}
