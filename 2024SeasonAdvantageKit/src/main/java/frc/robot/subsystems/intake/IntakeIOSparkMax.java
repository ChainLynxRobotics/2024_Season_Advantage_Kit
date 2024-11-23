package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;

public class IntakeIOSparkMax implements IntakeIO {
    private CANSparkMax intakeMotor;
    
    public IntakeIOSparkMax(int id) {
        intakeMotor = new CANSparkMax(id, MotorType.kBrushless);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRPM = MutableMeasure.ofBaseUnits(intakeMotor.getEncoder().getVelocity(), Units.RPM);
        inputs.current = MutableMeasure.ofBaseUnits(intakeMotor.getOutputCurrent(), Units.Amps);
        inputs.voltage = MutableMeasure.ofBaseUnits(intakeMotor.getBusVoltage(), Units.Volts);
        inputs.temperature = MutableMeasure.ofBaseUnits(intakeMotor.getMotorTemperature(), Units.Celsius);
        inputs.motorConnected = intakeMotor.getFirmwareVersion() == 0 ? false: true;
    }

    @Override
    public void setVoltage(MutableMeasure<Voltage> voltage) {
        intakeMotor.setVoltage(voltage.in(Units.Volts));
    }
}
