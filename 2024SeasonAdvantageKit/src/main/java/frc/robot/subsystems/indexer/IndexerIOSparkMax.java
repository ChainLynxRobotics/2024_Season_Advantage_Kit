package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOSparkMax implements IndexerIO {
    private CANSparkMax indexerMotor;
    private DigitalInput lineBreakSensor;
    
    public IndexerIOSparkMax(int id, int sensorId) {
        indexerMotor = new CANSparkMax(id, MotorType.kBrushless);
        lineBreakSensor = new DigitalInput(sensorId);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerVelocity = MutableMeasure.ofBaseUnits(indexerMotor.getEncoder().getVelocity(), Units.RPM);
        inputs.motorCurrent = MutableMeasure.ofBaseUnits(indexerMotor.getOutputCurrent(), Units.Amps);
        inputs.motorVoltage = MutableMeasure.ofBaseUnits(indexerMotor.getBusVoltage(), Units.Volts);
        inputs.motorTemp = MutableMeasure.ofBaseUnits(indexerMotor.getMotorTemperature(), Units.Celsius);
        inputs.isLinebreakSensor = lineBreakSensor.get();
        inputs.indexerMotorConnected = indexerMotor.getFirmwareVersion() == 0 ? false: true;
    }

    @Override
    public void setVoltage(MutableMeasure<Voltage> voltage) {
        indexerMotor.setVoltage(voltage.in(Units.Volts));
    }
}
