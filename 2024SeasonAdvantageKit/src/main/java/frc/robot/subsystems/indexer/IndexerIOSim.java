package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class IndexerIOSim implements IndexerIO {
    double curVoltage = 0.0;

    private final FlywheelSim indexerSim =
            new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025); 

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        indexerSim.update(Robot.defaultPeriodSecs);
        inputs.indexerVelocity = MutableMeasure.ofBaseUnits(indexerSim.getAngularVelocityRPM(), Units.RPM);
        inputs.motorCurrent = MutableMeasure.ofBaseUnits(indexerSim.getCurrentDrawAmps(), Units.Amps);
        inputs.motorVoltage = MutableMeasure.ofBaseUnits(curVoltage, Units.Volts);
    }

    @Override
    public void setVoltage(MutableMeasure<Voltage> voltage) {
        curVoltage = voltage.in(Units.Volts);
        indexerSim.setInputVoltage(curVoltage);
    }
}
