package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class IntakeIOSim implements IntakeIO {
    private DCMotorSim intakeSim;
    private double curVoltage = 0.0;

    public IntakeIOSim() {
        intakeSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.1);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        intakeSim.update(Robot.defaultPeriodSecs);
        inputs.velocityRPM = MutableMeasure.ofBaseUnits(intakeSim.getAngularVelocityRPM(), Units.RPM);
        inputs.current = MutableMeasure.ofBaseUnits(intakeSim.getCurrentDrawAmps(), Units.Amps);
        inputs.voltage = MutableMeasure.ofBaseUnits(curVoltage, Units.Volts);
    }

    @Override
    public void setVoltage(MutableMeasure<Voltage> voltage) {
        curVoltage = voltage.in(Units.Volts);
        intakeSim.setInputVoltage(curVoltage);
    }
}
