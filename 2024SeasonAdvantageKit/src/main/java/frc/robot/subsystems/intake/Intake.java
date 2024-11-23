package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);
    }

    public void run(double motorOutput) {
        intakeIO.setVoltage(MutableMeasure.ofBaseUnits(12, Units.Volts));
      }
    
      public void stop() {
        intakeIO.setVoltage(MutableMeasure.ofBaseUnits(0, Units.Volts));
      }
}
