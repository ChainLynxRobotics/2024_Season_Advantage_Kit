package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private ClimberIO climberIO;
    private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber", climberInputs);
    }

    public void moveClimbers(double speed, boolean reverse, boolean leftOnly, boolean rightOnly) {
        int multiplier = reverse ? -1 : 1;
        climberIO.setPercentageMaxSpeed(speed * multiplier, false, leftOnly, rightOnly);
    }

    public void setSetpoint(Measure<Angle> setpoint) {
        climberIO.setSetpoint(setpoint);
    }
}
