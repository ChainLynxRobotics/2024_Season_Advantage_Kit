package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private ClimberIO climberIO;
    private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO climberIO) {

    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);
    }
    
}
