package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Coordinates high-level actions between 1 or more subsystems,  */
public class Superstructure {

    //holds current robot state
    public enum RobotState {
        DO_NOTHING
    }

    //periodically update currentState based on desiredState
    private RobotState currentState = RobotState.DO_NOTHING;
    private RobotState desiredState = RobotState.DO_NOTHING;

    
    public void setDesiredRobotState(RobotState desiredState) {
        this.desiredState = desiredState;
    }

    public Command setDesiredRobotStateCommand(RobotState desiredState) {
        return new InstantCommand(() -> setDesiredRobotState(desiredState));
    }
    
}
