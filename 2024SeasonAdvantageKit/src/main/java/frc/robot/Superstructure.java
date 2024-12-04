package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/** Coordinates high-level actions between 1 or more subsystems,  */
public class Superstructure extends SubsystemBase {
    /* 
    //holds current robot state
    public enum RobotState {
        DO_NOTHING
    }

    //periodically update currentState based on desiredState
    private RobotState currentState = RobotState.DO_NOTHING;
    private RobotState desiredState = RobotState.DO_NOTHING;
    private RobotState previousState; //handle state transititions
    
    public void setDesiredRobotState(RobotState desiredState) {
        this.desiredState = desiredState;
    }

    public Command setDesiredRobotStateCommand(RobotState desiredState) {
        return new InstantCommand(() -> setDesiredRobotState(desiredState));
    }

    @Override 
    public void periodic() {
        Logger.recordOutput("DesiredSuperstate", desiredState);
        if (currentState != preState) {
            Logger.recordOutput("CurrentSuperstate", currentSuperState);
        }
    }
    */
    private Climber climber;
    private Indexer indexer;
    private Intake intake;
    private Shooter shooter;

    public Superstructure(Climber climber, Indexer indexer, Intake intake, Shooter shooter) {
        this.climber = climber;
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
    }

    public Command climb(boolean reverse, boolean leftOnly, boolean rightOnly) {
        return runOnce(
            () -> climber.moveClimbers(ClimberConstants.defaultClimbSpeed, reverse, leftOnly, rightOnly));
    }

    public Command stopClimb() {
        return runOnce(
            () -> climber.moveClimbers(0, false, false, false));
    }

    public Command intake(boolean reverse) {
        return runOnce(
            () -> intake.run(reverse));
    } 

    public Command stopIntake() {
        return runOnce(intake::stop);
    }
 
    public Command movePivot(MutableMeasure<Angle> setpoint) {
        return run(
            () -> shooter.setAngle(setpoint)) 
            .until(() -> shooter.isAtAngleSetpoint());
    } 

    public Command runFlywheels(MutableMeasure<Velocity<Angle>> rpm, boolean reverse) {
        return run(
            () -> shooter.runFlywheels(rpm, reverse))
            .finallyDo(() -> shooter.runFlywheels(MutableMeasure.ofBaseUnits(0, Units.RPM), reverse));
    } 

    public Command indexPiece(boolean reverse) {
        return runOnce(
            () -> indexer.startFeedNote(reverse));
    } 

    public Command stopIndexer() {
        return runOnce(indexer::stopFeedNote);
    }

    public Command scoreSpeaker(MutableMeasure<Angle> angleSetpoint, MutableMeasure<Velocity<Angle>> rpm) {
        return sequence(
                movePivot(angleSetpoint),
                intake(false).withTimeout(3).andThen(stopIntake()),
                runFlywheels(rpm, false).withTimeout(2),
                indexPiece(false),
                runFlywheels(rpm, false).withTimeout(2),
                stopIndexer()
            );
    } 
}
