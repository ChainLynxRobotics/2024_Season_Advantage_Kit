package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

/** Coordinates high-level actions between 1 or more subsystems,  */
public class Superstructure extends SubsystemBase {
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
        return runOnce(() -> shooter.setAngle(setpoint));
    } 

    public Command runFlywheels(MutableMeasure<Velocity<Angle>> rpm) {
        return runOnce(() -> shooter.setFlywheelSpeed(rpm));
    } 

    public Command stopFlywheels() {
        return runOnce(() -> shooter.setFlywheelSpeed(MutableMeasure.ofBaseUnits(0, Units.RPM)));
    }

    public Command indexPiece(boolean reverse) {
        return runOnce(() -> indexer.startFeedNote(reverse));
    } 

    public Command stopIndexer() {
        return runOnce(indexer::stopFeedNote);
    }

    public Command scoreSpeaker(MutableMeasure<Angle> angleSetpoint, MutableMeasure<Velocity<Angle>> rpm) {
        return sequence(
                movePivot(angleSetpoint), // set pivot setpoint
                runOnce(() -> shooter.setShooterState(ShooterState.PIVOTING)), // set state
                intake(false).withTimeout(3).andThen(stopIntake()),
                runFlywheels(rpm), //set flywheel setpoint
                runOnce(() -> shooter.setShooterState(ShooterState.SHOOTING)), // set state
                indexPiece(false),
                waitSeconds(2), 
                stopFlywheels(),
                stopIndexer()
            );
    } 
}
