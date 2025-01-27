package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private ShooterIO shooterIO;
    private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    public enum ShooterState {
        PIVOTING,
        SHOOTING,
        REST
    }
    private ShooterState curState = ShooterState.REST;
    MutableMeasure<Angle> angleSetpoint = shooterInputs.pivotAngle;
    MutableMeasure<Velocity<Angle>> rpm = MutableMeasure.zero(Units.RPM);


    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);

        switch (curState) {
            case PIVOTING:
                shooterIO.setAngle(angleSetpoint);
                curState = ShooterState.REST;
                break;
            case SHOOTING:
                shooterIO.setRPM(rpm);
                curState = ShooterState.REST;
                break;
            case REST:
            default:
                angleSetpoint = shooterInputs.pivotAngle;
                rpm = MutableMeasure.zero(Units.RPM);
                break;

        }
    }

    public void setFlywheelSpeed(MutableMeasure<Velocity<Angle>> setpoint) {
        rpm = setpoint;
    }

    public void setAngle(MutableMeasure<Angle> setpoint) {
        angleSetpoint = setpoint;
    }

    public boolean isAtAngleSetpoint() {
        return shooterInputs.atAngleSetpoint;
    }

    public boolean isAtFlywheelSetpoint() {
        return shooterInputs.atFlywheelSetpoint;
    }

    public void setShooterState(ShooterState state) {
        this.curState = state;
    }
}
