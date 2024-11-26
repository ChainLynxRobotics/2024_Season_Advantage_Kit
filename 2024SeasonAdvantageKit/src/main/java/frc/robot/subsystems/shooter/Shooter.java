package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private ShooterIO shooterIO;
    private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);
    }

    public void runFlywheels(MutableMeasure<Velocity<Angle>> setpoint, boolean reverse) {
        int multiplier = reverse ? -1 : 1;
        shooterIO.setRPM(setpoint.mut_times(multiplier));
    }

    public void setAngle(MutableMeasure<Angle> setpoint) {
        shooterIO.setAngle(setpoint);
    }

    public boolean isAtAngleSetpoint() {
        return shooterInputs.atAngleSetpoint;
    }

    public boolean isAtFlywheelSetpoint() {
        return shooterInputs.atFlywheelSetpoint;
    }
}
