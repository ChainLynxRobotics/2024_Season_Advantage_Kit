package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO, AutoCloseable {
    private final FlywheelSim topRollers =
        new FlywheelSim(DCMotor.getNEO(1), 0.5, 0.025);
    private final FlywheelSim bottomRollers = 
        new FlywheelSim(DCMotor.getNEO(1), 0.5, 0.025);


    private final PIDController m_controller = new PIDController(0.4, 0, 0);
    private final Encoder m_encoder = new Encoder(0, 1);
    private final PWMSparkMax m_motor = new PWMSparkMax(Ports.kPivotMotorLeaderId);
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    private final SingleJointedArmSim pivotSim = 
        new SingleJointedArmSim(
            DCMotor.getNEO(2), 
            PivotConstants.kPivotReduction, 
            SingleJointedArmSim.estimateMOI(PivotConstants.kPivotLength, PivotConstants.kPivotMass), 
            PivotConstants.kPivotLength, 
            PivotConstants.kMinAngleRads, 
            PivotConstants.kMaxAngleRads, 
            true, 
            0);

    private final PIDController topRollerFeedback = new PIDController(1.5, 0, 0.5);
    private double appliedVolts = 0.0;
    private double setpointAngleRads = 0.0;
    public double setpointFlywheels = 0.0;

    private static final double ROLLER_RADIUS_METERS = 0.0762;

    public ShooterIOSim() {
        m_encoder.setDistancePerPulse(PivotConstants.kPivotEncoderDistPerPulse);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        topRollers.update(Robot.defaultPeriodSecs);
        bottomRollers.update(Robot.defaultPeriodSecs);
        pivotSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
        pivotSim.update(Robot.defaultPeriodSecs);

        m_encoderSim.setDistance(pivotSim.getAngleRads());
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));

        inputs.topFlywheelCurrent = MutableMeasure.ofBaseUnits(topRollers.getCurrentDrawAmps(), Units.Amps);
        inputs.bottomFlywheelCurrent = MutableMeasure.ofBaseUnits(bottomRollers.getCurrentDrawAmps(), Units.Amps);
        inputs.angleMotorLeaderCurrent = MutableMeasure.ofBaseUnits(pivotSim.getCurrentDrawAmps(), Units.Amps);
        inputs.angleMotorFollowerCurrent = inputs.angleMotorLeaderCurrent;
        inputs.topFlywheelVelocityRPM = MutableMeasure.ofBaseUnits(topRollers.getAngularVelocityRPM(), Units.RPM);
        inputs.bottomFlywheelVelocityRPM = MutableMeasure.ofBaseUnits(bottomRollers.getAngularVelocityRPM(), Units.RPM);
        inputs.atFlywheelSetpoint = Math.abs(topRollers.getAngularVelocityRPM() - setpointFlywheels) < ShooterConstants.kFlywheelError;

        inputs.pivotAngle = MutableMeasure.ofBaseUnits(pivotSim.getAngleRads(), Units.Radians);
        inputs.atAngleSetpoint = Math.abs(pivotSim.getAngleRads() - setpointAngleRads) < PivotConstants.kAngleError;
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        setpointAngleRads = angle.in(Units.Radians);
        double pidOutput =
            m_controller.calculate(
                m_encoder.getDistance(), setpointAngleRads);
        m_motor.setVoltage(pidOutput);
      }

    @Override
    public void setRPM(MutableMeasure<Velocity<Angle>> target) {
        setpointFlywheels = target.in(Units.RPM);
        // rotations per minute times circumference in meters gives meters traveled per minute and then divide by 60 for meters per second
        double topVelocityFromRPM = setpointFlywheels * ROLLER_RADIUS_METERS * 2 * Math.PI / 60;
        appliedVolts = topRollerFeedback.calculate(topRollers.getAngularVelocityRPM(), topVelocityFromRPM);
        appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);
        topRollers.setInputVoltage(appliedVolts);
        bottomRollers.setInputVoltage(-appliedVolts);
    }

    @Override
    public void close() throws Exception {
        m_motor.set(0);
    }
}
