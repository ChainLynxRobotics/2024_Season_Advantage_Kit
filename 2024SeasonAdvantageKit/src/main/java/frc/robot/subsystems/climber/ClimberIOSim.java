package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

//TODO fix constants to actual mechanism
public class ClimberIOSim implements ClimberIO, AutoCloseable {
  //use WPILib's ElevatorSim class to set max/min climber extensions (okay to simulate 2 stage climber as 1 stage elevator)
  private final DCMotor m_elevatorGearbox = DCMotor.getNeo550(2);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          ElevatorConstants.kElevatorKp,
          ElevatorConstants.kElevatorKi,
          ElevatorConstants.kElevatorKd,
          new TrapezoidProfile.Constraints(2.45, 2.45));
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kElevatorkS,
          ElevatorConstants.kElevatorkG,
          ElevatorConstants.kElevatorkV,
          ElevatorConstants.kElevatorkA);
  private final Encoder m_encoder =
      new Encoder(ElevatorConstants.kEncoderAChannel, ElevatorConstants.kEncoderBChannel);
  private final CANSparkMax m_motor = new CANSparkMax(ElevatorConstants.kMotorPort, MotorType.kBrushless);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kElevatorDrumRadius,
          ElevatorConstants.kMinElevatorHeightMeters,
          ElevatorConstants.kMaxElevatorHeightMeters,
          true,
          0);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  //TODO calculate inertia
  private final DCMotorSim m_motorSim = new DCMotorSim(m_elevatorGearbox, 40, DCMotor.getNeo550(2).stallTorqueNewtonMeters);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(5, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  public ClimberIOSim() {
    m_encoder.setDistancePerPulse(ElevatorConstants.kElevatorEncoderDistPerPulse);
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getAngularVelocityRPM() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberLeaderPosition = MutableMeasure.ofBaseUnits(m_motorSim.getAngularPositionRotations(), Units.Rotations);
    inputs.climberFollowerPosition = inputs.climberLeaderPosition;
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal) {
    m_controller.setGoal(goal);

    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_encoder.getDistance());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput + feedforwardOutput);
  }

  @Override
  public void setSetpoint(Measure<Angle> setpoint) {
    reachGoal(setpoint.in(Units.Rotation));
  }

  @Override
  public void setPercentageMaxSpeed(double percentage, boolean inverted) {
    m_motor.set(percentage);
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_controller.setGoal(0.0);
    m_motor.set(0.0);
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(m_encoder.getDistance());
  }

  @Override
  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech2d.close();
  }
}
