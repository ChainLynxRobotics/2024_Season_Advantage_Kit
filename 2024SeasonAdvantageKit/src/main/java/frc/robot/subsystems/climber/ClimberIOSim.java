package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
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

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

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

  public ClimberIOSim() {
    m_encoder.setDistancePerPulse(ElevatorConstants.kElevatorEncoderDistPerPulse);
  }


  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    m_motorSim.update(Constants.LOOP_PERIOD_SECS);
    m_elevatorSim.setInput(m_motorSim.getAngularVelocityRPM() * RobotController.getBatteryVoltage());

    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    inputs.climberLeaderCurrent = MutableMeasure.ofBaseUnits(m_motorSim.getCurrentDrawAmps(), Units.Amps);
    inputs.climberFollowerCurrent = inputs.climberLeaderCurrent;
    inputs.climberLeaderPosition = MutableMeasure.ofBaseUnits(m_motorSim.getAngularPositionRotations(), Units.Rotations);
    inputs.climberFollowerPosition = inputs.climberLeaderPosition;
    inputs.climberLeaderVelocity = MutableMeasure.ofBaseUnits(m_motorSim.getAngularVelocityRPM(), Units.RPM);
    inputs.climberFollowerVelocity = inputs.climberLeaderVelocity;
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
    m_motorSim.setInputVoltage(pidOutput + feedforwardOutput);
  }

  @Override
  public void setSetpoint(Measure<Angle> setpoint) {
    reachGoal(setpoint.in(Units.Rotation));
  }

  @Override
  public void setPercentageMaxSpeed(double percentage, boolean inverted) {
    m_motorSim.setInputVoltage(MathUtil.clamp(percentage*12, -12, 12));
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_controller.setGoal(0.0);
    m_motorSim.setInput(0.0);
  }


  @Override
  public void close() {
    m_encoder.close();
  }
}
