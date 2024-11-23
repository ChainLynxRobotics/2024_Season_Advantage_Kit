package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Ports;

public class ShooterIOSparkMax implements ShooterIO {
    
    private CANSparkMax m_angleMotorLeader;
    private CANSparkMax m_angleMotorFollower;
    private SparkPIDController m_anglePIDController;
    private AbsoluteEncoder m_angleEncoder;

    private CANSparkMax m_topFlywheelMotor;

    private CANSparkMax m_bottomFlywheelMotor;
    private RelativeEncoder m_topFlywheelEncoder;
    private RelativeEncoder m_bottomFlywheelEncoder;
    private SparkPIDController m_topFlywheelPIDController;

    private MutableMeasure<Velocity<Angle>> m_shooterSpeed;
    private MutableMeasure<Angle> m_shooterAngle;
    private MutableMeasure<Angle> m_targetAngle;

    public ShooterIOSparkMax(int topFlywheelId, int bottomFlywheelId, int angleLeaderId, int angleFollowerId) {
        m_topFlywheelMotor =
            new CANSparkMax(topFlywheelId, MotorType.kBrushless);
        m_topFlywheelEncoder = m_topFlywheelMotor.getEncoder();
        m_topFlywheelPIDController = m_topFlywheelMotor.getPIDController();
        m_bottomFlywheelMotor =
            new CANSparkMax(bottomFlywheelId, MotorType.kBrushless);
        m_bottomFlywheelMotor.follow(m_topFlywheelMotor, true);
        m_bottomFlywheelEncoder = m_bottomFlywheelMotor.getEncoder();

        m_topFlywheelPIDController.setP(ShooterConstants.kTopFlywheelP);
        m_topFlywheelPIDController.setI(ShooterConstants.kTopFlywheelI);
        m_topFlywheelPIDController.setD(ShooterConstants.kTopFlywheelD);
        m_topFlywheelPIDController.setFF(ShooterConstants.kTopFlywheelD);
        m_topFlywheelPIDController.setIZone(ShooterConstants.kTopFlywheelIZone);
        m_topFlywheelPIDController.setOutputRange(
            ShooterConstants.kTopFlywheelMinOutput,
            ShooterConstants.kTopFlywheelMaxOutput);

        // Angle
        m_angleMotorLeader =
            new CANSparkMax(Ports.kPivotMotorLeaderId, MotorType.kBrushless);
        m_angleMotorFollower =
            new CANSparkMax(Ports.kPivotMotorFollowerId, MotorType.kBrushless);
        // sets follower motor to run inversely to the leader
        m_angleMotorFollower.follow(m_angleMotorLeader, true);
        m_angleEncoder = m_angleMotorFollower.getAbsoluteEncoder();
        m_angleEncoder.setZeroOffset(28.6 / 360 * 160);

        m_anglePIDController = m_angleMotorLeader.getPIDController();
        m_anglePIDController.setP(PivotConstants.kAngleControlP);
        m_anglePIDController.setI(PivotConstants.kAngleControlI);
        m_anglePIDController.setD(PivotConstants.kAngleControlD);
        m_anglePIDController.setFF(PivotConstants.kAngleControlFF);
        m_anglePIDController.setIZone(PivotConstants.kAngleControlIZone);
        m_anglePIDController.setOutputRange(
            PivotConstants.kAngleControlMinOutput,
            PivotConstants.kAngleControlMaxOutput);

        m_shooterAngle = MutableMeasure.zero(Units.Revolutions);
        m_targetAngle = MutableMeasure.zero(Units.Rotations);
        m_shooterSpeed = MutableMeasure.zero(Units.RPM);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topFlywheelVoltage = MutableMeasure.ofBaseUnits(m_topFlywheelMotor.getBusVoltage(), Units.Volts);
        inputs.bottomFlywheelVoltage = MutableMeasure.ofBaseUnits(m_bottomFlywheelMotor.getBusVoltage(), Units.Volts);
        inputs.angleMotorLeaderVoltage = MutableMeasure.ofBaseUnits(m_angleMotorLeader.getBusVoltage(), Units.Volts);
        inputs.angleMotorFollowerVoltage = MutableMeasure.ofBaseUnits(m_angleMotorFollower.getBusVoltage(), Units.Volts);

        inputs.topFlywheelCurrent = MutableMeasure.ofBaseUnits(m_topFlywheelMotor.getOutputCurrent(), Units.Amps);
        inputs.bottomFlywheelCurrent = MutableMeasure.ofBaseUnits(m_bottomFlywheelMotor.getOutputCurrent(), Units.Amps);
        inputs.angleMotorLeaderCurrent = MutableMeasure.ofBaseUnits(m_angleMotorLeader.getOutputCurrent(), Units.Amps);
        inputs.angleMotorFollowerCurrent = MutableMeasure.ofBaseUnits(m_angleMotorFollower.getOutputCurrent(), Units.Amps);

        inputs.topFlywheelTemperature = MutableMeasure.ofBaseUnits(m_topFlywheelMotor.getMotorTemperature(), Units.Celsius);
        inputs.bottomFlywheelTemperature = MutableMeasure.ofBaseUnits(m_bottomFlywheelMotor.getMotorTemperature(), Units.Celsius);
        inputs.angleMotorLeaderTemperature = MutableMeasure.ofBaseUnits(m_angleMotorLeader.getMotorTemperature(), Units.Celsius);
        inputs.angleMotorFollowerTemperature = MutableMeasure.ofBaseUnits(m_angleMotorFollower.getMotorTemperature(), Units.Celsius);

        inputs.topFlywheelVelocityRPM = MutableMeasure.ofBaseUnits(m_topFlywheelEncoder.getVelocity(), Units.RPM);
        inputs.bottomFlywheelVelocityRPM = MutableMeasure.ofBaseUnits(m_bottomFlywheelEncoder.getVelocity(), Units.RPM);
        inputs.pivotAngle = MutableMeasure.ofBaseUnits(MutableMeasure.ofBaseUnits(m_angleEncoder.getPosition(), Units.Rotations).in(Units.Radians), Units.Radians);
        inputs.atAngleSetpoint = Math.abs(m_angleMotorLeader.getEncoder().getPosition() - m_shooterAngle.in(Units.Rotations)) < PivotConstants.kAngleError;
        inputs.atFlywheelSetpoint = Math.abs(m_topFlywheelEncoder.getPosition() - m_shooterSpeed.in(Units.RPM)) < ShooterConstants.kFlywheelError;

        inputs.topFlywheelMotorConnected = m_topFlywheelMotor.getFirmwareVersion() == 0 ? false: true;
        inputs.bottomFlywheelMotorConnected = m_bottomFlywheelMotor.getFirmwareVersion() == 0 ? false: true;
        inputs.angleMotorLeaderConnected = m_angleMotorLeader.getFirmwareVersion() == 0 ? false: true;
        inputs.angleMotorFollowerConnected = m_angleMotorFollower.getFirmwareVersion() == 0 ? false: true;

        double ff = Math.cos(m_shooterAngle.in(Units.Radians)) * PivotConstants.kAngleControlFF;
        setFF(ff);
    }

    @Override
    public void setRPM(MutableMeasure<Velocity<Angle>> target) {
        m_shooterSpeed.mut_replace(m_topFlywheelEncoder.getVelocity(), Units.RPM);
        m_topFlywheelPIDController.setReference(m_shooterSpeed.in(Units.RPM), CANSparkBase.ControlType.kVelocity);
    }

    @Override
    public void setAngle(MutableMeasure<Angle> angle) {
        m_targetAngle.mut_replace(angle);
        m_anglePIDController.setReference(
            angle.in(Units.Rotations), CANSparkBase.ControlType.kPosition);
    }

    public void setFF(double ff) {
        m_anglePIDController.setReference(
            m_targetAngle.in(Units.Rotations),
            CANSparkBase.ControlType.kPosition,
            0,
            ff,
            ArbFFUnits.kPercentOut);
    }
}
