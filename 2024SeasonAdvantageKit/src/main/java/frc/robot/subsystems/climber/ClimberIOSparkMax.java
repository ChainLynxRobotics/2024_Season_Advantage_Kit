package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;

import java.util.Arrays;

public class ClimberIOSparkMax implements ClimberIO {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private SparkPIDController controller;

    public ClimberIOSparkMax(int leftId, int rightId) {
        leftMotor = new CANSparkMax(leftId, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightId, MotorType.kBrushless);
        controller = leftMotor.getPIDController();

        controller.setP(Constants.PIDConstants.kClimberP);
        controller.setI(Constants.PIDConstants.kClimberI);
        controller.setD(Constants.PIDConstants.kClimberD);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberLeaderVoltage = MutableMeasure.ofBaseUnits(leftMotor.getBusVoltage(), Units.Volts);
        inputs.climberFollowerVoltage = MutableMeasure.ofBaseUnits(rightMotor.getBusVoltage(), Units.Volts);
        inputs.climberLeaderCurrent = MutableMeasure.ofBaseUnits(leftMotor.getOutputCurrent(), Units.Amps);
        inputs.climberFollowerCurrent = MutableMeasure.ofBaseUnits(rightMotor.getOutputCurrent(), Units.Amps);
        inputs.climberLeaderTemperature = MutableMeasure.ofBaseUnits(leftMotor.getMotorTemperature(), Units.Fahrenheit);
        inputs.climberFollowerTemperature = MutableMeasure.ofBaseUnits(rightMotor.getMotorTemperature(), Units.Fahrenheit);

        inputs.climberLeaderPosition = MutableMeasure.ofBaseUnits(leftMotor.getEncoder().getPosition(), Units.Rotations);
        inputs.climberFollowerPosition = MutableMeasure.ofBaseUnits(rightMotor.getEncoder().getPosition(), Units.Rotations);
        inputs.climberLeaderVelocity = MutableMeasure.ofBaseUnits(leftMotor.getEncoder().getVelocity(), Units.RotationsPerSecond);
        inputs.climberFollowerVelocity = MutableMeasure.ofBaseUnits(rightMotor.getEncoder().getVelocity(), Units.RotationsPerSecond);

        inputs.leaderConnected = leftMotor.getFirmwareVersion() == 0 ? false : true;
        inputs.followerConnected = rightMotor.getFirmwareVersion() == 0 ? false : true;
    }


    @Override
    public void setSetpoint(Measure<Angle> setpoint) {
        if (MathUtil.isNear(setpoint.in(Units.Rotations), leftMotor.getEncoder().getPosition(), 0.5)) {
            leftMotor.set(0);
            rightMotor.set(0);
        }
        controller.setReference(setpoint.in(Units.Rotations), CANSparkBase.ControlType.kPosition);
    }

    @Override
    public void zeroEncoders() {
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
    }

    @Override
    public void setPercentageMaxSpeed(double percentage, boolean followerInverted, boolean leftOnly, boolean rightOnly) {
        int multiplier = followerInverted ? -1 : 1;
        if (!rightOnly) {
            leftMotor.set(-percentage);
        }
        if (!leftOnly) {
            rightMotor.set(percentage * multiplier);
        }
    }

    @Override
    public void disableSoftLimits() {
        Arrays.asList(leftMotor, rightMotor).forEach(motor -> {
            motor.setSoftLimit(SoftLimitDirection.kForward, 0);
            motor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        });
    }

}
