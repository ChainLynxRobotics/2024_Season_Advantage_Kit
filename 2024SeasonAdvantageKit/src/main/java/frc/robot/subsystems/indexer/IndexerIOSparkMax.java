package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOSparkMax implements IndexerIO {
    private CANSparkMax indexerMotor;
    private DigitalInput lineBreakSensor;
    
    public IndexerIOSparkMax() {
        indexerMotor = new CANSparkMax(0, MotorType.kBrushless);
    }
}
