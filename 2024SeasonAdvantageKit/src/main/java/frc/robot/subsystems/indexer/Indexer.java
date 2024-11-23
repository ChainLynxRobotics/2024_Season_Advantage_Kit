package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private IndexerIO indexerIO;
    private IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    @Override
    public void periodic() {
        indexerIO.updateInputs(indexerInputs);
        Logger.processInputs("Indexer", indexerInputs);
    }

    public void startFeedNote(boolean reverse) {
        indexerIO.setVoltage(MutableMeasure.ofBaseUnits(reverse ? -12 : 12, Units.Volts));
    }

    public void stopFeedNote() {
        indexerIO.setVoltage(MutableMeasure.ofBaseUnits(0, Units.Volts));
    }

    public boolean getLineBreak() {
        return indexerInputs.isLinebreakSensor;
    }
}
