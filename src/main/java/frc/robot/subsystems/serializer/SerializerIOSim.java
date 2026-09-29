package frc.robot.subsystems.serializer;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class SerializerIOSim extends GenericRollersIOSim implements SerializerIO {
    public SerializerIOSim() {
        super(
            SerializerConstants.SERIALIZER_CONFIG.motorID1(),
            SerializerConstants.CURRENT_LIMIT_AMPS,
            SerializerConstants.SERIALIZER_CONFIG.inverted(),
            SerializerConstants.SERIALIZER_CONFIG.brake(),
            SerializerConstants.SERIALIZER_CONFIG.reduction()
        );
    }
    @Override
    public void updateInputs(GenericRollersIOInputs inputs) {

    }
}