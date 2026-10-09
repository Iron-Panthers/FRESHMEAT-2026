package frc.robot.subsystems.serializer;

import static frc.robot.subsystems.serializer.SerializerConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class SerializerIOSim extends GenericRollersIOSim implements SerializerIO {
    public SerializerIOSim() {
        super(
            SerializerConstants.SERIALIZER_CONFIG.motorID1(),
            SerializerConstants.CURRENT_LIMIT_AMPS,
            SerializerConstants.SERIALIZER_CONFIG.inverted(),
            SerializerConstants.SERIALIZER_CONFIG.brake(),
            SerializerConstants.SERIALIZER_CONFIG.reduction(),
            new RollerSim(1, 2, PHYSICAL_CONSTANTS.momentOfIntertia(), 0.2237)
        );
        setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
        talon.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void updateInputs(GenericRollersIOInputs inputs) {

    }
}