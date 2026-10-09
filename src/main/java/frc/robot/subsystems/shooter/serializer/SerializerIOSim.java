package frc.robot.subsystems.shooter.serializer;

import static frc.robot.subsystems.shooter.serializer.SerializerConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class SerializerIOSim extends GenericRollersIOSim implements SerializerIO {
    public SerializerIOSim() {
        super(
            SERIALIZER_CONFIG.motorID1(),
            CURRENT_LIMIT_AMPS,
            SERIALIZER_CONFIG.inverted(),
            SERIALIZER_CONFIG.brake(),
            SERIALIZER_CONFIG.reduction(),
            new RollerSim(2, 2, PHYSICAL_CONSTANTS.momentOfInertia(), 0.0267));
        setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
        talon.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void updateInputs(GenericRollersIOInputs inputs) {

    }
}