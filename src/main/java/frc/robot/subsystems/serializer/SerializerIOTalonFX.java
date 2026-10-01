package frc.robot.subsystems.serializer;

import static frc.robot.subsystems.serializer.SerializerConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersConfiguration;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;

public class SerializerIOTalonFX extends GenericRollersIOTalonFX implements SerializerIO {
    public SerializerIOTalonFX() {
        super(new GenericRollersConfiguration()
        .withID(SerializerConstants.SERIALIZER_CONFIG.motorID1())
            .withMotorDirection(
                SerializerConstants.SERIALIZER_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withSupplyCurrentLimit(SerializerConstants.CURRENT_LIMIT_AMPS)
            .withReduction(SerializerConstants.SERIALIZER_CONFIG.reduction())
            .withNeutralMode(SerializerConstants.SERIALIZER_CONFIG.brake())
            .withAdditionalFollowerMotor(
                SerializerConstants.SERIALIZER_CONFIG.motorID2(), SerializerConstants.SERIALIZER_CONFIG.opposeMotor())
            .withStatorCurrentLimit(SerializerConstants.STATOR_CURRENT_LIMIT));
        super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
    }
}
