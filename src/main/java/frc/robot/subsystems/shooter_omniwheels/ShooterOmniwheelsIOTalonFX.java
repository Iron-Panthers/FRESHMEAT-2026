package frc.robot.subsystems.shooter_omniwheels;

import com.ctre.phoenix6.signals.InvertedValue;

import static frc.robot.subsystems.shooter_omniwheels.ShooterOmniwheelsConstants.*;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersConfiguration;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;

public class ShooterOmniwheelsIOTalonFX extends GenericRollersIOTalonFX implements ShooterOmniwheelsIO {
    public ShooterOmniwheelsIOTalonFX() {
        super(new GenericRollersConfiguration()
            .withID(SHOOTER_OMNIWHEELS_CONFIG.motorID1())
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withMotorDirection(
                SHOOTER_OMNIWHEELS_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withNeutralMode(SHOOTER_OMNIWHEELS_CONFIG.brake())
            .withReduction(SHOOTER_OMNIWHEELS_CONFIG.reduction())
            .withStatorCurrentLimit(STATOR_CURRENT_LIMIT));
    super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
    }
}
