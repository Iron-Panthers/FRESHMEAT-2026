package frc.robot.subsystems.shooter_omniwheels;

import static frc.robot.subsystems.shooter_omniwheels.ShooterOmniwheelsConstants.*;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class ShooterOmniwheelsIOSim extends GenericRollersIOSim implements ShooterOmniwheelsIO {
    public ShooterOmniwheelsIOSim() {
        super(
            SHOOTER_OMNIWHEELS_CONFIG.motorID1(),
            SUPPLY_CURRENT_LIMIT,
            SHOOTER_OMNIWHEELS_CONFIG.inverted(),
            SHOOTER_OMNIWHEELS_CONFIG.brake(),
            SHOOTER_OMNIWHEELS_CONFIG.reduction()
        );
    }

    public void updateInputs(GenericRollersIOInputs inputs) {}
}
