package frc.robot.subsystems.shooter.shooter_omniwheels;

import static frc.robot.subsystems.shooter.shooter_omniwheels.ShooterOmniwheelsConstants.*;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim.RollerSim;

public class ShooterOmniwheelsIOSim extends GenericRollersIOSim implements ShooterOmniwheelsIO {
    public ShooterOmniwheelsIOSim() {
        super(
            SHOOTER_OMNIWHEELS_CONFIG.motorID1(),
            SUPPLY_CURRENT_LIMIT,
            SHOOTER_OMNIWHEELS_CONFIG.inverted(),
            SHOOTER_OMNIWHEELS_CONFIG.brake(),
            SHOOTER_OMNIWHEELS_CONFIG.reduction(),
            new RollerSim(2, 2, PHYSICAL_CONSTANTS.momentOfInertia(), 0.0267));
    }

    public void updateInputs(GenericRollersIOInputs inputs) {}
}
