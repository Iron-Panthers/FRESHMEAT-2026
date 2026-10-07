package frc.robot.subsystems.shooter.shooter_omniwheels;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;

public class ShooterOmniwheels extends GenericRollers<ShooterOmniwheels.ShooterOmniwheelsTarget> {
    public enum ShooterOmniwheelsTarget implements GenericRollers.VelocityTarget{
        IDLE(-10, ShooterOmniwheelsConstants.SUPPLY_CURRENT_LIMIT),
        SHOOT(10, ShooterOmniwheelsConstants.SUPPLY_CURRENT_LIMIT);

        private double velocity;
        private double supplyCurrentLimit;

        private ShooterOmniwheelsTarget(double velocity, double supplyCurrentLimit) {
            this.velocity = velocity;
            this.supplyCurrentLimit = supplyCurrentLimit;
        }

        public double getVelocity() {
            return velocity;
        }

        public double getSupplyCurrentLimit() {
            return supplyCurrentLimit;
        }
    }

    public ShooterOmniwheels(GenericRollersIO io) {
        super("Shooter/Shooter Omniwheels", io);
    }

    public AngularVelocity getCurrentVelocity() {
        return Units.RadiansPerSecond.of(inputs.velocityRadsPerSec);
    }
}
