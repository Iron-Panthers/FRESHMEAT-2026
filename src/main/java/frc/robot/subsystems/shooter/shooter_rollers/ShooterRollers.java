package frc.robot.subsystems.shooter.shooter_rollers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.generic_subsystems.rollers.*;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterRollers extends GenericRollers<ShooterRollers.ShooterRollersTarget>{
    public enum ShooterRollersTarget implements GenericRollers.VelocityTarget{
        SHOOT(1000, ShooterRollersConstants.CURRENT_LIMIT_AMPS),// replace with actual value
        IDLE(0, ShooterRollersConstants.CURRENT_LIMIT_AMPS);

         
        public double velocity;
        public double supplyCurrentLimit;
        ShooterRollersTarget(double velocity, double supplyCurrentLimit) {
            this.velocity = velocity;
        }
        public double getVelocity() {
            return getVelocity();
        }

        @Override
        public double getSupplyCurrentLimit() {
           return getSupplyCurrentLimit();
        }
    }

        public ShooterRollers(String test, GenericRollersIO io){
            super("Shooter/Shooter Flywheels", io);
        }
        public boolean reachedVelocityTarget() {
        if (super.useManualVelocity) {
        return Math.abs(super.inputs.velocityRadsPerSec - Units.rotationsToRadians(manualVelocityRPS))
            < 40;
        } else {
        if (velocityTarget == null) return false;
        return Math.abs(super.inputs.velocityRadsPerSec - Units.rotationsToRadians(velocityTarget.velocity))
            < 40;
        }
    }
}
