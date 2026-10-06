package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.generic_subsystems.rollers.*;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterRollers extends GenericRollers<ShooterRollers.ShooterRollerTarget>{
    public enum ShooterRollerTarget implements GenericRollers.VelocityTarget{
        SHOOT(1000),// replace with actual value
        IDLE(0);

         
        public double velocity;
        ShooterRollerTarget(int velocity) {
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

        public ShooterRollers(GenericRollersIO io){
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
