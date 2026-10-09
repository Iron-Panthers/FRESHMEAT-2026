<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/shooter/ShooterRollers/ShooterRollers.java
package frc.robot.subsystems.shooter.ShooterRollers;
========
package frc.robot.subsystems.shooter.shooter_rollers;
>>>>>>>> feat/shooter-merge:src/main/java/frc/robot/subsystems/shooter/shooter_rollers/ShooterRollers.java

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.generic_subsystems.rollers.*;
import org.littletonrobotics.junction.AutoLogOutput;

<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/shooter/ShooterRollers/ShooterRollers.java
import static edu.wpi.first.units.Units.MetersPerSecond;

public class ShooterRollers extends GenericRollers<ShooterRollers.ShooterRollerTarget>{
    public enum ShooterRollerTarget implements GenericRollers.VelocityTarget{
        SHOOT(40),// replace with actual value
        IDLE(0),
        INTAKE(0);// Do we need this?

         
        public double velocity;

        ShooterRollerTarget(int velocity) {
            this.velocity = velocity;
        }
        
        public double getVelocity() {
            return velocity;
========
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
>>>>>>>> feat/shooter-merge:src/main/java/frc/robot/subsystems/shooter/shooter_rollers/ShooterRollers.java
        }

        @Override
        public double getSupplyCurrentLimit() {
           return getSupplyCurrentLimit();
        }
    }


    @AutoLogOutput(key = "Shooter/Shooter Rollers/Current Velocity")
    public LinearVelocity getCurrentVelocity() {
        return MetersPerSecond.of(
            Units.radiansToRotations(inputs.velocityRadsPerSec)
                * ShooterRollersConstants.PHYSICAL_CONSTANTS.circumferenceMeters());
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
