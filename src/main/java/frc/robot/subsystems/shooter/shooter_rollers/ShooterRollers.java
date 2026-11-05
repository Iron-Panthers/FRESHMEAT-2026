package frc.robot.subsystems.shooter.shooter_rollers;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.lib.generic_subsystems.rollers.*;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAcceleratorConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterRollers extends GenericRollers<ShooterRollers.ShooterRollersTarget> {
  public enum ShooterRollersTarget implements GenericRollers.VelocityTarget {
    SHOOT(42, ShooterRollersConstants.CURRENT_LIMIT_AMPS),
    SPIN_UP(52, ShooterRollersConstants.CURRENT_LIMIT_AMPS),
    CONSTANT_SHOOT(42, ShooterRollersConstants.CURRENT_LIMIT_AMPS),
    IDLE(5, ShooterRollersConstants.CURRENT_LIMIT_AMPS),
    SHOOT_0M(35, ShooterAcceleratorConstants.CURRENT_LIMIT_AMPS),
    SHOOT_1POINT9M(46, ShooterAcceleratorConstants.CURRENT_LIMIT_AMPS),
    TRENCH_DIVIDER(47.2, ShooterAcceleratorConstants.CURRENT_LIMIT_AMPS);

    public double velocity;
    public double supplyCurrentLimit;

    ShooterRollersTarget(double velocity, double supplyCurrentLimit) {
      this.velocity = velocity;
      this.supplyCurrentLimit = supplyCurrentLimit;
    }

    public double getVelocity() {
      return velocity;
    }

    @Override
    public double getSupplyCurrentLimit() {
      return supplyCurrentLimit;
    }
  }

  public ShooterRollers(GenericRollersIO io) {
    super("Shooter/Shooter Flywheels", io);
    setVelocityTarget(ShooterRollersTarget.SHOOT);
  }

  @AutoLogOutput(key = "Shooter/Shooter Rollers/Current Velocity")
  public LinearVelocity getCurrentVelocity() {
    return MetersPerSecond.of(
        Units.radiansToRotations(inputs.velocityRadsPerSec)
            * ShooterRollersConstants.PHYSICAL_CONSTANTS.circumferenceMeters());
  }

  public boolean reachedVelocityTarget() {
    if (super.useManualVelocity) {
      return Math.abs(super.inputs.velocityRadsPerSec - Units.rotationsToRadians(manualVelocityRPS))
          < 40;
    } else {
      if (velocityTarget == null) return false;
      return Math.abs(
              super.inputs.velocityRadsPerSec - Units.rotationsToRadians(velocityTarget.velocity))
          < 40;
    }
  }
}
