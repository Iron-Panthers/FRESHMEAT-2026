package frc.robot.subsystems.pivotvater.pivotvater_pivot;

import static frc.robot.subsystems.pivotvater.pivotvater_pivot.PivotvaterPivotConstants.*;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;

public class PivotvaterPivotIOTalonFX extends GenericSuperstructureIOTalonFX
    implements PivotvaterPivotIO {
  public PivotvaterPivotIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(PIVOTVATER_CONFIG.motorID())
            .withReduction(PIVOTVATER_CONFIG.reduction())
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
            .withMotorDirection(PIVOTVATER_CONFIG.motorDirection())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET));

    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        MOTION_MAGIC_CONFIG.acceleration(),
        MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        GRAVITY_TYPE);
  }
}
