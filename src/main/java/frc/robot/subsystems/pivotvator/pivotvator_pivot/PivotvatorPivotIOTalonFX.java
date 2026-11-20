package frc.robot.subsystems.pivotvator.pivotvator_pivot;

import static frc.robot.subsystems.pivotvator.pivotvator_pivot.PivotvatorPivotConstants.*;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;

public class PivotvatorPivotIOTalonFX extends GenericSuperstructureIOTalonFX
    implements PivotvatorPivotIO {
  public PivotvatorPivotIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(PIVOTVATOR_PIVOT_CONFIG.motorID())
            .withReduction(PIVOTVATOR_PIVOT_CONFIG.reduction())
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
            .withMotorDirection(PIVOTVATOR_PIVOT_CONFIG.motorDirection())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withAdditionalFollowerMotor(
                PIVOTVATOR_PIVOT_CONFIG.followerMotorID(),
                PIVOTVATOR_PIVOT_CONFIG.opposeMotor()
                    ? MotorAlignmentValue.Opposed
                    : MotorAlignmentValue.Aligned));

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
