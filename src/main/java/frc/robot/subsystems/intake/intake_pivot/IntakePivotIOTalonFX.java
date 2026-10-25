package frc.robot.subsystems.intake.intake_pivot;

import static frc.robot.subsystems.intake.intake_pivot.IntakePivotConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;

public class IntakePivotIOTalonFX extends GenericSuperstructureIOTalonFX implements IntakePivotIO {
    public IntakePivotIOTalonFX() {
        super(new GenericSuperstructureConfiguration()
        .withID(INTAKE_PIVOT_CONFIG.motorID())
        .withReduction(INTAKE_PIVOT_CONFIG.reduction())
        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
        .withMotorDirection(INTAKE_PIVOT_CONFIG.motorDirection())
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
