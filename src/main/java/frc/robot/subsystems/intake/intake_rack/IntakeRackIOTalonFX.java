package frc.robot.subsystems.intake.intake_rack;

import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;

public class IntakeRackIOTalonFX extends GenericSuperstructureIOTalonFX implements IntakeRackIO {
    public IntakeRackIOTalonFX() {
        super(new GenericSuperstructureConfiguration()
        .withID(IntakeRackConstants.INTAKE_RACK_CONFIG.motorID())
        .withReduction(IntakeRackConstants.INTAKE_RACK_CONFIG.reduction())
        .withSupplyCurrentLimit(IntakeRackConstants.CURRENT_LIMIT_AMPS)
        .withMotorDirection(IntakeRackConstants.INTAKE_RACK_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive));
    }
}
