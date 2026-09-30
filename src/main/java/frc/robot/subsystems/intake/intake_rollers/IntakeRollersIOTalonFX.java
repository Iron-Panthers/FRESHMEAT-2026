package frc.robot.subsystems.intake.intake_rollers;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersConfiguration;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;

public class IntakeRollersIOTalonFX extends GenericRollersIOTalonFX implements IntakeRollersIO {
    public IntakeRollersIOTalonFX() {
        super(new GenericRollersConfiguration());
    }
}