package frc.robot.subsystems.intake.intake_rack;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;

public class IntakeRackIOTalonFX extends GenericSuperstructureIOTalonFX implements IntakeRackIO {
    public IntakeRackIOTalonFX() {
        super(new GenericSuperstructureConfiguration());
    }
}
