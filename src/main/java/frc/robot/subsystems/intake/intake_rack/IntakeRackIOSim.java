package frc.robot.subsystems.intake.intake_rack;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;

public class IntakeRackIOSim extends GenericSuperstructureIOSim implements IntakeRackIO {
    public IntakeRackIOSim() {
        super(IntakeRackConstants.INTAKE_RACK_CONFIG.motorID());
    }

    @Override
    public void updateInputs(GenericSuperstructureIOInputs inputs) {}
}
