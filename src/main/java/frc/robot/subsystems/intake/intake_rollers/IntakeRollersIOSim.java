package frc.robot.subsystems.intake.intake_rollers;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class IntakeRollersIOSim extends GenericRollersIOSim implements IntakeRollersIO {
    public IntakeRollersIOSim() {
        super(
            IntakeRollersConstants.INTAKE_ROLLERS_CONFIG.motorID1(),
            IntakeRollersConstants.CURRENT_LIMIT_AMPS,
            IntakeRollersConstants.INTAKE_ROLLERS_CONFIG.inverted(),
            IntakeRollersConstants.INTAKE_ROLLERS_CONFIG.brake(),
            IntakeRollersConstants.INTAKE_ROLLERS_CONFIG.reduction());
    }

    public void updateInputs(GenericRollersIOInputs inputs) {}
}