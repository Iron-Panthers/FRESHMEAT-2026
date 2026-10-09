package frc.robot.subsystems.intake.intake_rollers;

import com.ctre.phoenix6.sim.TalonFXSimState;

import static frc.robot.subsystems.intake.intake_rollers.IntakeRollersConstants.*;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class IntakeRollersIOSim extends GenericRollersIOSim implements IntakeRollersIO {
    public IntakeRollersIOSim() {
        super(
            INTAKE_ROLLERS_CONFIG.motorID1(),
            CURRENT_LIMIT_AMPS,
            INTAKE_ROLLERS_CONFIG.inverted(),
            INTAKE_ROLLERS_CONFIG.brake(),
            INTAKE_ROLLERS_CONFIG.reduction(),
            new RollerSim(1, 2, PHYSICAL_CONSTANTS.momentOfInertia(), 0.1068));
        setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
        talon.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    public void updateInputs(GenericRollersIOInputs inputs) {}
}