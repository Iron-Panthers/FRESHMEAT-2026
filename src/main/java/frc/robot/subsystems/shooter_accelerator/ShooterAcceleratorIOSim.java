package frc.robot.subsystems.shooter_accelerator;

import static frc.robot.subsystems.shooter_accelerator.ShooterAcceleratorConstants.*;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class ShooterAcceleratorIOSim extends GenericRollersIOSim implements ShooterAcceleratorIO {
  public ShooterAcceleratorIOSim() {
    super(
        SHOOTER_ACCELERATOR.motorID1(),
        CURRENT_LIMIT_AMPS,
        SHOOTER_ACCELERATOR.inverted(),
        SHOOTER_ACCELERATOR.brake(),
        SHOOTER_ACCELERATOR.reduction(),
        new RollerSim(2, 2, PHYSICAL_CONSTANTS.momentOfInertia(), 0.0267));
    setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    
  }
}
