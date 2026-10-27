package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drive;

public class LowPowerModeCommand extends Command {
    private Drive swerve;
    
    public LowPowerModeCommand(Drive swerve) {
        this.swerve = swerve;
    }

    // lowers supply current limit
    @Override
    public void initialize() {
        swerve.setLowPowerMode(true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setLowPowerMode(false);
    }
    
}
