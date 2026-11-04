package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

public class AlignToShootCommand extends Command {

  private Drive swerve;

  public AlignToShootCommand(Drive swerve) {
    this.swerve = swerve;
  }

  @Override
  public void execute() {
    swerve.setMovementScoped(true);

    CommandScheduler.getInstance()
        .schedule(
            new AlignToPassCommand(swerve, (DriveConstants.RED_HUB_ORIGIN.toTranslation2d())));
  }

  public void end(boolean interrupted) {
    swerve.setMovementScoped(false);
  }
}
