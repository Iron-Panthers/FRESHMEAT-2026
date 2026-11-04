package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.swerve.Drive;

public class AlignToShootCommand extends Command {

  private Drive swerve;
  private ShooterController shooterController;

  public AlignToShootCommand(Drive swerve, ShooterController shooterController) {
    this.swerve = swerve;
    this.shooterController = shooterController;
  }

  public void initialize() {
    swerve.setMovementScoped(true);
    shooterController.setAutoAimCommand(true);
  }

  public void execute() { // WORLDS CHANGE
    swerve.setMovementScoped(true);
    shooterController.setAutoAim(true);
  }

  public void end(boolean interrupted) {
    swerve.setMovementScoped(false);
    shooterController.setAutoAimCommand(false);
  }
}
