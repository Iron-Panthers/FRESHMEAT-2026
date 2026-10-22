package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

/** Purely for testing purposes :/
*/
public class ShootCommand extends Command {

    private ShooterController shooterController;
    private Drive swerve;

    public ShootCommand(ShooterController shooterController, Drive swerve) {
        this.swerve = swerve;
        this.shooterController = shooterController;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new AlignToShootCommand(swerve));
        shooterController.setTargetState(ShooterState.SHOOT);
    }
}
