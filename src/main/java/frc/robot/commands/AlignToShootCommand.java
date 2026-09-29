package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

public class AlignToShootCommand extends Command {

    private Drive swerve; 

    public AlignToShootCommand(Drive swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new AlignToPassCommand(swerve, (RobotState.isAllianceRed()) ?
            DriveConstants.RED_HUB_ORIGIN.toTranslation2d() :
            DriveConstants.BLUE_HUB_ORIGIN.toTranslation2d()));
    }
}
