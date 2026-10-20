package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;

public class AlignToTrenchCommand extends Command {
    private Drive swerve;

    public AlignToTrenchCommand(Drive swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new AlignToPoseCommand(swerve,
        (RobotState.isAllianceRed()) ?
        (RobotState.getInstance().getEstimatedPose().getY() < 4)
        ? new Pose2d(12.5,0.8, new Rotation2d(1.59))
        : new Pose2d(12.5, 7.2, new Rotation2d(4.69)) :
        (RobotState.getInstance().getEstimatedPose().getY() > 4)
        ? new Pose2d(12.5,0.8, new Rotation2d(1.59))
        : new Pose2d(12.5, 7.2, new Rotation2d(4.69)), true));
    }
}
