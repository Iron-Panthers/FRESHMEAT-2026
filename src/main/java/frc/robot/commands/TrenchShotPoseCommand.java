package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

public class TrenchShotPoseCommand extends Command {
    private Drive swerve;
    private Rotation2d targetAngle;
    private Translation2d pointTarget;

    public TrenchShotPoseCommand(Drive swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        
        Pose2d underTrench = new Pose2d(12.5,0.8, new Rotation2d(1.59));
        
        Pose2d estimatedPose = underTrench;
        


    }
}

    