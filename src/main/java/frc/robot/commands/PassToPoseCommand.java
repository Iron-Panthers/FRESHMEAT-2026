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

public class PassToPoseCommand extends Command {
    private Drive swerve;
    private Rotation2d targetAngle;
    private Translation2d pointTarget;

    public PassToPoseCommand(Drive swerve, Translation2d pointTarget) {
        this.swerve = swerve;
        this.pointTarget = pointTarget;
    }

    @Override
    public void initialize() {
        Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
        
        targetAngle = pointTarget.minus(estimatedPose.getTranslation()).getAngle();

        // I have zero idea why it needs to be flipped for red...
        if (RobotState.isAllianceRed()) targetAngle = targetAngle.plus(Rotation2d.kPi);

        swerve.setTargetHeading(targetAngle);
    }
}