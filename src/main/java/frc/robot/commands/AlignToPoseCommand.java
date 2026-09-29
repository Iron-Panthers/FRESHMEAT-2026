package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;

public class AlignToPoseCommand extends Command{
    private Drive swerve;
    private Pose2d targetPose;
    private Command poseAlignCommand;
    private boolean endOnAccurate = false;
    private Pose2d currentApproachPose;

    public AlignToPoseCommand(Drive swerve, Pose2d targetPose){
        this.swerve = swerve;
        this.targetPose = 
            RobotState.isAllianceRed() 
            ? FlippingUtil.flipFieldPose(targetPose)
            : targetPose;

        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        currentApproachPose = targetPose;
        swerve.setTargetPosition(targetPose);
        try {
            poseAlignCommand =
                new VelocityClamp(swerve)
                    .andThen(
                        RobotState.getInstance()
                        .getPathPlannerApproachPoseCommand(currentApproachPose));
            poseAlignCommand.initialize();
        } catch(Exception e) {
            e.printStackTrace();
            System.out.println("Already at target");
        }
    }

    @Override
    public void execute(){
        if (!poseAlignCommand.isFinished() 
            && RobotState.getInstance()
                    .getEstimatedPose()
                    .getTranslation()
                    .getDistance(currentApproachPose.getTranslation())
                    >= DriveConstants.PATHPLANNER_PID_OFFSET) {
            poseAlignCommand.execute();
        } else {
            if (!swerve.isPIDAutoAlign()){
                swerve.setPIDAutoAlignTargetPosition(targetPose);
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        poseAlignCommand.end(interrupted);
        swerve.clearTargetPositionController();
    }

    @Override
    public boolean isFinished(){
        if (endOnAccurate 
            && currentApproachPose
                .getTranslation()
                .getDistance(RobotState.getInstance().getEstimatedPose().getTranslation())
                < 0.04 
            && Math.abs(
                currentApproachPose
                    .getRotation()
                    .minus(RobotState.getInstance()
                    .getEstimatedPose()
                    .getRotation())
                    .getDegrees())
                    < 2.5) {
                    return true;
                }
        return false;
    }
}
