package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import java.util.function.Supplier;

public class AlignToPoseCommand extends Command {
  private Drive swerve;
  private Supplier<Pose2d> approachPose;
  private Command poseAlignCommand;
  private boolean endOnAccurate = false;
  private Pose2d currentApproachPose;
  private boolean stayOnCurrentSide = false;

  public AlignToPoseCommand(Drive swerve, Supplier<Pose2d> approachPose) {
    this.swerve = swerve;
    this.approachPose =
        RobotState.isAllianceRed()
            ? () -> FlippingUtil.flipFieldPose(approachPose.get())
            : approachPose;

    addRequirements(swerve);
  }

  public AlignToPoseCommand(Drive swerve, Supplier<Pose2d> approachPose, boolean endOnAccurate) {
    this.swerve = swerve;
    this.approachPose = approachPose;
    this.endOnAccurate = endOnAccurate;
    this.stayOnCurrentSide = true;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    currentApproachPose = approachPose.get();
    swerve.setTargetPosition(currentApproachPose);
    try {
      poseAlignCommand =
          new VelocityClamp(swerve)
              .andThen(
                  RobotState.getInstance()
                      .getPathPlannerApproachPoseCommand(currentApproachPose, true));
      poseAlignCommand.initialize();
    } catch (Exception e) {
      e.printStackTrace();
      System.out.println("Already at target");
    }
  }

  @Override
  public void execute() {
    if (poseAlignCommand == null) return;
    if (!poseAlignCommand.isFinished()
        && RobotState.getInstance()
                .getEstimatedPose()
                .getTranslation()
                .getDistance(currentApproachPose.getTranslation())
            >= DriveConstants.PATHPLANNER_PID_OFFSET) {
      poseAlignCommand.execute();
    } else {
      if (!swerve.isPIDAutoAlign()) {
        swerve.setPIDAutoAlignTargetPosition(approachPose.get());
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (poseAlignCommand != null) poseAlignCommand.end(interrupted);
    swerve.clearTargetPositionController();
  }

  @Override
  public boolean isFinished() {
    if (endOnAccurate
        && currentApproachPose
                .getTranslation()
                .getDistance(RobotState.getInstance().getEstimatedPose().getTranslation())
            < 0.04
        && Math.abs(
                currentApproachPose
                    .getRotation()
                    .minus(RobotState.getInstance().getEstimatedPose().getRotation())
                    .getDegrees())
            < 2.5) {
      return true;
    }
    return false;
  }
}
