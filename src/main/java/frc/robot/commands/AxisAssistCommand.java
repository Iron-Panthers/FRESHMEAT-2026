package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;

public class AxisAssistCommand extends Command {
    private Drive swerve;

    final double DISTANCE_FROM_WALL = 0.7;
    final double ANGLE_TOWARD_WALL = 1; // radians!!!
    /* ──────wall────────wall────────wall───────wall
           ┐  theta = ANGLE_TOWARD_WALL
          /
    ┌───┐/
    │bot│ ─────> theta = 0
    └───┘
    */

    public AxisAssistCommand(Drive swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        // may bee maybe be smart
        
        boolean closerToY0 = (RobotState.getInstance().getEstimatedPose().getTranslation().getY() < FlippingUtil.fieldSizeY / 2);

        final Distance yDistance = Meters.of(closerToY0 ? DISTANCE_FROM_WALL : FlippingUtil.fieldSizeY - DISTANCE_FROM_WALL);

        final Rotation2d targetAngle = new Rotation2d(
            (closerToY0 ^ RobotState.isAllianceRed() ? ANGLE_TOWARD_WALL : -ANGLE_TOWARD_WALL) + Math.PI);

        swerve.setAxisPosition(yDistance, targetAngle, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.clearTargetPositionController();
        swerve.setTeleopMode();
    }
}
