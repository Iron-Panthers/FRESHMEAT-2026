package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elastic_updater.ElasticUpdater;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.intake.intake_pivot.IntakePivot;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import java.util.function.Supplier;

/**
 * Like ShootCommandFactory, but replaces the intake-cycling phase with stator-current-adaptive
 * velocity control: the pivot moves toward STOW, slowing down when stator current exceeds {@code
 * statorLimit} and speeding back up when it falls below, until STOW is reached.
 */
public class ShootStatorCommandFactory {
  private final ShooterController shooterController;
  private final IntakeController intakeController;
  private final IntakePivot intakePivot;
  private final ElasticUpdater matchTimerUpdater;
  private final Supplier<Rotation2d> getHeadingError;
  private final double initialVelocity;
  private final double statorLimit;
  private final double velocityRatio;

  double time = Timer.getFPGATimestamp();
  private boolean justShoot = false;
  private double currentVelocity;

  /**
   * @param initialVelocity starting cruise velocity for the pivot (rot/s)
   * @param statorLimit stator current threshold (A) above which the pivot slows
   * @param velocityRatio multiply velocity by this when stator is high; divide when low (< 1 to
   *     slow)
   */
  public ShootStatorCommandFactory(
      ShooterController shooterController,
      IntakeController intakeController,
      IntakePivot intakePivot,
      ElasticUpdater matchTimerUpdater,
      Supplier<Rotation2d> getHeadingError,
      double initialVelocity,
      double statorLimit,
      double velocityRatio) {
    this.shooterController = shooterController;
    this.intakeController = intakeController;
    this.intakePivot = intakePivot;
    this.matchTimerUpdater = matchTimerUpdater;
    this.getHeadingError = getHeadingError;
    this.initialVelocity = initialVelocity;
    this.statorLimit = statorLimit;
    this.velocityRatio = velocityRatio;
    this.currentVelocity = initialVelocity;
    SmartDashboard.putNumber("Intake Pivot In Time", 0);
  }

  public Command setJustShootCommand(boolean justShoot) {
    return new InstantCommand(() -> this.justShoot = justShoot);
  }

  public Command whileHeld() {
    Command statorAdaptiveStow =
        new InstantCommand(
                () -> {
                  currentVelocity = initialVelocity;
                  intakePivot.setMaxCruiseVelocity(currentVelocity);
                  intakeController.setTargetState(IntakeState.STOW);
                })
            .andThen(
                Commands.run(
                        () -> {
                          if (intakePivot.getStatorCurrentAmps() > statorLimit) {
                            currentVelocity *= velocityRatio;
                          } else {
                            currentVelocity =
                                Math.min(currentVelocity / velocityRatio, initialVelocity);
                          }
                          intakePivot.setMaxCruiseVelocity(currentVelocity);
                        })
                    .until(() -> intakePivot.reachedTarget()));

    return statorAdaptiveStow
        .repeatedly()
        .alongWith(
            new InstantCommand(
                    () -> {
                      shooterController.setTargetState(
                          (shooterController.getTargetState() == ShooterState.TOTAL_SPIN_UP
                                      || shooterController.getTargetState() == ShooterState.SHOOT)
                                  && shooterController.shooterRollersUpToSpeed()
                                  && (matchTimerUpdater.isOurHubActive()
                                      || matchTimerUpdater.getTimeUntilOurHubShifts() <= 2
                                      || matchTimerUpdater.getTimeUntilOurHubShifts() <= 24)
                                  && (getHeadingError.get().getDegrees() < 6
                                      || getHeadingError.get().getDegrees() > 354
                                      || justShoot)
                              ? ShooterState.SHOOT
                              : ShooterState.TOTAL_SPIN_UP);
                    })
                .repeatedly()
                .alongWith(
                    (new WaitUntilCommand(
                                () -> shooterController.getTargetState() == ShooterState.SHOOT)
                            .andThen(new InstantCommand(() -> time = Timer.getFPGATimestamp()))
                            .andThen(
                                new WaitUntilCommand(
                                    () ->
                                        (SmartDashboard.getNumber("Intake Pivot In Time", 0) + time)
                                            < Timer.getFPGATimestamp()))
                            .andThen(intakeController.setTargetStateCommand(IntakeState.STOW))
                            .withDeadline(
                                new WaitUntilCommand(
                                    () ->
                                        shooterController.getTargetState()
                                            == ShooterState.TOTAL_SPIN_UP)))
                        .repeatedly()));
  }

  public Command onRelease() {
    return new InstantCommand(
            () -> {
              if (shooterController.getTargetState() == ShooterState.SHOOT) {
                shooterController.setTargetState(ShooterState.COMPACT_SPIN_UP);
              }
            })
        .alongWith(intakeController.setTargetStateCommand(IntakeState.IDLE));
  }
}
