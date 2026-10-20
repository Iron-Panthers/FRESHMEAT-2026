package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elastic_updater.ElasticUpdater;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import java.util.function.Supplier;

/**
 * Handles the shooting sequence while held. Toggles between spin-up and shoot states, runs
 * serializer intake, and cycles the intake up/down while shooting.
 *
 * <p>Use {@link #whileHeld()} for the whileTrue binding and {@link #onRelease()} for the onFalse
 * binding.
 */
public class ShootCommandFactory {
  private final ShooterController shooterController;
  private final IntakeController intakeController;
  private final ElasticUpdater matchTimerUpdater;
  private final Supplier<Rotation2d> getHeadingError;
  double time = Timer.getFPGATimestamp();
  private boolean justShoot = false;
    public ShootCommandFactory(
      ShooterController shooterController,
      IntakeController intakeController,
      ElasticUpdater matchTimerUpdater,
      Supplier<Rotation2d> getHeadingError) {
        this.shooterController = shooterController;
        this.intakeController = intakeController;
        this.matchTimerUpdater = matchTimerUpdater;
        this.getHeadingError = getHeadingError;
        SmartDashboard.putNumber("INtake rack in time",5);
      }

      public Command whileHeld() {
        return justshootCommand(false)
      }
      
      
      public Command justShootCommand(boolean justShoot) {
    return new InstantCommand(() -> this.justShoot = justShoot)
      .alongWith(
            new InstantCommand(() -> intakeController.setTargetState(IntakeState.INTAKE_SLOW))
                .andThen(
                    new WaitCommand(1)
                        .andThen(
                            new InstantCommand(
                                () -> intakeController.setTargetState(IntakeState.STOW)))
                        .andThen(new WaitCommand(0.1))
                        .andThen(() -> intakeController.setTargetState(IntakeState.INTAKE_SLOW)))
                .andThen(
                    new WaitCommand(0.2)
                        .andThen(
                            new InstantCommand(
                                () -> intakeController.setTargetState(IntakeState.STOW)))
                        .andThen(new WaitCommand(0.1))
                        .andThen(() -> intakeController.setTargetState(IntakeState.INTAKE_SLOW)))
                .andThen(
                    new WaitCommand(0.1)
                        .andThen(
                            new InstantCommand(
                                () -> intakeController.setTargetState(IntakeState.STOW)))
                        .andThen(new WaitCommand(0.1))
                        .andThen(
                            () ->
                                intakeController.setTargetState(
                                    IntakeState
                                        .INTAKE_SLOW))))
            .alongWith(
              new InstantCommand(()->{
                shooterController.setTargetState(
                  (shooterController.getTargetState()==ShooterState.TOTAL_SPIN_UP||
                  shooterController.getTargetState()==ShooterState.SHOOT)
                  &&shooterRollersUpToSpeed()
                  &&(matchTimerUpdater.isOurHubActive()
                  ||matchTimerUpdater.getTimeUntilOurHubShifts()<=2
                  ||matchTimerUpdater.getTimeUntilOurHubShifts()
                  <=24)
                  &&
                )
              })
            )

  }

}