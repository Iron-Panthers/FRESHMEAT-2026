package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.pivotvater.PivotvaterController;
import frc.robot.subsystems.pivotvater.PivotvaterController.PivotvaterState;
import frc.robot.subsystems.pivotvater.pivotvater_elevator.PivotvaterElevator;
import frc.robot.subsystems.pivotvater.pivotvater_elevator.PivotvaterElevator.PivotvaterElevatorTarget;
import frc.robot.subsystems.pivotvater.pivotvater_pivot.PivotvaterPivot;
import frc.robot.subsystems.pivotvater.pivotvater_pivot.PivotvaterPivot.PivotvaterPivotTarget;

public class PivotvaterPunchingCommand {
  private final PivotvaterController pivotvaterController;
  private final Supplier<Rotation2d> getHeadingError;
  double time = Timer.getFPGATimestamp();
 

  public PivotvaterPunchingCommand(
      PivotvaterController pivotvaterController,
      Supplier<Rotation2d> getHeadingError) {
    this.pivotvaterController = pivotvaterController;
    this.getHeadingError = getHeadingError;
    SmartDashboard.putNumber("Intake Pivot In Time", 0.5);
  }

  

  public Command whileHeld() {
    PivotvaterPivot pivot = pivotvaterController.getPivotvaterPivot();
    PivotvaterElevator elevator = pivotvaterController.getPivotvaterElevator();
    return new InstantCommand(() -> pivot.setPositionTarget(PivotvaterPivotTarget.HALF_UP))
    .andThen(
        new WaitUntilCommand(() -> pivot.getPivotvaterPivotTarget().getPosition() > 0.14 && pivot.getPivotvaterPivotTarget().getPosition() < 0.16)
        .andThen(new InstantCommand(() -> elevator.setPositionTarget(PivotvaterElevatorTarget.FULL_EXTEND)))
        .andThen(new WaitUntilCommand(() -> elevator.getPivotvaterElevatorTarget().getPosition() > 0.99 && elevator.getPivotvaterElevatorTarget().getPosition() < 1.01))
        .andThen(new InstantCommand(() -> elevator.setPositionTarget(PivotvaterElevatorTarget.LOW)))
        .andThen(new WaitUntilCommand(() -> elevator.getPivotvaterElevatorTarget().getPosition() > 0.24 && elevator.getPivotvaterElevatorTarget().getPosition() < 0.36))
        .andThen(new InstantCommand(() -> pivot.setPositionTarget(PivotvaterPivotTarget.STOW)))
        .andThen(new WaitUntilCommand(() -> pivot.getPivotvaterPivotTarget().getPosition() > 0.09 && pivot.getPivotvaterPivotTarget().getPosition() < 0.11)) 
        .andThen(new InstantCommand(() -> pivot.setPositionTarget(PivotvaterPivotTarget.STOW))))
                    .repeatedly();
               
                       
  }

  public Command onRelease() {
    return new InstantCommand(() -> pivotvaterController.setTargetState(PivotvaterState.IDLE));
  }
/* 
  public Command whileHeldPassing() {
    return new InstantCommand(
            () -> {
              shooterController.setTargetState(
                  (shooterController.getTargetState() == ShooterState.PASS_SPIN_UP
                              || shooterController.getTargetState() == ShooterState.PASS)
                          && shooterController.shooterRollersUpToSpeed() // time correct
                      ? ShooterState.PASS
                      : ShooterState.PASS_SPIN_UP);
            })
        .repeatedly()
        .alongWith(
            new WaitCommand(1.5).andThen(pivotvaterController.setTargetStateCommand(PivotvaterState.LOW)));
  }
            */
}

