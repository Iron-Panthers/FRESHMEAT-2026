package frc.robot.subsystems.pivotvator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.pivotvator.pivotvator_elevator.PivotvatorElevator;
import frc.robot.subsystems.pivotvator.pivotvator_elevator.PivotvatorElevator.PivotvatorElevatorTarget;
import frc.robot.subsystems.pivotvator.pivotvator_pivot.PivotvatorPivot;
import frc.robot.subsystems.pivotvator.pivotvator_pivot.PivotvatorPivot.PivotvatorPivotTarget;
import org.littletonrobotics.junction.Logger;

public class PivotvatorController extends SubsystemBase {

  public enum PivotvatorState {
    IDLE(PivotvatorPivotTarget.STOW, PivotvatorElevatorTarget.NONE),
    STOW(PivotvatorPivotTarget.STOW, PivotvatorElevatorTarget.NONE),
    LOW(PivotvatorPivotTarget.STOW, PivotvatorElevatorTarget.LOW),
    MID(PivotvatorPivotTarget.STOW, PivotvatorElevatorTarget.CENTER_POINT),
    HIGH(PivotvatorPivotTarget.STOW, PivotvatorElevatorTarget.FULL_EXTEND),
    ZEROING(PivotvatorPivotTarget.STOW, PivotvatorElevatorTarget.NONE);

    private PivotvatorPivotTarget pivotTarget;
    private PivotvatorElevatorTarget elevatorTarget;

    private PivotvatorState(
        PivotvatorPivotTarget pivotTarget, PivotvatorElevatorTarget elevatorTarget) {
      this.pivotTarget = pivotTarget;
      this.elevatorTarget = elevatorTarget;
    }

    public PivotvatorPivotTarget getPivotTarget() {
      return pivotTarget;
    }

    public PivotvatorElevatorTarget getElevatorTarget() {
      return elevatorTarget;
    }
  }

  private final PivotvatorPivot pivot;
  private final PivotvatorElevator elevator;

  private PivotvatorState targetState = PivotvatorState.IDLE;
  private boolean stopped = false;

  public PivotvatorController(PivotvatorPivot pivot, PivotvatorElevator elevator) {
    this.pivot = pivot;
    this.elevator = elevator;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Pivotvator/State", targetState);

    if (stopped) {
      // TODO maybe there should be loggers here
      pivot.setControlMode(ControlMode.STOP);
      elevator.setControlMode(ControlMode.STOP);
    } else {
      pivot.setPositionTarget(targetState.getPivotTarget());
      elevator.setPositionTarget(targetState.getElevatorTarget());
    }

    // TODO for the love of god fix the override thing above

    pivot.periodic();
    elevator.periodic();

    Logger.recordOutput("Pivotvator/Reached", reachedTarget());
  }

  public void setTargetState(PivotvatorState targetState) {
    setStopped(false);
    this.targetState = targetState;
  }

  public void setStopped(boolean stopped) {
    this.stopped = stopped;
  }

  public boolean reachedTarget() {
    return pivot.reachedTarget() && elevator.reachedTarget();
  }

  public Command setTargetStateCommand(PivotvatorState targetState) {
    return new InstantCommand(() -> setTargetState(targetState), this)
        .andThen(new WaitCommand(0.2).andThen(new WaitUntilCommand(this::reachedTarget)));
  }

  public Command setStoppedCommand(boolean stopped) {
    return new InstantCommand(() -> setStopped(stopped));
  }

  public Command zeroCommand() {
    return new InstantCommand(
        () -> {
          setTargetState(PivotvatorState.ZEROING);
          pivot.setControlMode(ControlMode.ZEROING);
          elevator.setControlMode(ControlMode.ZEROING);
        },
        this);
  }

  // public Command stopZeroingCommand() {
  //   return new InstantCommand(() -> this.endZeroing());
  // }

  public void stopZeroing() {
    pivot.endZeroing();
    elevator.endZeroing();
  }
}
