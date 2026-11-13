package frc.robot.subsystems.pivotvater;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure.ControlMode;
import frc.robot.subsystems.pivotvater.pivotvater_elevator.PivotvaterElevator;
import frc.robot.subsystems.pivotvater.pivotvater_elevator.PivotvaterElevator.PivotvaterElevatorTarget;
import frc.robot.subsystems.pivotvater.pivotvater_pivot.PivotvaterPivot;
import frc.robot.subsystems.pivotvater.pivotvater_pivot.PivotvaterPivot.PivotvaterPivotTarget;
import org.littletonrobotics.junction.Logger;

public class PivotvaterController extends SubsystemBase {

  public enum PivotvaterState {
    IDLE(PivotvaterPivotTarget.STOW, PivotvaterElevatorTarget.NONE),
    STOW(PivotvaterPivotTarget.STOW, PivotvaterElevatorTarget.NONE),
    LOW(PivotvaterPivotTarget.STOW, PivotvaterElevatorTarget.LOW),
    MID(PivotvaterPivotTarget.STOW, PivotvaterElevatorTarget.CENTER_POINT),
    HIGH(PivotvaterPivotTarget.STOW, PivotvaterElevatorTarget.FULL_EXTEND),
    ZEROING(PivotvaterPivotTarget.STOW, PivotvaterElevatorTarget.NONE);

    private final PivotvaterPivotTarget pivotTarget;
    private final PivotvaterElevatorTarget elevatorTarget;

    private PivotvaterState(
        PivotvaterPivotTarget pivotTarget, PivotvaterElevatorTarget elevatorTarget) {
      this.pivotTarget = pivotTarget;
      this.elevatorTarget = elevatorTarget;
    }

    public PivotvaterPivotTarget getPivotTarget() {
      return pivotTarget;
    }

    public PivotvaterElevatorTarget getElevatorTarget() {
      return elevatorTarget;
    }
  }

  private final PivotvaterPivot pivot;
  private final PivotvaterElevator elevator;

  private PivotvaterState targetState = PivotvaterState.IDLE;
  private boolean stopped = false;

  public PivotvaterController(PivotvaterPivot pivot, PivotvaterElevator elevator) {
    this.pivot = pivot;
    this.elevator = elevator;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Pivotvater/State", targetState);
    Logger.recordOutput("Pivotvater/Stopped", stopped);

    if (stopped) {
      pivot.setControlMode(ControlMode.STOP);
      elevator.setControlMode(ControlMode.STOP);
    } else {
      if (pivot.getControlMode() != ControlMode.ZEROING) {
        pivot.setPositionTarget(targetState.getPivotTarget());
      }
      if (elevator.getControlMode() != ControlMode.ZEROING) {
        elevator.setPositionTarget(targetState.getElevatorTarget());
      }
    }

    pivot.periodic();
    elevator.periodic();

    Logger.recordOutput("Pivotvater/Reached Target", reachedTarget());
  }

  public void setTargetState(PivotvaterState targetState) {
    setStopped(false);
    this.targetState = targetState;
  }

  public PivotvaterState getTargetState() {
    return targetState;
  }

  public void setStopped(boolean stopped) {
    this.stopped = stopped;
  }

  public boolean reachedTarget() {
    return pivot.reachedTarget() && elevator.reachedTarget();
  }

  public Command setTargetStateCommand(PivotvaterState targetState) {
    return new InstantCommand(() -> setTargetState(targetState), this)
        .andThen(new WaitCommand(0.2).andThen(new WaitUntilCommand(this::reachedTarget)));
  }

  public Command setStoppedCommand(boolean stopped) {
    return new InstantCommand(() -> setStopped(stopped));
  }

  public Command zeroCommand() {
    return new InstantCommand(
            () -> {
              setTargetState(PivotvaterState.ZEROING);
              pivot.setControlMode(ControlMode.ZEROING);
              elevator.setControlMode(ControlMode.ZEROING);
            },
            this);
  }

  public Command stopZeroingCommand() {
    return new InstantCommand(this::stopZeroing);
  }

  public void stopZeroing() {
    pivot.endZeroing();
    elevator.endZeroing();
  }
}
