package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.subsystems.intake.intake_pivot.IntakePivot;
import frc.robot.subsystems.intake.intake_pivot.IntakePivot.IntakePivotTarget;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers.IntakeRollersTarget;
import frc.robot.lib.generic_subsystems.rollers.GenericRollers.ControlMode;

public class IntakeController extends SubsystemBase {
    public enum IntakeState {
        IDLE(IntakePivotTarget.IDLE, IntakeRollersTarget.IDLE),
        INTAKE(IntakePivotTarget.INTAKE, IntakeRollersTarget.INTAKE),
        ZEROING(IntakePivotTarget.STOW, IntakeRollersTarget.IDLE),
        REVERSE(IntakePivotTarget.INTAKE, IntakeRollersTarget.REVERSE);
        

        private IntakePivotTarget intakePivotTarget;
        private IntakeRollersTarget intakeRollersTarget;

        private IntakeState(IntakePivotTarget intakePivotTarget, IntakeRollersTarget intakeRollersTarget) {
            this.intakePivotTarget = intakePivotTarget;
            this.intakeRollersTarget = intakeRollersTarget;
        }

        public IntakePivotTarget getIntakePivotTarget() {
            return intakePivotTarget;
        }

        public IntakeRollersTarget getIntakeRollersTarget() {
            return intakeRollersTarget;
        }
    }

    private IntakeState targetState = IntakeState.IDLE;
    private IntakePivot intakePivot;
    private IntakeRollers intakeRollers;

    private boolean stopped = false;
    private boolean intakePivotActive = true;

    public IntakeController(IntakePivot intakePivot, IntakeRollers intakeRollers){
        this.intakePivot = intakePivot;
        this.intakeRollers = intakeRollers;
    }
    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Intake State", targetState);
        
        if (stopped) {
            Logger.recordOutput("Intake/Intake State Test", "stopped");
            intakeRollers.setControlMode(ControlMode.STOP);
            intakePivot.setControlMode(GenericSuperstructure.ControlMode.STOP);
        // if else set control mode to zero
        } else if (intakePivot.getControlMode() == GenericSuperstructure.ControlMode.ZEROING) {
            Logger.recordOutput("Intake/Intake State Test", "zeroing");
            intakeRollers.setVelocityTarget(targetState.getIntakeRollersTarget());
        } else if (targetState == IntakeState.IDLE && !intakePivot.reachedTarget()) {
            Logger.recordOutput("Intake/Intake State Test", "idle");
            intakeRollers.setVelocityTarget(IntakeRollersTarget.INTAKE);
            intakePivot.setPositionTarget(targetState.getIntakePivotTarget());
        } else {
            Logger.recordOutput("Intake/Intake State Test", "other");
            // set target states to those in the current controller state
            intakePivot.setPositionTarget(targetState.getIntakePivotTarget());
            intakeRollers.setVelocityTarget(targetState.getIntakeRollersTarget());
        }
        if (!intakePivotActive) {
            intakePivot.setPositionTarget(IntakePivotTarget.INTAKE);
        }
        intakePivot.periodic();
        intakeRollers.periodic();

        Logger.recordOutput("Intake/Active", intakePivotActive);
    }

    public void setStopped(boolean stopped) {
        this.stopped = stopped;
    }

    public void setTargetState(IntakeState targetState) {
        setStopped(false);
        this.targetState = targetState;
    }
    
    public IntakeState getTargetState() {
        return targetState;
    }

    public Command setTargetStateCommand(IntakeState targetState) {
    return new InstantCommand(() -> setTargetState(targetState), this)
        .andThen(
            new WaitCommand(0.2).andThen(new WaitUntilCommand(() -> intakePivot.reachedTarget())));
    }

    public Command setStoppedCommand(boolean stopped) {
        return new InstantCommand(() -> setStopped(stopped));
    }

    public Command zeroCommand() {
        return new InstantCommand(
                () -> intakePivot.setControlMode(GenericSuperstructure.ControlMode.ZEROING))
            .alongWith(setTargetStateCommand(IntakeState.ZEROING).alongWith(setStoppedCommand(false)));
    }

    public Command stopZeroingCommand() {
        return new InstantCommand(() -> intakePivot.endZeroing());
    }

    public void setIntakePivotActive(boolean isActive) {
        intakePivotActive = isActive;
    }

    public boolean getIntakePivotActive() {
        return intakePivotActive;
    }

    public void stopZeroing() {
        intakePivot.endZeroing();
    }
}
