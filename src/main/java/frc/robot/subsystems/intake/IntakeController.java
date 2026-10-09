package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.subsystems.intake.intake_rack.IntakeRack;
import frc.robot.subsystems.intake.intake_rack.IntakeRack.IntakeRackTarget;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers.IntakeRollersTarget;
import frc.robot.lib.generic_subsystems.rollers.GenericRollers.ControlMode;

public class IntakeController extends SubsystemBase {
    public enum IntakeState {
        IDLE(IntakeRackTarget.IDLE, IntakeRollersTarget.IDLE),
        INTAKE(IntakeRackTarget.INTAKE, IntakeRollersTarget.INTAKE),
    ZEROING(IntakeRackTarget.STOW, IntakeRollersTarget.IDLE);
        

        private IntakeRackTarget intakeRackTarget;
        private IntakeRollersTarget intakeRollersTarget;

        private IntakeState(IntakeRackTarget intakeRackTarget, IntakeRollersTarget intakeRollersTarget) {
            this.intakeRackTarget = intakeRackTarget;
            this.intakeRollersTarget = intakeRollersTarget;
        }

        public IntakeRackTarget getIntakeRackTarget() {
            return intakeRackTarget;
        }

        public IntakeRollersTarget getIntakeRollersTarget() {
            return intakeRollersTarget;
        }
    }

    private IntakeState targetState = IntakeState.IDLE;
    private IntakeRack intakeRack;
    private IntakeRollers intakeRollers;

    private boolean stopped = false;
    private boolean intakeRackActive = true;

    public IntakeController(IntakeRack intakeRack, IntakeRollers intakeRollers){
        this.intakeRack = intakeRack;
        this.intakeRollers = intakeRollers;
    }
    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Intake State", targetState);
        
        if (stopped) {
            Logger.recordOutput("Intake/Intake State Test", "stopped");
            intakeRollers.setControlMode(ControlMode.STOP);
            intakeRack.setControlMode(GenericSuperstructure.ControlMode.STOP);
        // if else set control mode to zero
        } else if (intakeRack.getControlMode() == GenericSuperstructure.ControlMode.ZEROING) {
            Logger.recordOutput("Intake/Intake State Test", "zeroing");
            intakeRollers.setVelocityTarget(targetState.getIntakeRollersTarget());
        } else if (intakeRack.getPosition() < 1.5 && targetState == IntakeState.INTAKE) {
            Logger.recordOutput("Intake/Intake State Test", "intake");
            intakeRollers.setVelocityTarget(IntakeRollersTarget.IDLE);
            intakeRack.setPositionTarget(targetState.getIntakeRackTarget());
        } else if (targetState == IntakeState.IDLE && !intakeRack.reachedTarget()) {
            Logger.recordOutput("Intake/Intake State Test", "idle");
            intakeRollers.setVelocityTarget(IntakeRollersTarget.INTAKE);
            intakeRack.setPositionTarget(targetState.getIntakeRackTarget());
        } else {
            Logger.recordOutput("Intake/Intake State Test", "other");
            // set target states to those in the current controller state
            intakeRack.setPositionTarget(targetState.getIntakeRackTarget());
            intakeRollers.setVelocityTarget(targetState.getIntakeRollersTarget());
        }
        if (!intakeRackActive) {
            intakeRack.setPositionTarget(IntakeRackTarget.INTAKE);
        }
        intakeRack.periodic();
        intakeRollers.periodic();

        Logger.recordOutput("Intake/Active", intakeRackActive);
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
            new WaitCommand(0.2).andThen(new WaitUntilCommand(() -> intakeRack.reachedTarget())));
    }

    public Command setStoppedCommand(boolean stopped) {
        return new InstantCommand(() -> setStopped(stopped));
    }

    public Command zeroCommand() {
        return new InstantCommand(
                () -> intakeRack.setControlMode(GenericSuperstructure.ControlMode.ZEROING))
            .alongWith(setTargetStateCommand(IntakeState.ZEROING).alongWith(setStoppedCommand(false)));
    }

    public Command stopZeroingCommand() {
        return new InstantCommand(() -> intakeRack.endZeroing());
    }

    public void setIntakeRackActive(boolean isActive) {
        intakeRackActive = isActive;
    }

    public boolean getIntakeRackActive() {
        return intakeRackActive;
    }

    public void stopZeroing() {
        intakeRack.endZeroing();
    }
}
