package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.intake_rack.IntakeRack;
import frc.robot.subsystems.intake.intake_rack.IntakeRack.IntakeRackTarget;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers.IntakeRollersTarget;

public class IntakeController extends SubsystemBase {
    public enum IntakeState {
        IDLE(IntakeRackTarget.IDLE, IntakeRollersTarget.IDLE),
        INTAKE(IntakeRackTarget.INTAKE, IntakeRollersTarget.INTAKE);
        

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

    public IntakeController(IntakeRack intakeRack, IntakeRollers intakeRollers){
        this.intakeRack = intakeRack;
        this.intakeRollers = intakeRollers;
    }
    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Intake State", targetState);
    }

    public void setTargetState(IntakeState targetState) {
        this.targetState = targetState;
    }
    
    public IntakeState getTargetState() {
        return targetState;
    }
}
