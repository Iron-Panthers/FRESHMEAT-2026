package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController;

public class ShootCommand {
    private final ShooterController shooterController;
    private final IntakeController intakeController;

    public ShootCommand(ShooterController shooterController, IntakeController intakeController) {
        this.shooterController = shooterController;
        this.intakeController = intakeController;
    }
    public Command whileHeld() {
        return Commands.sequence(Commands.runOnce(() -> intakeController.setTargetState(IntakeState.SHOOT)), 
                (Commands.runOnce(() -> shooterController.setTargetState(ShooterController.ShooterState.SHOOT))));
    } 
    public Command whenReleased() {
        return Commands.sequence(Commands.runOnce(() -> intakeController.setTargetState(IntakeState.IDLE)), 
                (Commands.runOnce(() -> shooterController.setTargetState(ShooterController.ShooterState.SHOOT))));
    }   
}
