package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;

import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(IntakeController intakeController, ShooterController shooterController){ {
        addCommands(
           intakeController
              .setTargetStateCommand(IntakeState.INTAKE)
                .alongWith(new InstantCommand(() -> shooterController.setTargetState(ShooterState.INTAKE))));
              
    }
}
}
