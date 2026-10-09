package frc.robot.commands;

import java.io.Serial;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.intake.intake_rack.IntakeRack;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(IntakeController intakeController, ShooterController shooterController){ {
        addCommands(
           intakeController
              .setTargetStateCommand(IntakeState.INTAKE)
                .alongWith(shooterController.setTargetStateCommand(ShooterState.INTAKE)));
              
    }
}
}
