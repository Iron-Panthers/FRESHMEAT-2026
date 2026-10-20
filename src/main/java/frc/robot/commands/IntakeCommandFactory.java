package frc.robot.commands;

import java.io.Serial;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.intake.intake_rack.IntakeRack.IntakeRackTarget;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.shooter.serializer.Serializer;
import frc.robot.subsystems.shooter.serializer.Serializer.SerializerTarget;

public class IntakeCommandFactory {
    private IntakeController intakeController;
    private ShooterController shooterController;
    private Serializer serializer;

    public IntakeCommandFactory(
        IntakeController intakeController, 
        ShooterController shooterController, 
        Serializer serializer){
            this.intakeController = intakeController;
            this.shooterController = shooterController;
            this.serializer = serializer;
    }

    public Command whileHeld(){
        return intakeController
            .setTargetStateCommand(IntakeState.INTAKE)
            .alongWith(shooterController.setTargetStateCommand(ShooterState.INTAKE))
            .alongWith(new InstantCommand(() -> serializer.setVelocityTarget(SerializerTarget.SLOW)));
    }

    //FIXME
    public Command onRelease(){
        return intakeController
            .setTargetStateCommand(IntakeState.IDLE)
            .alongWith(shooterController.setTargetStateCommand(ShooterState.IDLE))
            .alongWith(new InstantCommand(() -> serializer.setVelocityTarget(SerializerTarget.IDLE)));
    }

}
