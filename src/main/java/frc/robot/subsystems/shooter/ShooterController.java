package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.intake_rack.IntakeRack.IntakeRackTarget;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers.IntakeRollersTarget;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.serializer.Serializer.SerializerTarget;
import frc.robot.subsystems.shooter.ShooterRollers.ShooterRollers.ShooterRollerTarget;

public class ShooterController extends SubsystemBase{
    public enum ShooterState{
        IDLE(ShooterRollerTarget.IDLE),
        SHOOT(ShooterRollerTarget.SHOOT),
        INTAKE(ShooterRollerTarget.INTAKE);//do we need this?
        
       public final ShooterRollerTarget ShooterTarget;
        private ShooterState(ShooterRollerTarget ShooterTarget){ 
        this.ShooterTarget = ShooterTarget;
        
    }  
    
}

    private ShooterState shooterState = ShooterState.IDLE;
    private final Serializer serializer = new Serializer(null); 
    public ShooterState getTargetStateCommand() {
        return shooterState;
    }   

    public Command setTargetStateCommand(ShooterState targetState) {
        return new InstantCommand(() -> shooterState = targetState);
    } 
    

}
    

        


