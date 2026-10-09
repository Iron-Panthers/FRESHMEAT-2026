package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.serializer.Serializer;
import frc.robot.subsystems.shooter.serializer.Serializer.SerializerTarget;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAccelerator;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAccelerator.ShooterAcceleratorTarget;
import frc.robot.subsystems.shooter.shooter_omniwheels.ShooterOmniwheels;
import frc.robot.subsystems.shooter.shooter_omniwheels.ShooterOmniwheels.ShooterOmniwheelsTarget;
import frc.robot.subsystems.shooter.shooter_rollers.ShooterRollers;
import frc.robot.subsystems.shooter.shooter_rollers.ShooterRollers.ShooterRollersTarget;


public class ShooterController extends SubsystemBase {
    public enum ShooterState {
        IDLE(
            SerializerTarget.IDLE,
            ShooterAcceleratorTarget.IDLE,
            ShooterOmniwheelsTarget.IDLE,
            ShooterRollersTarget.IDLE),
        SHOOT(
            SerializerTarget.SHOOT,
            ShooterAcceleratorTarget.SHOOT,
            ShooterOmniwheelsTarget.SHOOT,
            ShooterRollersTarget.SHOOT);

        public final SerializerTarget serializerTarget;
        public final ShooterAcceleratorTarget acceleratorTarget;
        public final ShooterOmniwheelsTarget omniwheelsTarget;
        public final ShooterRollersTarget rollersTarget;

        private ShooterState(
            SerializerTarget serializerTarget,
            ShooterAcceleratorTarget acceleratorTarget,
            ShooterOmniwheelsTarget omniwheelsTarget,
            ShooterRollersTarget rollersTarget) {
          this.serializerTarget = serializerTarget;
          this.acceleratorTarget = acceleratorTarget;
          this.omniwheelsTarget = omniwheelsTarget;
          this.rollersTarget = rollersTarget;
        }
    }

    private ShooterState targetState = ShooterState.IDLE;
    private boolean stopped = false;

    private final Serializer serializer;
    private final ShooterAccelerator shooterAccelerator;
    private final ShooterOmniwheels shooterOmniwheels;
    private final ShooterRollers shooterRollers;

    public ShooterController(
        Serializer serializer,
        ShooterAccelerator shooterAccelerator,
        ShooterOmniwheels shooterOmniwheels,
        ShooterRollers shooterRollers
    ) {
        this.serializer = serializer;
        this.shooterAccelerator = shooterAccelerator;
        this.shooterOmniwheels = shooterOmniwheels;
        this.shooterRollers = shooterRollers;
    }

    @Override
    public void periodic() {
        serializer.periodic();
        shooterAccelerator.periodic();
        shooterOmniwheels.periodic();
        shooterRollers.periodic();
    }

    public ShooterState getTargetState() {
        return targetState;
    }

    public void setTargetState(ShooterState targetState) {
        this.targetState = targetState;
    }

    public void setStopped(boolean stopped) {
        this.stopped = stopped;
    }
}
