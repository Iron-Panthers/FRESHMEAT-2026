package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.serializer.Serializer;
import frc.robot.subsystems.shooter.serializer.Serializer.SerializerTarget;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAccelerator;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAccelerator.ShooterAcceleratorTarget;
import frc.robot.subsystems.shooter.shooter_omniwheels.ShooterOmniwheels;
import frc.robot.subsystems.shooter.shooter_omniwheels.ShooterOmniwheels.ShooterOmniwheelsTarget;
import frc.robot.subsystems.shooter.shooter_rollers.ShooterRollers;
import frc.robot.subsystems.shooter.shooter_rollers.ShooterRollers.ShooterRollersTarget;
import frc.robot.subsystems.swerve.DriveConstants;


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
            ShooterRollersTarget.SHOOT),
        INTAKE(
            SerializerTarget.INTAKE, //TODO: FIX
            ShooterAcceleratorTarget.INTAKE,
            ShooterOmniwheelsTarget.INTAKE,
            ShooterRollersTarget.INTAKE);

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
        initialize();
    }

    public void initialize() {
        RobotState.getInstance().initializeShooterTable();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter Speed", RobotState.getInstance().getShooterSpeed(RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(RobotState.isAllianceRed() ?
        DriveConstants.RED_HUB_ORIGIN.toTranslation2d() :
        DriveConstants.BLUE_HUB_ORIGIN.toTranslation2d())));
        Logger.recordOutput("Shooter/Target", targetState.toString());
        shooterRollers.setVelocityTarget(targetState.rollersTarget);
        shooterOmniwheels.setVelocityTarget(targetState.omniwheelsTarget);
        shooterAccelerator.setVelocityTarget(targetState.acceleratorTarget);
        serializer.setVelocityTarget(targetState.serializerTarget);
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
