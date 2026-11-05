package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.subsystems.shooter.shooter_rollers.ShooterRollersConstants;
import frc.robot.subsystems.swerve.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterController extends SubsystemBase {
  public enum ShooterState {
    IDLE(
        SerializerTarget.IDLE,
        ShooterAcceleratorTarget.IDLE,
        ShooterOmniwheelsTarget.IDLE,
        ShooterRollersTarget.SPIN_UP),
    SHOOT(
        SerializerTarget.SHOOT,
        ShooterAcceleratorTarget.SHOOT,
        ShooterOmniwheelsTarget.SHOOT,
        ShooterRollersTarget.SHOOT),
    SPIN_UP(
        SerializerTarget.SPIN_UP,
        ShooterAcceleratorTarget.SHOOT,
        ShooterOmniwheelsTarget.IDLE,
        ShooterRollersTarget.SHOOT),
    INTAKE(
        SerializerTarget.INTAKE,
        ShooterAcceleratorTarget.INTAKE,
        ShooterOmniwheelsTarget.INTAKE,
        ShooterRollersTarget.SPIN_UP),
    TOTAL_SPIN_UP(
        SerializerTarget.INTAKE,
        ShooterAcceleratorTarget.SHOOT,
        ShooterOmniwheelsTarget.INTAKE,
        ShooterRollersTarget.SPIN_UP),
    COMPACT_SPIN_UP(
        SerializerTarget.INTAKE,
        ShooterAcceleratorTarget.SHOOT,
        ShooterOmniwheelsTarget.IDLE,
        ShooterRollersTarget.SPIN_UP),
    ZEROING(
        SerializerTarget.IDLE,
        ShooterAcceleratorTarget.IDLE,
        ShooterOmniwheelsTarget.IDLE,
        ShooterRollersTarget.IDLE),
    PASS_SPIN_UP(
        SerializerTarget.IDLE,
        ShooterAcceleratorTarget.SHOOT,
        ShooterOmniwheelsTarget.IDLE,
        ShooterRollersTarget.SHOOT),
    PASS(
        SerializerTarget.SHOOT,
        ShooterAcceleratorTarget.SHOOT,
        ShooterOmniwheelsTarget.SHOOT,
        ShooterRollersTarget.SHOOT),
    TRENCH_DIVIDER(
        SerializerTarget.SHOOT,
        ShooterAcceleratorTarget.SHOOT,
        ShooterOmniwheelsTarget.SHOOT,
        ShooterRollersTarget.TRENCH_DIVIDER);

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
  private boolean constantRollersVelocity = false;
  private boolean autoAim = false;

  private final Serializer serializer;
  private final ShooterAccelerator shooterAccelerator;
  private final ShooterOmniwheels shooterOmniwheels;
  private final ShooterRollers shooterRollers;

  public ShooterController(
      Serializer serializer,
      ShooterAccelerator shooterAccelerator,
      ShooterOmniwheels shooterOmniwheels,
      ShooterRollers shooterRollers) {
    this.serializer = serializer;
    this.shooterAccelerator = shooterAccelerator;
    this.shooterOmniwheels = shooterOmniwheels;
    this.shooterRollers = shooterRollers;
    initialize();
    SmartDashboard.putNumber("Shoot Speed", 42);
  }

  public void initialize() {
    RobotState.getInstance().initializeShooterTable();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Shooter Speed",
        RobotState.getInstance()
            .getShooterSpeed(
                RobotState.getInstance()
                    .getEstimatedPose()
                    .getTranslation()
                    .getDistance(
                        RobotState.isAllianceRed()
                            ? DriveConstants.RED_HUB_ORIGIN.toTranslation2d()
                            : DriveConstants.BLUE_HUB_ORIGIN.toTranslation2d())));
    Logger.recordOutput("Shooter/Target", targetState.toString());
    double robotDistanceFromHub =
        RobotState.getInstance()
            .getEstimatedPose()
            .getTranslation()
            .getDistance(
                RobotState.isAllianceRed()
                    ? DriveConstants.RED_HUB_ORIGIN.toTranslation2d()
                    : DriveConstants.BLUE_HUB_ORIGIN.toTranslation2d());
    double shooterSpeed = RobotState.getInstance().getShooterSpeed(robotDistanceFromHub);

    if (targetState.rollersTarget == ShooterRollersTarget.SHOOT
        || targetState == ShooterState.TOTAL_SPIN_UP) {
      shooterRollers.setVelocityTargetManual(
          shooterSpeed, ShooterRollersConstants.CURRENT_LIMIT_AMPS);
    } else {
      shooterRollers.setVelocityTarget(targetState.rollersTarget);
    }
    shooterOmniwheels.setVelocityTarget(targetState.omniwheelsTarget);
    shooterAccelerator.setVelocityTarget(targetState.acceleratorTarget);
    serializer.setVelocityTarget(targetState.serializerTarget);
    Logger.recordOutput("Shooter/Shooter State", targetState);
    serializer.periodic();
    shooterAccelerator.periodic();
    shooterOmniwheels.periodic();
    shooterRollers.periodic();
    Logger.recordOutput("Shooter/Use Constant Roller Velocity", constantRollersVelocity);
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

  public Command setTargetStateCommand(ShooterState targetState) {
    return new InstantCommand(() -> setTargetState(targetState));
  }

  public void setAutoAim(boolean autoAim) {
    this.autoAim = autoAim;
  }

  public Command setAutoAimCommand(boolean autoAim) {
    return new InstantCommand(() -> setAutoAim(autoAim));
  }

  public boolean shooterRollersUpToSpeed() {
    return shooterRollers.reachedVelocityTarget();
  }

  public void setConstantRollersVelocity(boolean constantRollersVelocity) {
    this.constantRollersVelocity = constantRollersVelocity;
  }
}
