package frc.robot.subsystems.intake.intake_rack;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIO;
import frc.robot.utility.LoggableMechanism3d;

public class IntakeRack extends GenericSuperstructure<IntakeRack.IntakeRackTarget>
    implements LoggableMechanism3d {
    public enum IntakeRackTarget implements GenericSuperstructure.PositionTarget {
        IDLE(0, IntakeRackConstants.SUPPLY_CURRENT_LIMIT_AMPS, Optional.empty()),
        INTAKE(10, IntakeRackConstants.SUPPLY_CURRENT_LIMIT_AMPS, Optional.empty()),
        STOW(0, IntakeRackConstants.SUPPLY_CURRENT_LIMIT_AMPS, Optional.empty());

        private static final double EPSILON = IntakeRackConstants.POSITION_TARGET_EPSILON;
        private double position;
        private double supplyCurrentLimit;
    private Optional<Double> maxCruiseVelocity;

        public double getEpsilon() {
            return EPSILON;
        }

        public double getPosition() {
            return position;
        }

        public double getSupplyCurrentLimit() {
            return supplyCurrentLimit;
        }

        public Optional<Double> getMaxCruiseVelocity() {
            return maxCruiseVelocity;
        }

        private IntakeRackTarget(double position, double supplyCurrentLimit, Optional<Double> maxCruiseVelocity) {
            this.position = position;
            this.supplyCurrentLimit = supplyCurrentLimit;
            this.maxCruiseVelocity = maxCruiseVelocity;
        }
    }

    public IntakeRack(GenericSuperstructureIO io) {
        super("Intake/Intake Rack", io);
        setPositionTarget(IntakeRackTarget.STOW);
        setControlMode(ControlMode.STOP);
    }

    public LoggableMechanism3d loggableMechanism3dParent = null;
    public ProfiledPIDController pidController =
        new ProfiledPIDController(
            IntakeRackConstants.GAINS.kP(),
            IntakeRackConstants.GAINS.kI(),
            IntakeRackConstants.GAINS.kD(),
            new Constraints(
                IntakeRackConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
                IntakeRackConstants.MOTION_MAGIC_CONFIG.acceleration()));

    private IntakeRackTarget lastTarget = null;

    @Override
    public void setPositionTarget(IntakeRackTarget target) {
        if (target != lastTarget && target.getMaxCruiseVelocity().isPresent()) {
        pidController.reset(getPosition());
        }
        lastTarget = target;
        super.setPositionTarget(target);
    }

    @Override
    public void periodic() {
        super.periodic();
        if (getPositionTarget().getMaxCruiseVelocity().isPresent()
            && getControlMode() != ControlMode.ZEROING) {
        pidController.setConstraints(
            new Constraints(
                getPositionTarget().getMaxCruiseVelocity().get(),
                IntakeRackConstants.MOTION_MAGIC_CONFIG.acceleration()));
        pidController.setGoal(getPositionTarget().getPosition());
        pidController.calculate(getPosition());
        superstructureIO.runPosition(pidController.getSetpoint().position);
        }
        Logger.recordOutput(
            "Intake/Intake Rack/Position Target Rotations", getPositionTarget().getPosition());
        Logger.recordOutput(
            "Intake/Intake Rack/Position Target Rotations Pid", pidController.getSetpoint().position);
    }

    @Override
    public Pose3d getParentPosition() {
        if (loggableMechanism3dParent != null) {
            return loggableMechanism3dParent.getDisplayPose3d();
        }
        return new Pose3d();
    }

    @Override
    public void setParent(LoggableMechanism3d parent) {
        if (parent == null) {
            throw new IllegalArgumentException("Parent cannot be null");
        }
        if (parent == this) {
            throw new IllegalArgumentException("Parent cannot be itself");
        }
        this.loggableMechanism3dParent = parent;
    }

    @AutoLogOutput(key = "Intake/Intake Rack/Display Pose3d")
    @Override
    public Pose3d getDisplayPose3d() {
        return getParentPosition()
            .plus(IntakeRackConstants.BASE_TO_INTAKE_RACK_TRANSFORM)
            .plus(
                new Transform3d(
                    new Translation3d(0, Units.inchesToMeters(getPosition()), 0), Rotation3d.kZero));
    }
}
