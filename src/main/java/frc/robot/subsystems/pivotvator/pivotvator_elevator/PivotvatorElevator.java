package frc.robot.subsystems.pivotvator.pivotvator_elevator;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;

/*
 *
 * For freshmen: look at OFFSEASON-2025 code ->
 * https://github.com/Iron-Panthers/FRC-2025-Sprint/blob/dev/src/main/java/frc/robot/subsystems/superstructure/elevator/Elevator.java
 *
 * Some things will be a little different, but we believe you can do it. Try your best!
 *
 */

public class PivotvatorElevator
    extends GenericSuperstructure<PivotvatorElevator.PivotvatorElevatorTarget> {
  public enum PivotvatorElevatorTarget implements GenericSuperstructure.PositionTarget {
    // put values
    FULL_EXTEND(1, Optional.empty()),
    CENTER_POINT(0.5, Optional.empty()),
    LOW(0.25, Optional.empty()),
    NONE(0, Optional.empty());
    
    private double position;
    private Optional<Double> maxCruiseVelocity;
    private double supplyCurrentLimit = PivotvatorElevatorConstants.SUPPLY_CURRENT_LIMIT_AMPS;

    private static final double EPSILON = PivotvatorElevatorConstants.POSITION_TARGET_EPSILON;

    private PivotvatorElevatorTarget(double position, Optional<Double> maxCruiseVelocity) {
      this.position = position;
      this.maxCruiseVelocity = maxCruiseVelocity;
    }

    public double getPosition() {
      return position;
    }

    public double getEpsilon() {
      return EPSILON;
    }

    public Optional<Double> getMaxCruiseVelocity() {
      return maxCruiseVelocity;
    }

    public double getSupplyCurrentLimit() {
      return supplyCurrentLimit;
    }
  }

  private boolean zeroing = false;
  private double filteredSupplyCurrentAmps = 0;
  private final LinearFilter supplyCurrentFilter;
  public ProfiledPIDController pidController =
      new ProfiledPIDController(
          PivotvatorElevatorConstants.GAINS.kP(),
          PivotvatorElevatorConstants.GAINS.kI(),
          PivotvatorElevatorConstants.GAINS.kD(),
          new Constraints(
              PivotvatorElevatorConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
              PivotvatorElevatorConstants.MOTION_MAGIC_CONFIG.acceleration()));

  // private GenericSuperstructureIOInputsMotorAuroLogged inputs = new
  // GenericSuperstructureIOInputsMotorAuroLogged();

  public PivotvatorElevator(PivotvatorElevatorIO io) {
    super("Pivotvator/PivotvatorElevator", io);
    setPositionTarget(PivotvatorElevatorTarget.NONE);
    setControlMode(ControlMode.STOP);
    supplyCurrentFilter = LinearFilter.movingAverage(30);
  }

  @Override
  public void periodic() {
    super.periodic();

    if (zeroing) {
      superstructureIO.runCharacterization();
    }
    filteredSupplyCurrentAmps = supplyCurrentFilter.calculate(getSupplyCurrentAmps());
    if (getPositionTarget().getMaxCruiseVelocity().isPresent()
        && getControlMode() != ControlMode.ZEROING) {
      pidController.setConstraints(
          new Constraints(
              getPositionTarget().getMaxCruiseVelocity().get(),
              PivotvatorElevatorConstants.MOTION_MAGIC_CONFIG.acceleration()));
      pidController.setGoal(getPositionTarget().getPosition());
      pidController.calculate(getPosition());
      superstructureIO.runPosition(pidController.getSetpoint().position);
    }
  }

  public double getFilteredSupplyCurrentAmps() {
    return filteredSupplyCurrentAmps;
  }

  public void setZeroing(boolean zeroing) {
    this.zeroing = zeroing;
  }

  public boolean isZeroing() {
    return zeroing;
  }
}
    /*
     * Position target enum
     */

  // Constructor

  // setPositionTarget (to reset pid)

     // periodic
