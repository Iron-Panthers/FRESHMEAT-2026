package frc.robot.subsystems.pivotvater.pivotvater_elevator;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.subsystems.intake.IntakeController.IntakeState;

/*
 *
 * For freshmen: look at OFFSEASON-2025 code ->
 * https://github.com/Iron-Panthers/FRC-2025-Sprint/blob/dev/src/main/java/frc/robot/subsystems/superstructure/elevator/Elevator.java
 *
 * Some things will be a little different, but we believe you can do it. Try your best!
 *
 */

public class PivotvaterElevator
    extends GenericSuperstructure<PivotvaterElevator.PivotvaterElevatorTarget> {
  public enum PivotvaterElevatorTarget implements GenericSuperstructure.PositionTarget {
    // put values
    FULL_EXTEND(1),
    CENTER_POINT(0.5),
    LOW(0.25),
    NONE(0);
    private double position = 0;
    private double supplyCurrentLimit = PivotvaterElevatorConstants.SUPPLY_CURRENT_LIMIT_AMPS;

    private static final double EPSILON = PivotvaterElevatorConstants.POSITION_TARGET_EPSILON;

    private PivotvaterElevatorTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }

    public double getEpsilon() {
      return EPSILON;
    }

    public double getSupplyCurrentLimit() {
      return supplyCurrentLimit;
    }
  }

  private double filteredSupplyCurrentAmps = 0;
  private final LinearFilter supplyCurrentFilter;

  // private GenericSuperstructureIOInputsMotorAuroLogged inputs = new
  // GenericSuperstructureIOInputsMotorAuroLogged();

  public PivotvaterElevator(PivotvaterElevatorIO io) {
    super("PivotvaterElevator", io);
    setPositionTarget(PivotvaterElevatorTarget.NONE);
    setControlMode(ControlMode.STOP);
    supplyCurrentFilter = LinearFilter.movingAverage(30);
  }

  @Override
  public void periodic() {
    super.periodic();

   
    filteredSupplyCurrentAmps = supplyCurrentFilter.calculate(getSupplyCurrentAmps());
  }

  public double getFilteredSupplyCurrentAmps() {
    return filteredSupplyCurrentAmps;
  }

 
}
    /*
     * Position target enum
     */

  // Constructor

  // setPositionTarget (to reset pid)

     // periodic
