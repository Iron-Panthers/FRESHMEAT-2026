package frc.robot.subsystems.pivotvator.pivotvator_elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotSimState;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;
import frc.robot.utility.*;

// I mostly copied this from IntakePivotIOSim.java
public class PivotvatorElevatorIOSim extends GenericSuperstructureIOSim
    implements PivotvatorElevatorIO {

  private final ElevatorSim intakePivotSim;
  private final double reduction;

  /**
   * Holding load toward stow: median applied voltage while the real pivot is deployed and still.
   */
  private static final double LOAD_VOLTS = 0.61;

  private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);

  public PivotvatorElevatorIOSim() {
    super(
        PivotvatorElevatorConstants.PIVOTVATOR_ELEVATOR_CONFIG.motorID(),
        PivotvatorElevatorConstants.PIVOTVATOR_ELEVATOR_CONFIG.reduction());

    this.reduction = PivotvatorElevatorConstants.PIVOTVATOR_ELEVATOR_CONFIG.reduction();

    intakePivotSim =
        new ElevatorSim(
            MOTOR,
            reduction,
            PivotvatorElevatorConstants.PHYSICAL_CONSTANTS.massInKilograms(),
            PivotvatorElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters(),
            PivotvatorElevatorConstants.PHYSICAL_CONSTANTS.minExtensionMeters(),
            PivotvatorElevatorConstants.PHYSICAL_CONSTANTS.maxExtensionMeters(),
            PivotvatorElevatorConstants.PHYSICAL_CONSTANTS.simulateGravity(),
            0);
    setOffset();
    frc.robot.utility.SimBattery.getInstance()
        .register(
            () -> lastSupplyCurrentAmps, PivotvatorElevatorConstants.SUPPLY_CURRENT_LIMIT_AMPS);
    setSlot0(
        PivotvatorElevatorConstants.GAINS.kP(),
        PivotvatorElevatorConstants.GAINS.kI(),
        PivotvatorElevatorConstants.GAINS.kD(),
        PivotvatorElevatorConstants.GAINS.kS(),
        PivotvatorElevatorConstants.GAINS.kV(),
        PivotvatorElevatorConstants.GAINS.kA(),
        PivotvatorElevatorConstants.GAINS.kG(),
        PivotvatorElevatorConstants.MOTION_MAGIC_CONFIG.acceleration(),
        PivotvatorElevatorConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        PivotvatorElevatorConstants.GRAVITY_TYPE);
  }

  /** Last computed supply current, published to SimBattery. */
  private double lastSupplyCurrentAmps = 0.0;

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    appliedVoltage =
        frc.robot.utility.SimCurrentLimit.clampToSupplyLimit(
            appliedVoltage,
            intakePivotSim.getVelocityMetersPerSecond()
                / PivotvatorElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters(),
            reduction,
            MOTOR,
            RobotController.getBatteryVoltage(),
            PivotvatorElevatorConstants.SUPPLY_CURRENT_LIMIT_AMPS);

    double angleRadians =
        intakePivotSim.getPositionMeters()
            / PivotvatorElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters();
    intakePivotSim.setInputVoltage(
        frc.robot.utility.SimCurrentLimit.applyConstantLoad(
            appliedVoltage, LOAD_VOLTS * Math.cos(angleRadians)));
    intakePivotSim.update(0.02);

    // Mechanism units, converted to rotor units for the sensor.
    double mechanismRotations =
        intakePivotSim.getPositionMeters()
            / (2 * Math.PI * PivotvatorElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters());
    double mechanismRPS =
        intakePivotSim.getVelocityMetersPerSecond()
            / (2 * Math.PI * PivotvatorElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters());

    talon.getSimState().setRawRotorPosition(mechanismRotations * reduction);
    talon.getSimState().setRotorVelocity(mechanismRPS * reduction);

    inputs.isConnected = true;
    inputs.positionRotations = mechanismRotations;
    inputs.velocityRotPerSec = mechanismRPS;
    inputs.appliedVolts = appliedVoltage;
    double availableVolts = RobotController.getBatteryVoltage();
    // Against the commanded voltage, so holding the load draws current.
    double statorAmps =
        frc.robot.utility.SimCurrentLimit.statorAmps(
            appliedVoltage,
            intakePivotSim.getVelocityMetersPerSecond()
                / PivotvatorElevatorConstants.PHYSICAL_CONSTANTS.drumRadiusMeters(),
            reduction,
            MOTOR);
    double dutyCycle = availableVolts > 0.0 ? Math.abs(appliedVoltage) / availableVolts : 0.0;
    inputs.statorCurrent = statorAmps;
    inputs.supplyCurrentAmps = statorAmps * dutyCycle;
    lastSupplyCurrentAmps = inputs.supplyCurrentAmps;
    // reportedSupplyCurrentAmps = inputs.supplyCurrentAmps;

    // update the Sim State to match if it is up or down
    if (mechanismRotations < .1) {
      RobotSimState.getInstance().setIntakeState(true);
    } else {
      RobotSimState.getInstance().setIntakeState(false);
    }
  }

  @Override
  public void setOffset() {
    intakePivotSim.setState(0, 0);
  }
}
