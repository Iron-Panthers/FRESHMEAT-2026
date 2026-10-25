package frc.robot.subsystems.intake.intake_pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotSimState;
import frc.robot.utility.*;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;

public class IntakePivotIOSim extends GenericSuperstructureIOSim implements IntakePivotIO {

  private final ElevatorSim intakePivotSim;
  private final double reduction;

  /** Holding load toward stow: median applied voltage while the real pivot is deployed and still. */
  private static final double LOAD_VOLTS = 0.61;

  private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);

  public IntakePivotIOSim() {
    super(
        IntakePivotConstants.INTAKE_PIVOT_CONFIG.motorID(),
        IntakePivotConstants.INTAKE_PIVOT_CONFIG.reduction());

    this.reduction = IntakePivotConstants.INTAKE_PIVOT_CONFIG.reduction();

    intakePivotSim =
        new ElevatorSim(
            MOTOR,
            reduction,
            IntakePivotConstants.PHYSICAL_CONSTANTS.massInKilograms(),
            IntakePivotConstants.PHYSICAL_CONSTANTS.drumRadiusMeters(),
            IntakePivotConstants.PHYSICAL_CONSTANTS.minExtensionMeters(),
            IntakePivotConstants.PHYSICAL_CONSTANTS.maxExtensionMeters(),
            IntakePivotConstants.PHYSICAL_CONSTANTS.simulateGravity(),
            0);
    setOffset();
    frc.robot.utility.SimBattery.getInstance()
        .register(() -> lastSupplyCurrentAmps, IntakePivotConstants.SUPPLY_CURRENT_LIMIT_AMPS);
    setSlot0(
        IntakePivotConstants.GAINS.kP(),
        IntakePivotConstants.GAINS.kI(),
        IntakePivotConstants.GAINS.kD(),
        IntakePivotConstants.GAINS.kS(),
        IntakePivotConstants.GAINS.kV(),
        IntakePivotConstants.GAINS.kA(),
        IntakePivotConstants.GAINS.kG(),
        IntakePivotConstants.MOTION_MAGIC_CONFIG.acceleration(),
        IntakePivotConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        IntakePivotConstants.GRAVITY_TYPE);
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
                / IntakePivotConstants.PHYSICAL_CONSTANTS.drumRadiusMeters(),
            reduction,
            MOTOR,
            RobotController.getBatteryVoltage(),
            IntakePivotConstants.SUPPLY_CURRENT_LIMIT_AMPS);

    intakePivotSim.setInputVoltage(
        frc.robot.utility.SimCurrentLimit.applyConstantLoad(appliedVoltage, LOAD_VOLTS));
    intakePivotSim.update(0.02);

    // Mechanism units, converted to rotor units for the sensor.
    double mechanismRotations =
        intakePivotSim.getPositionMeters()
            / (2 * Math.PI * IntakePivotConstants.PHYSICAL_CONSTANTS.drumRadiusMeters());
    double mechanismRPS =
        intakePivotSim.getVelocityMetersPerSecond()
            / (2 * Math.PI * IntakePivotConstants.PHYSICAL_CONSTANTS.drumRadiusMeters());

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
                / IntakePivotConstants.PHYSICAL_CONSTANTS.drumRadiusMeters(),
            reduction,
            MOTOR);
    double dutyCycle = availableVolts > 0.0 ? Math.abs(appliedVoltage) / availableVolts : 0.0;
    inputs.statorCurrent = statorAmps;
    inputs.supplyCurrentAmps = statorAmps * dutyCycle;
    lastSupplyCurrentAmps = inputs.supplyCurrentAmps;
    // reportedSupplyCurrentAmps = inputs.supplyCurrentAmps;

    // update the Sim State to match if it is up or down
    if (mechanismRotations < .1) {
      RobotSimState.getInstance().setIntakeState(false);
    } else {
      RobotSimState.getInstance().setIntakeState(true);
    }
  }

  @Override
  public void setOffset() {
    intakePivotSim.setState(0, 0);
  }
}
