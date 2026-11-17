package frc.robot.subsystems.pivotvater.pivotvater_elevator;

import static frc.robot.subsystems.pivotvater.pivotvater_elevator.PivotvaterElevatorConstants.*;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;

public class PivotvaterElevatorIOTalonFX extends GenericSuperstructureIOTalonFX
    implements PivotvaterElevatorIO {
  public PivotvaterElevatorIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(PIVOTVATER_ELEVATOR_CONFIG.motorID())
            .withReduction(PIVOTVATER_ELEVATOR_CONFIG.reduction())
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
            .withMotorDirection(PIVOTVATER_ELEVATOR_CONFIG.motorDirection())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withAdditionalFollowerMotor(
                PIVOTVATER_ELEVATOR_CONFIG.followerMotorID(),
                PIVOTVATER_ELEVATOR_CONFIG.opposeMotor()
                    ? MotorAlignmentValue.Opposed
                    : MotorAlignmentValue.Aligned));

    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        MOTION_MAGIC_CONFIG.acceleration(),
        MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        GRAVITY_TYPE);
  }
}
