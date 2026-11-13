package frc.robot.subsystems.pivotvater.pivotvater_elevator;

import static frc.robot.subsystems.superstructure.pivotvater_elevator.PivotvatorConstants.*;

import com.ctre.pheonix6.BaseStatusSignal;
import com.ctre.pheonix6.StatusSignal;
import com.ctre.pheonix6.controls.Follower;
import com.ctre.pheonix6.hardware.TalonFX;
import com.ctre.pheonix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.generic_subsystems.superstructure.*;

public class PivotvaterElevatorIOTalonFX extends GenericSuperstructureIOTalonFX implements PivotvaterElevatorIO { // extends ?, implements ?
    // only one thing really happens here. what is it?
    // also, in the offseason-2025 repo, there exists an extra motor defined here. consider whether this is necessary for this code specifically.
    // if so (or if not), what should be changed?
    private final StatusSignal<Angle> positionRotations2;
    private final StatusSignal<AngularVelocity> velocityRPS2;
    private final StatusSignal<Voltage> appliedVolts2;
    private final StatusSignal<Current> supplyCurrent2;
    private final StatusSignal<Current> statorCurrent2;
    private final StatusSignal<Temperature> temp2;

    protected TaloxFX talon2;

    public ElevatorIOTalonFX() {
        super(
            new GenericSuperstructureConfiguration()
                .withID(PIVOTVATOR_CONFIG.motor.ID))
                .withMotorDirection(MOTOR_DIRECTION)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withReduction(PIVOTVATOR_CONFIG.reduction())
                .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
                .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
                .withZeroingVolts(ZEROING_VOLTS)
                .withZeroingOffset(ZEROING_OFFSET)
                .withZeroingVoltageThreshhold(ZEROING_VOLTAGE_THRESHOLD));

                talon2 = new TalonFX(PIVOTVATOR_CONFIG.motor.ID2());
                talon2.getConfigurator().apply(config);
                talon2.setNeutralMode(NeutralModeValue.Brake);
                talon2.setControl(new Follower(talon.getDeviceID(), OPPOSE_MOTOR));
                setOffset();

                velocityRPS2 = talon2.getVelocity();
                appliedVolts2 = talon2.getMotorVoltage();
                supplyCurrent2 = talon2.getSupplyCurrent();
                statorCurrent2 = talon2.getStatorCurrent();
                temp2 = talon2.getDeviceTemp();
                positionRotations2 = talon2.getPosition();

                BaseStatusSignal.setUpdateFrequencyForAll(
                    50, positionRotations2, velocityRPS2, appliedVolts2, supplyCurrent2, statorCurrent2, temp2);,

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
                    MOTION_MAGIC_CONFIG.jerk(),
                    GRAVITY_TYPE);
                }

                @Override
                public void updateSecondaryInputs(GenericSuperstructureIOInputsMotor2 inputs) {
                    inputs.connected2 =
                        BaseStatusSignal.refreshAll(
                            positionRotations2, velocityRPS2, appliedVolts2, supplyCurrent2, statorCurrent2, temp2)
                        .isOK();
                    inputs.positionRotations2 = positionRotations2.getValueAsDouble();
                    inputs.velocityRPS2 = velocityRPS2.getValueAsDouble();
                    inputs.appliedVolts2 = appliedVolts2.getValueAsDouble();
                    inputs.supplyCurrent2 = supplyCurrent2.getValueAsDouble();
                    inputs.statorCurrent2 = statorCurrent2.getValueAsDouble();
                    inputs.temp2 = temp2.getValueAsDouble();
                }
                        
                @Override
                public void setOffset() {
                    super.setOffset();
                    if (talon2 != null) {
                        talon2.getConfigurator().setPosition(zeroingOffset);
                    }
                }
}
