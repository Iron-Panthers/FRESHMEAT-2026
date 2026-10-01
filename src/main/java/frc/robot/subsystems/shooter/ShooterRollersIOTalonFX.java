package frc.robot.subsystems.shooter;
import static frc.robot.subsystems.shooter.ShooterRollersConstants.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterRollersIOTalonFX extends GenericRollersIOTalonFX implements ShooterRollersIO {
    protected TalonFX talon1;
    protected TalonFX talon2;
    protected TalonFX talon3;
    protected TalonFX talon4;
    
    public ShooterRollersIOTalonFX() {
        super(
            new GenericRollersConfiguration()
                .withID(SHOOTER_ROLLERS_CONFIG.motorID1())
                .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
            .withMotorDirection(
                SHOOTER_ROLLERS_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withNeutralMode(SHOOTER_ROLLERS_CONFIG.brake())
            .withReduction(SHOOTER_ROLLERS_CONFIG.reduction())
            .withAdditionalFollowerMotor(
                SHOOTER_ROLLERS_CONFIG.motorID2(), SHOOTER_ROLLERS_CONFIG.opposeMotor1())
            .withAdditionalFollowerMotor(
                SHOOTER_ROLLERS_CONFIG.motorID3(), SHOOTER_ROLLERS_CONFIG.opposeMotor2())
            .withAdditionalFollowerMotor(
                SHOOTER_ROLLERS_CONFIG.motorID4(), SHOOTER_ROLLERS_CONFIG.opposeMotor3()));
    super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}


