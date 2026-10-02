package frc.robot.subsystems.intake.intake_rollers;

import static frc.robot.subsystems.intake.intake_rollers.IntakeRollersConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersConfiguration;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;

public class IntakeRollersIOTalonFX extends GenericRollersIOTalonFX implements IntakeRollersIO {
    public IntakeRollersIOTalonFX() {
        super(new GenericRollersConfiguration()
            .withID(INTAKE_ROLLERS_CONFIG.motorID1())
            .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
            .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
            .withMotorDirection(
                INTAKE_ROLLERS_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withNeutralMode(INTAKE_ROLLERS_CONFIG.brake())
            .withReduction(INTAKE_ROLLERS_CONFIG.reduction())
            .withAdditionalFollowerMotor(
                INTAKE_ROLLERS_CONFIG.motorID2(),
                INTAKE_ROLLERS_CONFIG.opposeMotor() ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
        super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
    }
}