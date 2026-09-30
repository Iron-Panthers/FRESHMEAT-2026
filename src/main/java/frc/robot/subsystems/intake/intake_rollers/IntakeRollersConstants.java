package frc.robot.subsystems.intake.intake_rollers;

import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class IntakeRollersConstants {
    public static final IntakeRollersConfig INTAKE_ROLLERS_CONFIG = new IntakeRollersConfig(
        CAN.at(5, "Intake Rollers 1"),
        CAN.at(6, "Intake Rollers 2"),
        1,
        false,
        false,
        false
    );

    public static final int CURRENT_LIMIT_AMPS = 1;

    public record IntakeRollersConfig(
        int motorID1,
        int motorID2,
        double reduction,
        boolean inverted,
        boolean brake,
        boolean opposeMotor
    ) {}
}