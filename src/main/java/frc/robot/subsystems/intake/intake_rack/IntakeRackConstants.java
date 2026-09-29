package frc.robot.subsystems.intake.intake_rack;

import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class IntakeRackConstants {
    public static final IntakeRackConfig INTAKE_RACK_CONFIG = new IntakeRackConfig(
        CAN.at(3, "Intake Rack"),
        1,
        false,
        false
    );

    public static final int CURRENT_LIMIT_AMPS = 1;

    public record IntakeRackConfig(
        int motorID,
        double reduction,
        boolean inverted,
        boolean opposeMotor
    ) {}
    public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
