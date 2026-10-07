package frc.robot.subsystems.shooter_omniwheels;

import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class ShooterOmniwheelsConstants {
    public static final ShooterOmniwheelsConfig SHOOTER_OMNIWHEELS_CONFIG =
    switch(Constants.getRobotType()) {
        case COMP -> new ShooterOmniwheelsConfig(
            CAN.at(9, "Shooter Omniwheels 1"),
            CAN.at(15, "Shooter Omniwheels 2"),
            1,
            false,
            false);
        case SIM -> new ShooterOmniwheelsConfig(
            CAN.at(9, "Shooter Omniwheels 1"),
            CAN.at(15, "ShooterOmniwheels 2"),
            1,
            false,
            false);
        default -> new ShooterOmniwheelsConfig(
            CAN.at(9, "Shooter Omniwheels 1"),
            CAN.at(15, "ShooterOmniwheels 2"),
            1,
            false,
            false);
    };

    public static final PIDGains GAINS =
    switch(Constants.getRobotType()) {
        case COMP -> new PIDGains(0.4, 0, 0, 0.2, .137, 0, 0);
        case SIM -> new PIDGains(0.4, 0, 0, 0.2, .137, 0, 0);
        default -> new PIDGains(0.4, 0, 0, 0.2, .137, 0, 0);
    };
    public static final int SUPPLY_CURRENT_LIMIT = 50;
    public static final int STATOR_CURRENT_LIMIT = 70;

    public record ShooterOmniwheelsConfig(
        int motorID1, int motorID2, double reduction, boolean inverted, boolean brake) {}

    public record PIDGains(
        double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
