package frc.robot.subsystems.intake.intake_rollers;

import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

// I really like this idea but sadly now is just not the time :( Also I am a bit too drunk rn to decide whether or not to keep this so... :P
// import frc.robot.subsystems.intake.intake_rack.IntakeRackConstants.PIDGains;

public class IntakeRollersConstants {
    public static final IntakeRollersConfig INTAKE_ROLLERS_CONFIG =
        switch (Constants.getRobotType()) {
            case COMP -> new IntakeRollersConfig(
                CAN.at(5, "Intake Rollers 1"),
                CAN.at(6, "Intake Rollers 2"),
                1,
                false,
                false,
                false);
            case SIM -> new IntakeRollersConfig(
                CAN.at(5, "Intake Rollers 1"),
                CAN.at(6, "Intake Rollers 2"),
                1,
                false,
                false,
                false);
            default -> new IntakeRollersConfig(
                CAN.at(5, "Intake Rollers 1"),
                CAN.at(6, "Intake Rollers 2"),
                1,
                false,
                false,
                false);
        };

    public static final PIDGains GAINS =
        switch (Constants.getRobotType()) {
            case COMP -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
            case SIM ->  new PIDGains(0, 0, 0, 0, 0, 0, 0);
            default ->   new PIDGains(0, 0, 0, 0, 0, 0, 0);
        };

    public static final IntakeRollerPhysicalConstants PHYSICAL_CONSTANTS =
        switch (Constants.getRobotType()) {
            case SIM -> new IntakeRollerPhysicalConstants(0.01);
            case COMP -> new IntakeRollerPhysicalConstants(0.1);
            default -> new IntakeRollerPhysicalConstants(0.1);
        };

    public static final double UPPER_VOLT_LIMIT = 12;
    public static final double LOWER_VOLT_LIMIT = -12;
    public static final int CURRENT_LIMIT_AMPS = 1;
    public static final double STATOR_CURRENT_LIMIT = 50;

    public record IntakeRollersConfig(
        int motorID1,
        int motorID2,
        double reduction,
        boolean inverted,
        boolean brake,
        boolean opposeMotor
    ) {}

    
    public record PIDGains(
        double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

    public static record IntakeRollerPhysicalConstants(double momentOfInertia) {}
}