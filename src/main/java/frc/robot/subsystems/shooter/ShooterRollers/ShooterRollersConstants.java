package frc.robot.subsystems.shooter.ShooterRollers;

import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class ShooterRollersConstants {
  public static final ShooterRollersConfig SHOOTER_ROLLERS_CONFIG =
      switch (Constants.getRobotType()) {
          // TODO update motor id for 3 and 4
        case SIM -> new ShooterRollersConfig(
            CAN.at(36, "Shooter Roller 1"),
            CAN.at(37, "Shooter Roller 2"),
            CAN.at(0, "Shooter Roller 3"),
            CAN.at(0, "Shooter Roller 4"),
            0.71,
            false,
            false,
            true,
            false,
            false);
        default -> new ShooterRollersConfig(
            CAN.at(18, "Shooter Roller Left Bottom"),
            CAN.at(10, "Shooter Roller Left Top"),
            CAN.at(16, "Shooter Roller Right Bottom"),
            CAN.at(15, "Shooter Roller Right Top"),
            1.411,
            true,
            false,
            false,
            true,
            true);
      };

  // CONTROL LOOP GAINS AND MOTION MAGIC CONFIG
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case SIM -> new PIDGains(3, 0, 0, 0, .1, 0, 0);
        default -> new PIDGains(0.5, 0, 0, 0.2, 0.35, 0, 0);
      };

  public static final double VELOCITY_ADJUSTMENT = 0.98;
  public static final int CURRENT_LIMIT_AMPS =
      switch (Constants.getRobotType()) {
        case SIM -> 40;
        default -> 20;
      };

  public static final ShooterRollersPhysicalConstants PHYSICAL_CONSTANTS = // TODO: update values
      switch (Constants.getRobotType()) {
        case SIM -> new ShooterRollersPhysicalConstants(0.01, 0.23938936);
        default -> new ShooterRollersPhysicalConstants(0.1, 0.23938936);
      };

  // RECORDS
  public record ShooterRollersConfig(
      int motorID1,
      int motorID2,
      int motorID3,
      int motorID4,
      double reduction,
      boolean inverted,
      boolean brake,
      boolean opposeMotor1,
      boolean opposeMotor2,
      boolean opposeMotor3) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static record ShooterRollersPhysicalConstants(
      double momentOfInertia, double circumferenceMeters) {}
}

