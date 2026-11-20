package frc.robot.subsystems.pivotvator.pivotvator_elevator;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class PivotvatorElevatorConstants {

  // based on ppiconstants

  public static final PivotvatorElevatorConfig PIVOTVATOR_ELEVATOR_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new PivotvatorElevatorConfig(
            CAN.at(0, "Pivotvator Elevator"),
            CAN.at(0, "Pivotvator Elevator"),
            0,
            InvertedValue.CounterClockwise_Positive,
            false);
        case SIM -> new PivotvatorElevatorConfig(
            CAN.at(74, "Pivotvator Elevator"),
            CAN.at(75, "Pivotvator Elevator"),
            1,
            InvertedValue.CounterClockwise_Positive,
            false);
        default -> new PivotvatorElevatorConfig(
            CAN.at(0, "Pivotvator Elevator"),
            CAN.at(0, "Pivotvator Elevator"),
            0,
            InvertedValue.CounterClockwise_Positive,
            false);
      };

  // put values
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        case SIM -> new PIDGains(0.4, 0, 0, 0.2, 0.137, 0, 0.61);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  // put values
  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(0, 0);
        case SIM -> new MotionMagicConfig(0, 0);
        default -> new MotionMagicConfig(0, 0);
      };

  public static final double PIVOTVATOR_HARD_STOP_ROTATIONS = 13.0;

  public static final double PIVOTVATOR_HARD_STOP_METERS =
      PIVOTVATOR_HARD_STOP_ROTATIONS * 2.0 * Math.PI * 0.1;

  public static final PivotvatorElevatorPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new PivotvatorElevatorPhysicalConstants(
            1,
            1,
            1,
            PIVOTVATOR_HARD_STOP_METERS,
            false); // TODO: Change constant values! All of them
        case COMP -> new PivotvatorElevatorPhysicalConstants(0, 0, 0, 0, false);
        default -> new PivotvatorElevatorPhysicalConstants(0, 0, 0, 0, false);
      };

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;

  public static final double POSITION_TARGET_EPSILON = 0.001;

  public static final double UPPER_VOLT_LIMIT = 16;
  public static final double LOWER_VOLT_LIMIT = -16;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS =
      switch (Constants.getRobotType()) {
        case SIM -> 40;
        default -> 40;
      };

  public static final double ZEROING_VOLTS = 3;
  public static final double ZEROING_OFFSET = 0.37;

  public record PivotvatorElevatorConfig(
      int motorID,
      int followerMotorID,
      double reduction,
      InvertedValue motorDirection,
      boolean opposeMotor) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicConfig(double acceleration, double cruiseVelocity) {}

  public static record PivotvatorElevatorPhysicalConstants(
      double massInKilograms,
      double drumRadiusMeters,
      double minExtensionMeters,
      double maxExtensionMeters,
      boolean simulateGravity) {}
}
