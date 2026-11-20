package frc.robot.subsystems.pivotvator.pivotvator_pivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class PivotvatorPivotConstants {
  public static final PivotvatorPivotConfig PIVOTVATOR_PIVOT_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new PivotvatorPivotConfig(
            CAN.at(0, "Pivotvator Pivot"),
            CAN.at(0, "Pivotvator Pivot"),
            0,
            InvertedValue.CounterClockwise_Positive,
            false);
        case SIM -> new PivotvatorPivotConfig(
            CAN.at(76, "Pivotvator Pivot"),
            CAN.at(77, "Pivotvator Pivot"),
            1,
            InvertedValue.CounterClockwise_Positive,
            false);
        default -> new PivotvatorPivotConfig(
            CAN.at(0, "Pivotvator Pivot"),
            CAN.at(0, "Pivotvator Pivot"),
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
  public static final PivotvatorPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new PivotvatorPhysicalConstants(1, 1, 0, PIVOTVATOR_HARD_STOP_METERS, false);
        case COMP -> new PivotvatorPhysicalConstants(0, 0, 0, 0, false);
        default -> new PivotvatorPhysicalConstants(0, 0, 0, 0, false);
      };

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final Transform3d BASE_TO_PIVOTVATOR_PIVOT_TRANSFORM = new Transform3d();

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

  public record PivotvatorPivotConfig(
      int motorID,
      int followerMotorID,
      double reduction,
      InvertedValue motorDirection,
      boolean opposeMotor) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicConfig(double acceleration, double cruiseVelocity) {}

  public static record PivotvatorPhysicalConstants(
      double massInKilograms,
      double drumRadiusMeters,
      double minExtensionMeters,
      double maxExtensionMeters,
      boolean simulateGravity) {}
}
