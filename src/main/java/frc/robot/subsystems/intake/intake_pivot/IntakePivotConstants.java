package frc.robot.subsystems.intake.intake_pivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class IntakePivotConstants {
    public static final IntakePivotConfig INTAKE_PIVOT_CONFIG =
        switch (Constants.getRobotType()) {
            case COMP -> new IntakePivotConfig(
                CAN.at(34, "Intake Pivot"),
                69,
                InvertedValue.CounterClockwise_Positive,
                false);
            case SIM -> new IntakePivotConfig(
                CAN.at(52, "Intake Pivot"),
                1,
                InvertedValue.CounterClockwise_Positive,
                false);
            default -> new IntakePivotConfig(
                CAN.at(52, "Intake Pivot"),
                1,
                InvertedValue.CounterClockwise_Positive,
                false);
        };

    public static final PIDGains GAINS =
        switch (Constants.getRobotType()) {
            case COMP -> new PIDGains(81, 0, 0, 0, 8, 0, 0.5);
            case SIM ->  new PIDGains(0.4, 0, 0, 0.2, 0.137, 0, 0);
            default ->   new PIDGains(0, 0, 0, 0, 0, 0, 0);
        };

    public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
        switch (Constants.getRobotType()) {
            case COMP -> new MotionMagicConfig(0, 9);
            case SIM -> new MotionMagicConfig(400, 40);
            default -> new MotionMagicConfig(0, 0);
        };

    /** Sim travel bound in mechanism rotations, above all observed real travel (11.68 in q93). */
    // I left this here to make it easier to read.
    public static final double PIVOT_HARD_STOP_ROTATIONS = 13.0;
    private static final double PIVOT_HARD_STOP_METERS =
        PIVOT_HARD_STOP_ROTATIONS * 2.0 * Math.PI * 0.1;
    public static final IntakePivotPhysicalConstants PHYSICAL_CONSTANTS =
        switch (Constants.getRobotType()) {
            case SIM -> new IntakePivotPhysicalConstants(0.1, 0.1, 0, PIVOT_HARD_STOP_METERS, false);
            case COMP -> new IntakePivotPhysicalConstants(0.1, 0, 0, 0, false);
            default -> new IntakePivotPhysicalConstants(0.1, 0, 0, 0, false);
        };

    public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

    public static final double POSITION_TARGET_EPSILON = 3;

    public static final double UPPER_VOLT_LIMIT = 0;
    public static final double LOWER_VOLT_LIMIT = 0;
    public static final double SUPPLY_CURRENT_LIMIT_AMPS =
        switch (Constants.getRobotType()) {
            case SIM -> 40;
            default -> 40;
        };

    public static final double ZEROING_VOLTS = -3;
    public static final double ZEROING_OFFSET = 0;

    public static final Transform3d BASE_TO_INTAKE_PIVOT_TRANSFORM =
        switch (Constants.getRobotType()) {
            default -> new Pose3d()
                .plus(
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(0),
                            Units.inchesToMeters(9.990),
                            Units.inchesToMeters(7.709)),
                        new Rotation3d(0, 0, 0)))
                .rotateBy(new Rotation3d(0, 0, Math.toRadians(90)))
                .minus(new Pose3d());
        };

    public record IntakePivotConfig(
        int motorID,
        double reduction,
        InvertedValue motorDirection,
        boolean opposeMotor
    ) {}

    public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

    public record MotionMagicConfig(
      double acceleration, double cruiseVelocity) {}

      public static record IntakePivotPhysicalConstants(
      double massInKilograms,
      double drumRadiusMeters,
      double minExtensionMeters,
      double maxExtensionMeters,
      boolean simulateGravity) {}

    
}
