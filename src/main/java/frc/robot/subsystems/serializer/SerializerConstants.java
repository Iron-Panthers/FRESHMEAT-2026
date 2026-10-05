package frc.robot.subsystems.serializer;
import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class SerializerConstants {
    public static final SerializerConfig SERIALIZER_CONFIG =
        //TODO: get actual serializer constants
        switch (Constants.getRobotType()) {
        case SIM -> new SerializerConfig(
            CAN.at(1, "Serializer 1"),
            CAN.at(2, "Serializer 2"),
            1,
            false,
            false,
            false);
        case COMP -> new SerializerConfig(
            CAN.at(1, "Serializer 1"),
            CAN.at(2, "Serializer 2"),
            1,
            false,
            false,
            false);
        default -> new SerializerConfig(
            CAN.at(1, "Serializer 1"),
            CAN.at(2, "Serializer 2"),
            1,
            false,
            false,
            false);
        };

    public static final PIDGains GAINS =
        switch (Constants.getRobotType()) {
            case SIM ->  new PIDGains(0, 0, 0, 0, 0, 0, 0);
            case COMP -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
            default ->   new PIDGains(0, 0, 0, 0, 0, 0, 0);
        };

    public static final double UPPER_VOLT_LIMIT = 12;
    public static final double LOWER_VOLT_LIMIT = -12;
    public static final int CURRENT_LIMIT_AMPS = 1;
    public static final int STATOR_CURRENT_LIMIT = 1;

    public record SerializerConfig (
        int motorID1,
        int motorID2,
        double reduction,
        boolean inverted,
        boolean brake,
        boolean opposeMotor
    ) {}

    public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
