package frc.robot.subsystems.shooter.serializer;
import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class SerializerConstants {
    public static final SerializerConfig SERIALIZER_CONFIG =
        //TODO: get actual serializer constants
        switch (Constants.getRobotType()) {
        case SIM -> new SerializerConfig(
            CAN.at(25, "Serializer Right"),
            CAN.at(24, "Serializer Left"),
            1,
            true,
            false,
            true);
        case COMP -> new SerializerConfig(
            CAN.at(25, "Serializer Right"),
            CAN.at(24, "Serializer Left"),
            1,
            true,
            false,
            true);
        default -> new SerializerConfig(
            CAN.at(25, "Serializer Right"),
            CAN.at(24, "Serializer Left"),
            1,
            true,
            false,
            true);
        };

    public static final PIDGains GAINS =
    switch (Constants.getRobotType()) {
        case SIM ->  new PIDGains(0.4, 0, 0, 0.2, 0.137, 0, 0);
        case COMP -> new PIDGains(0.1, 0, 0, 0.2, 0.1164, 0.321, 0);
        default ->   new PIDGains(0.4, 0, 0, 0.2, 0.137, 0, 0);
    };

    public static final double UPPER_VOLT_LIMIT = 12;
    public static final double LOWER_VOLT_LIMIT = -12;
    public static final int CURRENT_LIMIT_AMPS = 35;
    public static final int STATOR_CURRENT_LIMIT = 50;


    public static final SerializerPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new SerializerPhysicalConstants(0.000105);
        default -> new SerializerPhysicalConstants(0.000105);
      };

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
    
    public static record SerializerPhysicalConstants(double momentOfInertia) {}
}
