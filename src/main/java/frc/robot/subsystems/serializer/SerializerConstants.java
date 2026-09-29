package frc.robot.subsystems.serializer;

import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class SerializerConstants {
    public static final SerializerConfig SERIALIZER_CONFIG = new SerializerConfig(
        //TODO: get actual serializer constants
        CAN.at(1, "Serializer 1"),
        CAN.at(2, "Serializer 2"),
        1,
        false,
        false,
        false);

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
}
