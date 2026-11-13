package frc.robot.subsystems.pivotvater.pivotvater_pivot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class PivotvaterPivotConstants {
    public static final PivotvaterConfig PIVOTVATER_CONFIG = 
        switch (Constants.getRobotType()) {
            case COMP -> new PivotvaterConfig(
                CAN.at(0, "Pivotvater"), 0, InvertedValue.CounterClockwise_Positive, false);
            case SIM -> new PivotvaterConfig(
                CAN.at(0, "Pivotvater"), 0, InvertedValue.CounterClockwise_Positive, false);
            default -> new PivotvaterConfig(
                CAN.at(0, "Pivotvater"), 0, InvertedValue.CounterClockwise_Positive, false);
        };

    public static final PIDGains GAINS = 
        switch (Constants.getRobotType()) {
            case COMP -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
            case SIM -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
            default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        };

    public static final MotionMagicConfig MOTION_MAGIC_CONFIG = 
        switch(Constants.getRobotType()) {
            case COMP -> new MotionMagicConfig(0, 0);
            case SIM -> new MotionMagicConfig(0, 0);
            default -> new MotionMagicConfig(0, 0);
        };

    public static final double PIVOTVATER_HARD_STOP_ROTATIONS = 13.0;

    public static final double PIVOTVATER_HARD_STOP_METERS = 
        PIVOTVATER_HARD_STOP_ROTATIONS * 2.0 * Math.PI * 0.1;
    public static final PivotvaterPhysicalConstants PHYSICAL_CONSTANTS =
        switch (Constants.getRobotType()) {
            case SIM -> new PivotvaterPhysicalConstants(0, 0, 0, PIVOTVATER_HARD_STOP_METERS, false);
            case COMP -> new PivotvaterPhysicalConstants(0, 0, 0, 0, false);
            default -> new PivotvaterPhysicalConstants(0, 0, 0, 0, false);
        }; 

    public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

    public static final double POSITION_TARGET_EPSILON = 0.001;
    
    public static final double UPPER_VOLT_LIMIT = 16;
    public static final double LOWER_VOLT_LIMIT = -16;
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 
        switch(Constants.getRobotType()) {
            case SIM -> 40;
            default -> 40;
        };

    public static final double ZEROING_VOLTS = 3;
    public static final double ZEROING_OFFSET = 0.37;

    

    
     public record PivotvaterConfig(
        int motorID, double reduction, InvertedValue motorDirection, boolean opposeMotor) {}

     public record PIDGains(
        double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
     public record MotionMagicConfig(double acceleration, double cruiseVelocity) {} // TODO: find thingies through tuning, idk im a freshie
    
     public static record PivotvaterPhysicalConstants(
        double massInKilograms,
        double drumRadiusMeters,
        double minExtensionMeters,
        double maxExtensionMeters,
        boolean simulateGravity) {}

     
}
