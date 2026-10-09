package frc.robot.subsystems.shooter.shooter_accelerator;

import static edu.wpi.first.units.Units.Newton;

import com.pathplanner.lib.config.PIDConstants;
import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class ShooterAcceleratorConstants {
    public static final ShooterAcceleratorConfig SHOOTER_ACCELERATOR_CONFIG =
    switch(Constants.getRobotType()) { 
        case SIM -> new ShooterAcceleratorConfig( 
            CAN.at(38, "Shooter Accelerator 1"), 
            CAN.at(39, "Shooter Accelerator 2"), // Matches COMP. 
            1.5, 
            true, 
            false, 
            true
        ); 
        case COMP -> new ShooterAcceleratorConfig( 
            CAN.at(38, "Shooter Accelerator 1"), 
            CAN.at(39, "Shooter Accelerator 2"), // Matches COMP. 
            1.5, 
            true, 
            false, 
            true
        ); 
        default -> new ShooterAcceleratorConfig( 
            CAN.at(25, "Shooter Accelerator Left"), 
            CAN.at(13, "Shooter Accelerator Right"), 
            1.5, 
            true, 
            true, 
            true
        );
    };

    public static final PIDGains GAINS =
    switch(Constants.getRobotType()){
        case SIM -> new PIDGains(1,1,1,1,1,1,1);//placeholder value
        case COMP -> new PIDGains(1,1,1,1,1,1,1);//placeholder value
        default -> new PIDGains(1,1,1,1,1,1,1);//placeholder value
    };

    public static final ShooterAcceleratorPhysicalConstants PHYSICAL_CONSTANTS =
    switch(Constants.getRobotType()){
        case SIM -> new ShooterAcceleratorPhysicalConstants(1);//placeholder value
        case COMP -> new ShooterAcceleratorPhysicalConstants(1);//placeholder value
        default -> new ShooterAcceleratorPhysicalConstants (1);//placeholder value
    };
    
    public static final int CURRENT_LIMIT_AMPS = 50;
    public record ShooterAcceleratorConfig (
        int motorID1,
        int motorID2,
        double reduction,
        boolean inverted,
        boolean brake,
        boolean oppose_motor) {}
    public record PIDGains (
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA,
        double kG) {}
    public record ShooterAcceleratorPhysicalConstants(double momentOfInertia){}
     

}
