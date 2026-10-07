package frc.robot.subsystems.shooter_accelerator;

import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class ShooterAcceleratorConstants {
    private static final  String ShooterAcceleratorConfig = null;
    
    
                ShooterAccelerator accelerator = new ShooterAccelerator(ShooterAcceleratorConstants.ShooterAcceleratorConfig);
    public static final ShooterAccelerator SHOOTER_ACCELERATOR =
 switch(Constants.getRobotType()) { 
    case SIM -> new ShooterAccelerator( 
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
        case default -> new PIDGains(1,1,1,1,1,1,1);//placeholder value
         };

    public static final ShooterAcceleratorPhysicalConstants PHYSICAL_CONSTANTS =
    switch(Constants.getRobotType()){
        case SIM -> new PhysicalConstants(1);//placeholder value
        case COMP -> new PhysicalConstants(1);//placeholder value
        case default -> new PhysicalConstants(1);//placeholder value
    };
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
