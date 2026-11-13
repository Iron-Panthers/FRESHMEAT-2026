package frc.robot.subsystems.pivotvater.pivotvater_elevator;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;

/*
 * 
 * For freshmen: look at OFFSEASON-2025 code ->
 * https://github.com/Iron-Panthers/FRC-2025-Sprint/blob/dev/src/main/java/frc/robot/subsystems/superstructure/elevator/Elevator.java
 * 
 * Some things will be a little different, but we believe you can do it. Try your best!
 * 
 */

public class PivotvaterElevator extends GenericSuperstructure<PivotvaterElevator.PivotvaterElevatorTarget> {
    public enum PivotvaterElevatorTarget implements GenericSuperstructure.PositionTarget {
        FULL_EXTEND(1),
        CENTER_POINT(0.5),
        LOW(0.25),
        NONE(0);
        private double position = 0;
        private double supplyCurrentLimit;

        private static final double EPSILON = PivotvaterElevatorConstants.POSITION_TARGET_EPSILON;

        private PivotvaterElevatorTarget(double position) {
            this.position = position;
        }
        
        public double getPosition() {
            return position;
        }

        public double getEpsilon() {
            return EPSILON;
        }

        public double getSupplyCurrentLimit() {
            return supplyCurrentLimit;
        }


    }
    private boolean zeroing = false;
    private double filteredSupplyCurrentAmps = 0;
    private final LinearFilter supplyCurrentFilter;
    //private GenericSuperstructureIOInputsMotorAuroLogged inputs = new GenericSuperstructureIOInputsMotorAuroLogged();

    public PivotvaterElevator(PivotvaterElevatorIO io) {
        super("PivotvaterElevator", io);
        setPositionTarget(PivotvaterElevatorTarget.NONE);
        setControlMode(ControlMode.STOP);
        supplyCurrentFilter = LinearFilter.movingAverage(30);
    }
    public void periodic() {
        super.periodic();

        if (zeroing)  {
            superstructureIO.runCharacterization();
            
        }
        filteredSupplyCurrentAmps = supplyCurrentFilter.calculate(getSupplyCurrentAmps());



    }
    public double getFilteredSupplyCurrentAmps() {
        return filteredSupplyCurrentAmps;
    }
    public double getSupplyCurrentAmps() {
        return supplyCurrentFilter;
    }
    public void setZeroing(boolean zeroing) {
        this.zeroing = zeroing;
    }
    public boolean isZeroing() {
        return zeroing;
    }
}
    /*
     * Position target enum
     */

     // Constructor

     // setPositionTarget (to reset pid)

     // periodic

