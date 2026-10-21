package frc.robot.subsystems.shooter.serializer;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;

public class Serializer extends GenericRollers<Serializer.SerializerTarget> {
    // for logging the VelocityRadsPerSec and SupplyCurrentAmps every 5 seconds in test mode
    private final java.util.ArrayList<Double> speedIntervalReadings = new java.util.ArrayList<>();
    private final java.util.ArrayList<Double> supplyCurrentIntervalReadings = new java.util.ArrayList<>();

    public enum SerializerTarget implements GenericRollers.VelocityTarget{
        IDLE(0, SerializerConstants.CURRENT_LIMIT_AMPS),
        SHOOT(10, SerializerConstants.CURRENT_LIMIT_AMPS),
        INTAKE(-10, SerializerConstants.CURRENT_LIMIT_AMPS);

        private double velocity;
        private double supplyCurrentLimit;

        public double getVelocity() {
            return velocity;
        }

        public double getSupplyCurrentLimit() {
            return supplyCurrentLimit;
        }

        private SerializerTarget(double velocity, double supplyCurrentLimit) {
            this.velocity = velocity;
            this.supplyCurrentLimit = supplyCurrentLimit;
        }
    }

    public Serializer(GenericRollersIO io) {
        super("Serializer", io);
        setVelocityTarget(SerializerTarget.IDLE);
    }

    public double getVelocityRadsPerSec() {
        return inputs.velocityRadsPerSec;
    }

    /**
     * Returns true when the serializer is applying amps but not going anywhere
     *
     * @return
    */
    @AutoLogOutput(key = "Serializer/Serializer Stalling")
    public boolean serializerStalling() {
        return getFilteredCurrent() > 15d && getVelocityRadsPerSec() < 3d;
    }

    public static void setTargetState(SerializerTarget shoot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetState'");
    }

    public void resetTestState() {
        speedIntervalReadings.clear();
        supplyCurrentIntervalReadings.clear();
    }


    // Returns "avgRad/s,avgCurrent" at each 5s transition, null otherwise
    public String runTestLogger(double testStartTime, double currentTime, boolean isNextTransition) {
        if (testStartTime == -1) {
        testStartTime = currentTime;
        }

        double currentSpeed = inputs.velocityRadsPerSec;
        double currentSupplyCurrent = inputs.supplyCurrentAmps;
        speedIntervalReadings.add(currentSpeed);
        supplyCurrentIntervalReadings.add(currentSupplyCurrent);

        if (isNextTransition) {
        double sumSpeed = 0;
        double sumSupplyCurrent = 0;
        for (double val : speedIntervalReadings) {
            sumSpeed += val;
        }
        for (double val : supplyCurrentIntervalReadings) {
            sumSupplyCurrent += val;
        }

        double avgSpeed = speedIntervalReadings.isEmpty() ? 0 : sumSpeed / speedIntervalReadings.size();
        double avgSupplyCurrent = supplyCurrentIntervalReadings.isEmpty() ? 0 : sumSupplyCurrent / supplyCurrentIntervalReadings.size();

        speedIntervalReadings.clear();
        supplyCurrentIntervalReadings.clear();

        return String.format("%.2f", avgSpeed) + "," + String.format("%.2f", avgSupplyCurrent);
        }
        return null;
    }
}
