package frc.robot.subsystems.intake.intake_rollers;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;

public class IntakeRollers extends GenericRollers<IntakeRollers.IntakeRollersTarget> {
    private final java.util.ArrayList<Double> speedIntervalReadings = new java.util.ArrayList<>();
    private final java.util.ArrayList<Double> supplyCurrentIntervalReadings = new java.util.ArrayList<>();

    public enum IntakeRollersTarget implements GenericRollers.VelocityTarget{
        IDLE(0, IntakeRollersConstants.CURRENT_LIMIT_AMPS),
        INTAKE(10, IntakeRollersConstants.CURRENT_LIMIT_AMPS),
        SHOOT(0, IntakeRollersConstants.CURRENT_LIMIT_AMPS), //replace with actual value
        REVERSE(-10, IntakeRollersConstants.CURRENT_LIMIT_AMPS);
        
        private double velocity;
        private double supplyCurrentLimit;

        private IntakeRollersTarget(double velocity, double supplyCurrentLimit) {
            this.velocity = velocity;
            this.supplyCurrentLimit = supplyCurrentLimit;
        }

        public double getVelocity() {
            return velocity;
        }

        public double getSupplyCurrentLimit() {
            return supplyCurrentLimit;
        }
    }

    public IntakeRollers(GenericRollersIO io) {
        super("Intake/Intake Rollers", io);
        setVelocityTarget(IntakeRollersTarget.IDLE);
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