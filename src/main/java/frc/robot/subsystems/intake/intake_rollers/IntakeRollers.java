package frc.robot.subsystems.intake.intake_rollers;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;

public class IntakeRollers extends GenericRollers<IntakeRollers.IntakeRollersTarget> {
    public enum IntakeRollersTarget implements GenericRollers.VelocityTarget{
        IDLE(0, IntakeRollersConstants.CURRENT_LIMIT_AMPS),
        INTAKE(10, IntakeRollersConstants.CURRENT_LIMIT_AMPS);

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
    }
}