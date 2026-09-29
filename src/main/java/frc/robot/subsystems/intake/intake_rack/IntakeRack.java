package frc.robot.subsystems.intake.intake_rack;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIO;

public class IntakeRack extends GenericSuperstructure<IntakeRack.IntakeRackTarget> {
    public enum IntakeRackTarget implements GenericSuperstructure.PositionTarget {
        IDLE(0, IntakeRackConstants.CURRENT_LIMIT_AMPS),
        INTAKE(10, IntakeRackConstants.CURRENT_LIMIT_AMPS),
        STOW(0, IntakeRackConstants.CURRENT_LIMIT_AMPS);

        private double epsilon;
        private double position;
        private double supplyCurrentLimit;

        public double getEpsilon() {
            return epsilon;
        }

        public double getPosition() {
            return position;
        }

        public double getSupplyCurrentLimit() {
            return supplyCurrentLimit;
        }

        private IntakeRackTarget(double position, double supplyCurrentLimit) {
            this.position = position;
            this.supplyCurrentLimit = supplyCurrentLimit;
        }
    }

    public IntakeRack(GenericSuperstructureIO io) {
        super("Intake/Intake Rack", io);
    }
}
