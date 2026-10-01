package frc.robot.subsystems.serializer;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;

public class Serializer extends GenericRollers<Serializer.SerializerTarget> {
    public enum SerializerTarget implements GenericRollers.VelocityTarget{
        IDLE(0, SerializerConstants.CURRENT_LIMIT_AMPS),
        SHOOT(10, SerializerConstants.CURRENT_LIMIT_AMPS);

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
}
