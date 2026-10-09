package frc.robot.subsystems.shooter.serializer;

import org.littletonrobotics.junction.AutoLogOutput;

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
}
