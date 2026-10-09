package frc.robot.subsystems.shooter.shooter_rollers;

import static frc.robot.subsystems.shooter.shooter_rollers.ShooterRollersConstants.*;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim.RollerSim;

public class ShooterRollersIOSim extends GenericRollersIOSim implements ShooterRollersIO {
    private final FlywheelSim shooterFlywheelsSim;
    private final SimpleMotorFeedforward feedforward;
    private double rotorPositionRotations = 0.0;
    private double velocitySetpointRPS = 0.0;

    public ShooterRollersIOSim() {
        super(
            SHOOTER_ROLLERS_CONFIG.motorID1(),
            CURRENT_LIMIT_AMPS,
            SHOOTER_ROLLERS_CONFIG.inverted(),
            SHOOTER_ROLLERS_CONFIG.brake(),
            SHOOTER_ROLLERS_CONFIG.reduction(),
            new RollerSim(4, 4, PHYSICAL_CONSTANTS.momentOfInertia(), 0.0341));
        super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
        feedforward = new SimpleMotorFeedforward(GAINS.kS(), GAINS.kV(), GAINS.kA());

        shooterFlywheelsSim =
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60Foc(1),
                    PHYSICAL_CONSTANTS.momentOfInertia(),
                    SHOOTER_ROLLERS_CONFIG.reduction()),
                DCMotor.getKrakenX60Foc(1));

        var simState = talon.getSimState();
        simState.Orientation =
            SHOOTER_ROLLERS_CONFIG.inverted()
                ? ChassisReference.Clockwise_Positive
                : ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void runVelocity(double velocity) {
        velocitySetpointRPS = velocity;
        super.runVelocity(velocity);
    }

    @Override
    public void updateInputs(GenericRollersIOInputs inputs) {
        double currentVelocityRPS = shooterFlywheelsSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

        talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        talon.getSimState().setRawRotorPosition(rotorPositionRotations);
        talon.getSimState().setRotorVelocity(currentVelocityRPS);

        double feedforwardVoltage = feedforward.calculate(velocitySetpointRPS);
        double error = velocitySetpointRPS - currentVelocityRPS;
        double proportionalVoltage = GAINS.kP() * error;
        double appliedVoltage = Math.max(-12, Math.min(12, feedforwardVoltage + proportionalVoltage));

        shooterFlywheelsSim.setInputVoltage(appliedVoltage);
        shooterFlywheelsSim.update(0.02);

        currentVelocityRPS = shooterFlywheelsSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        rotorPositionRotations += currentVelocityRPS * 0.02;

        inputs.connected = true;
        inputs.positionRads = rotorPositionRotations * 2.0 * Math.PI;
        inputs.velocityRadsPerSec = shooterFlywheelsSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVoltage;
        inputs.supplyCurrentAmps = Math.abs(shooterFlywheelsSim.getCurrentDrawAmps());
    }
}
