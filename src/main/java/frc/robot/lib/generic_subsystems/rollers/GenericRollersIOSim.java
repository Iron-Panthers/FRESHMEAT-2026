package frc.robot.lib.generic_subsystems.rollers;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.MotorOutputManager;
import frc.robot.utility.SimBattery;

public abstract class GenericRollersIOSim implements GenericRollersIO {

  public record RollerSim(
    int plantMotors, int packMotors, double momentOfInertia, double dragAmpsPerRadPerSec){}

  protected final TalonFX talon;

  private final NeutralOut neutralOutput = new NeutralOut();
  private final double mechanismReduction;
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0);
  private final RollerSim sim;
  private double packSupplyCurrentAmps = 0.0;

  public GenericRollersIOSim(
      int id, int currentLimitAmps, boolean inverted, boolean brake, double reduction, RollerSim sim) {
    talon = new TalonFX(id);

    mechanismReduction = reduction;
    this.sim = sim;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    talon.optimizeBusUtilization();

    // The real IO registers with both; without these sim pack current omitted every mechanism.
    MotorOutputManager.getInstance().registerMotorOutputs(() -> packSupplyCurrentAmps);
    SimBattery.getInstance()
        .register(() -> packSupplyCurrentAmps, currentLimitAmps * sim.packMotors());
  }

  @Override
  public abstract void updateInputs(GenericRollersIOInputs inputs);

  @Override
  public void runVelocity(double velocity) {
    talon.setControl(velocityControl.withVelocity(velocity));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOutput);
  }

  /**
   * Sets all of the PID and motion magic gains.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kS Static gain
   * @param kV Velocity gain
   * @param kA Acceleration gain
   * @param kG Gravity gain
   * @param gravityTypeValue Gravity compensation type
   */
  @Override
  public void setSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {
    Slot0Configs gainsConfig = new Slot0Configs();
    gainsConfig.kP = kP;
    gainsConfig.kI = kI;
    gainsConfig.kD = kD;
    gainsConfig.kS = kS;
    gainsConfig.kV = kV;
    gainsConfig.kA = kA;

    talon.getConfigurator().apply(gainsConfig);
  }
}
