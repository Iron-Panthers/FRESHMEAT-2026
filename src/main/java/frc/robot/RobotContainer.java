// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.VibrateHIDCommand;
import frc.robot.commands.VisionTuningCommands;
import frc.robot.commands.WaitUnitlRobotStuckCommand;
import frc.robot.subsystems.can_watchdog.CANWatchdog;
import frc.robot.subsystems.can_watchdog.CANWatchdogIO;
import frc.robot.subsystems.elastic_updater.ElasticUpdater;
import frc.robot.subsystems.rgb.RGB;
import frc.robot.subsystems.rgb.RGBIO;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFXReal;
import frc.robot.subsystems.swerve.ModuleIOTalonFXSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonvision;
import frc.robot.subsystems.vision.VisionIOPhotonvisionSim;
import frc.robot.utility.ElasticSetpoints;
import frc.robot.utility.SimBattery;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // DO NOT DELETE -- this actually does something important
  private RobotState robotState = RobotState.getInstance();

  // DO NOT DELETE -- storing the instance here registers @AutoLogOutput methods with AdvantageKit
  private RobotSimState robotSimState;

  private ElasticSetpoints elasticSetpoints = ElasticSetpoints.getInstance();

  private ElasticUpdater matchTimerUpdater = new ElasticUpdater();

  private boolean defaultZeroing = true;

  // private SendableChooser<Command> autoChooser;
  private LoggedDashboardChooser<Command> autoChooser;

  private final CommandXboxController driverA = new CommandXboxController(0);
  private final CommandXboxController driverB = new CommandXboxController(1);

  private Drive swerve;
  private Vision vision;
  private RGB rgb;
  private CANWatchdog canWatchdog;

  private SwerveDriveSimulation driveSimulation = null;

  public RobotContainer() {

    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case COMP -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[3]));
          //   vision = new Vision(new VisionIOPhotonvision(4), new VisionIOPhotonvision(5));
          // rgb = new RGB(new RGBIOCANdle());
          // canWatchdog = new CANWatchdog(new CANWatchdogIOComp(), rgb);
          vision =
              new Vision(
                  new VisionIOPhotonvision("camB", 0),
                  new VisionIOPhotonvision("camA", 1),
                  new VisionIOPhotonvision("camC", 2));
        }
        case VISION -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[3]));
          // vision = new Vision(new VisionIOPhotonvision("arducam-4", 0), new
          // VisionIOPhotonvision("arducam-5", 1));
        }
        case SIM -> {
          robotSimState = RobotSimState.getInstance();
          // RobotSimState must be initialized first — its constructor calls
          // SimulatedArena.overrideInstance(), so any arena access before this point
          // would get orphaned. We reuse its driveSimulation so both share the same arena.
          driveSimulation = RobotSimState.getInstance().getDriveSimulation();
          swerve =
              new Drive(
                  new GyroIOSim(driveSimulation.getGyroSimulation()),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[0], driveSimulation.getModules()[0]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[1], driveSimulation.getModules()[1]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[2], driveSimulation.getModules()[2]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[3], driveSimulation.getModules()[3]));
          vision =
              new Vision(
                  new VisionIOPhotonvisionSim(
                      "arducam-3", 1, driveSimulation::getSimulatedDriveTrainPose));
          new VisionIOPhotonvisionSim("arducam-4", 2, driveSimulation::getSimulatedDriveTrainPose);
          SimulatedArena.getInstance().resetFieldForAuto();
        }
      }
    }

    if (swerve == null) {
      swerve =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      vision = new Vision(new VisionIO() {}, new VisionIO() {});
    }

    if (canWatchdog == null) {
      canWatchdog = new CANWatchdog(new CANWatchdogIO() {}, rgb);
    }

    if (rgb == null) {
      rgb = new RGB(new RGBIO() {});
    }

    nameCommands();
    configureAutos();
    configureBindings();
  }

  public void containerMatchStarting() {
    // runs when match starts
    canWatchdog.matchStarting();
  }

  /** Use this method to define the named commands for all of the autos */
  private void nameCommands() {
    Supplier<Pose2d> shootingPoseSupplier =
        () ->
            ((autoChooser == null ? false : autoChooser.get().getName().contains("Right")))
                ? new Pose2d(2.901, 2.523, new Rotation2d(-135.000 * Math.PI / 180))
                : new Pose2d(
                    2.901,
                    FlippingUtil.fieldSizeY - 2.523,
                    new Rotation2d((135.000) * Math.PI / 180));

    NamedCommands.registerCommand(
        "Check If Off", new WaitUnitlRobotStuckCommand(swerve, shootingPoseSupplier));
    Supplier<Pose2d> flippedShootingPoseSupplier =
        () -> {
          return RobotState.isAllianceRed()
              ? FlippingUtil.flipFieldPose(shootingPoseSupplier.get())
              : shootingPoseSupplier.get();
        };
    NamedCommands.registerCommand("Smart zero", new InstantCommand(() -> swerve.smartZeroGyro()));
  }

  private void configureBindings() {
    // -----Driver Controls-----
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  swerve.driveTeleopController(
                      -driverA.getLeftY(),
                      -driverA.getLeftX(),
                      driverA.getLeftTriggerAxis() - driverA.getRightTriggerAxis(),
                      DriveConstants.DRIVE_CONFIG.maxLinearAcceleration());
                  if ((Math.abs(driverA.getLeftTriggerAxis()) > 0.1
                      || Math.abs(driverA.getRightTriggerAxis()) > 0.1)) {
                    swerve.clearHeadingControl();
                  }
                })
            .withName("Drive Teleop"));
    // adjust this for x swerve
    configureDriverAButtons();
  }

  private void configureDriverAButtons() {

    driverA
        .start()
        .onTrue(
            swerve.zeroGyroCommand().alongWith(new InstantCommand(() -> defaultZeroing = true)));
    // driverA.povUp().onTrue(new LowPowerModeCommand(swerve));

  }

  private void configureAutos() {
    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      robotConfig = null;
    }

    var passRobotConfig = robotConfig; // workaround TODO: is it necessary?

    if (passRobotConfig != null) {
      AutoBuilder.configure(
          () -> RobotState.getInstance().getEstimatedPose(),
          (pose) -> RobotState.getInstance().resetPose(pose),
          () -> swerve.getRobotSpeeds(),
          (speeds) -> {
            swerve.setTrajectorySpeeds(speeds);
          },
          DriveConstants.HOLONOMIC_DRIVE_CONTROLLER,
          passRobotConfig,
          () -> RobotState.isAllianceRed(),
          swerve);
    } else {
      System.err.println(
          "[RobotContainer] RobotConfig failed to load — AutoBuilder not configured, autos"
              + " disabled.");
    }

    autoChooser =
        new LoggedDashboardChooser<Command>("Auto Chooser", AutoBuilder.buildAutoChooser());
    VisionTuningCommands.addTuningCommandsToAutoChooser(vision, autoChooser);
    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());
  }

  public Command getAutoCommand() {
    return autoChooser.get(); // HACK: Replace once we get auto logging
  }

  // runs when auto starts
  public void autoInit() {
    RobotState.getInstance().setIsAutoAdaptive(false);
    // Smart zero the robot
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> swerve.smartZeroGyro()));
  }

  // runs when teleop starts
  public void teleopInit() {
    CommandScheduler.getInstance().schedule(new VibrateHIDCommand(driverB.getHID(), 5, .5));
    swerve.setNeutralMode(NeutralModeValue.Brake);
  }

  private double testModeStartTime = -1;
  private final java.util.ArrayList<String> combinedTestData = new java.util.ArrayList<>();

  public void testInit() {

    // Insert command sequence for test mode

    testModeStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    timeOfLastTransition = testModeStartTime;
  }

  private double timeOfLastTransition = 0;

  public void testPeriodic() {
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    boolean isNextTransition = false;
    if (currentTime - testModeStartTime >= 25) {
      return;
    }
    if (currentTime - timeOfLastTransition >= 5) {
      isNextTransition = true;
      timeOfLastTransition = currentTime;
    }

    if (isNextTransition) {
      double relativeTime = currentTime - testModeStartTime;
      combinedTestData.add(String.format("%.1f", relativeTime) + "Insert test mode data here!");
    }
  }

  public void testExit() {
    printAndClearStoredData();
  }

  /** Ran when periodic disabled */
  public void updateDashboardStatus() {
    // TODO: Define all of the dashboard outputs here
    var selectedAuto = autoChooser.get();
    SmartDashboard.putString(
        "Current Auto", selectedAuto != null ? selectedAuto.getName() : "None");
  }

  public static double doubleToDegrees(double angle) {
    return (angle % 360 + 360) % 360;
  }

  public static double relativeAngularDifference(double currentAngle, double newAngle) {
    return (doubleToDegrees(newAngle - currentAngle) + 180) % 360 - 180;
  }

  /** Ran every 20 milliseconds */
  public void updateSimulation() {
    if (Constants.getRobotMode() != Constants.Mode.SIM) return;
    SimulatedArena.getInstance().simulationPeriodic();
    SimBattery.getInstance().update();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/ObstaclePositions",
        RobotSimState.getInstance().getObstaclePositions().toArray(new Pose2d[0]));
  }

  public void printAndClearStoredData() {
    System.out.println("=== Test Mode Combined Data ===");
    System.out.println(
        "Timestamp(s),Ser AvgRad/s,Ser AvgCurrent(A),Acc AvgRad/s,Acc AvgCurrent(A),Rack"
            + " AvgRot/s,Rack AvgCurrent(A),Rollers AvgRad/s,Rollers AvgCurrent(A)");
    for (String row : combinedTestData) {
      System.out.println(row);
    }
    combinedTestData.clear();
    // Reset test state for subsystems with a test state
  }
}
