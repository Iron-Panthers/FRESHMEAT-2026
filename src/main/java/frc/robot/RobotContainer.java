// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.AlignToPassCommand;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.commands.AlignToShootCommand;
import frc.robot.commands.AlignToTrenchCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LowPowerModeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.VibrateHIDCommand;
import frc.robot.commands.VisionTuningCommands;
import frc.robot.commands.WaitUnitlRobotStuckCommand;
import frc.robot.subsystems.can_watchdog.CANWatchdog;
import frc.robot.subsystems.can_watchdog.CANWatchdogIO;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.intake.intake_rack.IntakeRack;
import frc.robot.subsystems.intake.intake_rack.IntakeRackIO;
import frc.robot.subsystems.intake.intake_rack.IntakeRackIOSim;
import frc.robot.subsystems.intake.intake_rack.IntakeRackIOTalonFX;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollersIOSim;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.rgb.RGB;
import frc.robot.subsystems.rgb.RGBIO;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.shooter.serializer.Serializer;
import frc.robot.subsystems.shooter.serializer.SerializerIO;
import frc.robot.subsystems.shooter.serializer.SerializerIOSim;
import frc.robot.subsystems.shooter.serializer.SerializerIOTalonFX;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAccelerator;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAcceleratorIO;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAcceleratorIOSim;
import frc.robot.subsystems.shooter.shooter_omniwheels.ShooterOmniwheels;
import frc.robot.subsystems.shooter.shooter_omniwheels.ShooterOmniwheelsIO;
import frc.robot.subsystems.shooter.shooter_omniwheels.ShooterOmniwheelsIOSim;
import frc.robot.subsystems.shooter.shooter_rollers.ShooterRollers;
import frc.robot.subsystems.shooter.shooter_rollers.ShooterRollersIO;
import frc.robot.subsystems.shooter.shooter_rollers.ShooterRollersIOSim;
import frc.robot.subsystems.shooter.shooter_rollers.ShooterRollersIOTalonFX;
import frc.robot.subsystems.shooter.shooter_rollers.ShooterRollers.ShooterRollersTarget;
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
import frc.robot.subsystems.vision.VisionIOPhotonvisionSim;
import frc.robot.subsystems.vision.VisionIOPhotonvision;
import frc.robot.utility.ElasticSetpoints;
import frc.robot.utility.SimBattery;

import java.util.Set;
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

  private boolean defaultZeroing = false;

  // private SendableChooser<Command> autoChooser;
  private LoggedDashboardChooser<Command> autoChooser;

  private final CommandXboxController driverA = new CommandXboxController(0);
  private final CommandXboxController driverB = new CommandXboxController(1);

  private Serializer serializer;
  private Drive swerve;
  private Vision vision;
  private RGB rgb;
  private CANWatchdog canWatchdog;
  private IntakeController intakeController;
  private IntakeRack intakeRack;
  private IntakeRollers intakeRollers;
  private ShooterRollers shooterRollers;
  private ShooterAccelerator shooterAccelerator;
  private ShooterOmniwheels shooterOmniwheels;
  private ShooterController shooterController;

  private SwerveDriveSimulation driveSimulation = null;

  public RobotContainer() {

    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case COMP -> {
          intakeRack = new IntakeRack(new IntakeRackIOTalonFX());
          intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
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
                  new VisionIOPhotonvision("CamC", 0),
                  new VisionIOPhotonvision("CamA", 1),
                  new VisionIOPhotonvision("CamB", 2));
          serializer = new Serializer(new SerializerIOTalonFX());
          shooterRollers = new ShooterRollers(new ShooterRollersIOTalonFX());
          
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
          intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
          intakeRack = new IntakeRack(new IntakeRackIOSim());
          SimulatedArena.getInstance().resetFieldForAuto();
          serializer = new Serializer(new SerializerIOSim());
          shooterRollers = new ShooterRollers(new ShooterRollersIOSim());
          shooterAccelerator = new ShooterAccelerator(new ShooterAcceleratorIOSim());
          shooterOmniwheels = new ShooterOmniwheels(new ShooterOmniwheelsIOSim());
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
    
    if (intakeRack == null) {
      intakeRack = new IntakeRack(new IntakeRackIO() {});
    }

    if (intakeRollers == null) {
      intakeRollers = new IntakeRollers(new IntakeRollersIO() {});
    }

    if (shooterRollers == null) {
      shooterRollers = new ShooterRollers(new ShooterRollersIO() {});
    }

    if (serializer == null) {
      serializer = new Serializer(new SerializerIO() {});
    }

    if (shooterAccelerator == null) {
      shooterAccelerator = new ShooterAccelerator(new ShooterAcceleratorIO() {});
    }

    if (shooterOmniwheels == null) {
      shooterOmniwheels = new ShooterOmniwheels(new ShooterOmniwheelsIO() {});
    }
 
    intakeController = new IntakeController(intakeRack, intakeRollers);
    shooterController = new ShooterController(serializer, shooterAccelerator, shooterOmniwheels, shooterRollers);

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
    // Register Command Names in this method

    NamedCommands.registerCommand("Smart zero", new InstantCommand(() -> swerve.smartZeroGyro()));

    Supplier<Pose2d> shootingPoseSupplier =
        () ->
            ((autoChooser == null ? false : autoChooser.get().getName().contains("Right")))
                ? new Pose2d(3.891, 0.823, new Rotation2d(77.005 * Math.PI / 180))
                : new Pose2d(
                    3.891,
                    FlippingUtil.fieldSizeY - 0.823,
                    new Rotation2d((-77.005) * Math.PI / 180));
          
    NamedCommands.registerCommand(
        "Check If Off", new WaitUnitlRobotStuckCommand(swerve, shootingPoseSupplier)); 
    Supplier<Pose2d> flippedShootingPoseSupplier =
        () -> {
          return RobotState.isAllianceRed()
              ? FlippingUtil.flipFieldPose(shootingPoseSupplier.get())
              : shootingPoseSupplier.get();
        };
    NamedCommands.registerCommand(
        "Translate To Shoot",
        Commands.defer(
            () -> {
              if (!RobotState.getInstance().isAutoAdaptive()) {
                return new InstantCommand();
              }
              return ((new AlignToPoseCommand(swerve, flippedShootingPoseSupplier, true)
                          .raceWith(new WaitUnitlRobotStuckCommand(swerve, shootingPoseSupplier)))
                      .repeatedly())
                  .until(
                      () ->
                          flippedShootingPoseSupplier
                                  .get()
                                  .getTranslation()
                                  .getDistance(
                                      RobotState.getInstance().getEstimatedPose().getTranslation())
                              < .04)
                  .andThen(
                      new InstantCommand(() -> RobotState.getInstance().resetDynamicObstacles()))
                  .andThen(
                      new InstantCommand(() -> RobotState.getInstance().setIsAutoAdaptive(false)));
            },
            Set.of(swerve)));
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

    driverA.start().onTrue(swerve.zeroGyroCommand());

    // driverA.a().onTrue(new InstantCommand(() -> swerve.smartZeroGyro()));
    // driverA.x().onTrue(new InstantCommand(() -> {
    //   if (intakeController.getTargetState() != IntakeState.INTAKE) {
    //     intakeController.setTargetState(IntakeState.INTAKE);
    //   } else {
    //     intakeController.setTargetState(IntakeState.IDLE);
    //   }
    // }));

    // TODO: Test this eventually...
    // driverA.b().whileTrue(new AlignToShootCommand(swerve).repeatedly());

    // TODO: Test
    // driverA.y().whileTrue(new InstantCommand(() -> shooterRollers.setVelocityTarget(ShooterRollersTarget.SHOOT)));

    // TODO: Test this eventually...
    // driverA.b().whileTrue(new AxisAssistCommand(swerve));

    configureDriverAButtons();
    new Trigger(() -> vision.getMultiTags() && !defaultZeroing)
        .onTrue(new InstantCommand(() -> swerve.smartZeroGyro()));

  }

  private void configureDriverAButtons(){

    driverB.povUp().onTrue(new LowPowerModeCommand(swerve));
    
    driverA.leftBumper().whileTrue(new AlignToTrenchCommand(swerve));

    driverA.rightBumper().whileTrue(new AlignToPoseCommand(swerve, new Pose2d(12.9, 4, new Rotation2d(3.14))));

    driverA.x().onTrue(new InstantCommand(() -> swerve.smartZeroGyro()));
    
    driverA.b().onTrue(new IntakeCommand(intakeController, shooterController))
    .onFalse(new InstantCommand(() -> intakeController.setTargetState(IntakeState.IDLE)));

    driverA.y().onTrue(new InstantCommand(() -> intakeController.setTargetState(IntakeState.REVERSE)));

    driverA.a().whileTrue(new InstantCommand(() -> shooterController.setTargetState(ShooterState.SHOOT))
    .andThen(new AlignToShootCommand(swerve)))
    .onFalse(new InstantCommand(() -> shooterController.setTargetState(ShooterState.IDLE)));

    driverA.povDown().onTrue(new InstantCommand(() -> intakeController.setTargetState(IntakeState.ZEROING)));

    driverA.povLeft().whileTrue(new InstantCommand(() -> shooterController.setTargetState(ShooterState.SHOOT))
    .andThen(new AlignToPassCommand(swerve, new Translation2d(14.6, 0.7))))
    .onFalse(new InstantCommand(() -> shooterController.setTargetState(ShooterState.IDLE)));

    driverA.povRight().whileTrue(new InstantCommand(() -> shooterController.setTargetState(ShooterState.SHOOT))
    .andThen(new AlignToPassCommand(swerve, new Translation2d(14.6, 7.3))))
    .onFalse(new InstantCommand(() -> shooterController.setTargetState(ShooterState.IDLE)));

    driverA.start().onTrue(new InstantCommand(() -> defaultZeroing = !defaultZeroing));
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
    RobotState.getInstance().setIsAutoUnderTrench(true);
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
    Commands.sequence(
            new IntakeCommand(intakeController, shooterController),
            new WaitCommand(5.0),
            Commands.runOnce(() -> intakeController.setTargetState(IntakeState.IDLE)),
            Commands.runOnce(() -> shooterController.setTargetState(ShooterState.SHOOT)),
            new WaitCommand(5.0),
            Commands.runOnce(() -> shooterController.setTargetState(ShooterState.INTAKE)),
            new WaitCommand(5.0),
            Commands.runOnce(() -> shooterController.setTargetState(ShooterState.IDLE)),
            Commands.runOnce(() -> intakeController.setTargetState(IntakeState.IDLE)),
            new WaitCommand(5.0))
        .schedule();
    
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
    String serData = serializer.runTestLogger(testModeStartTime, currentTime, isNextTransition);
    String accData = shooterAccelerator.runTestLogger(testModeStartTime, currentTime, isNextTransition);
    String rackData = intakeRack.runTestLogger(testModeStartTime, currentTime, isNextTransition);
    String rolData = intakeRollers.runTestLogger(testModeStartTime, currentTime, isNextTransition);

    if (isNextTransition) {
      double relativeTime = currentTime - testModeStartTime;
      combinedTestData.add(
          String.format("%.1f", relativeTime) + "," + serData + "," + accData + "," + rackData + "," + rolData);
    }
    
  }

  public void testExit() {
    shooterController.setTargetState(ShooterState.IDLE);
    intakeController.setTargetState(IntakeState.IDLE);
    printAndClearStoredData();
  }


  /** Ran when periodic disabled */
  public void updateDashboardStatus() {
    // TODO: Define all of the dashboard outputs here
        var selectedAuto = autoChooser.get();
    SmartDashboard.putString("Current Auto", selectedAuto != null ? selectedAuto.getName() : "None");
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
    RobotSimState.getInstance().applyTerrainGravityForce();
    SimulatedArena.getInstance().simulationPeriodic();
    RobotSimState.getInstance().updateTerrainState();
    RobotSimState.getInstance().getFuelSim().updateSim();
    SimBattery.getInstance().update();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/ObstaclePositions",
        RobotSimState.getInstance().getObstaclePositions().toArray(new Pose2d[0]));

    RobotSimState.getInstance().logScoring();



    // Update the shooting logic with the correct rollers
    RobotSimState.getInstance()
        .setShooterRunning(
            shooterRollers.getCurrentVelocity().in(MetersPerSecond) > 1.0
                && shooterAccelerator.getCurrentVelocity().in(RotationsPerSecond) > 1.0
                && shooterOmniwheels.getCurrentVelocity().in(RotationsPerSecond) > 1.0,
            20.0,
                new Transform3d(new Translation3d(0.2, 0, 0.2), new Rotation3d(0, 0, 0)),
            shooterRollers.getCurrentVelocity(),
            Units.Inches.of(26));

    // Handle automatic shooter firing
    RobotSimState.getInstance().periodicShooter();
  }

    public void printAndClearStoredData() {
        System.out.println("=== Test Mode Combined Data ===");
        System.out.println("Timestamp(s),Ser AvgRad/s,Ser AvgCurrent(A),Acc AvgRad/s,Acc AvgCurrent(A),Rack AvgRot/s,Rack AvgCurrent(A),Rollers AvgRad/s,Rollers AvgCurrent(A)");
        for (String row : combinedTestData) {
            System.out.println(row);
        }
        combinedTestData.clear();
        serializer.resetTestState();
        shooterAccelerator.resetTestState();
        intakeRack.resetTestState();
        intakeRollers.resetTestState();
    }
}
