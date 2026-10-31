package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.utility.FuelSim;
import frc.robot.utility.TrenchTerrainSim;

import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotSimState {

  public static final int START_FUEL_CAPACITY = 8;

  private int fuelCount = START_FUEL_CAPACITY;
  private boolean intakeActive = false;

  List<SwerveDriveSimulation> obstacles = new ArrayList<SwerveDriveSimulation>();

  private RobotSimState() {
    // init the arena (drive sim only, no game piece placement)
    Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
    // arena.setEfficiencyMode(true);
    arena.clearGamePieces();
    arena.setShouldRunClock(true);

    // Add the drive simulation
    driveSimulation =
        new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, RobotState.getInstance().getEstimatedPose());
    arena.addDriveTrainSimulation(driveSimulation);

    SimulatedArena.overrideInstance(arena);

    // init fuel sim
    fuelSim = new FuelSim("Field Simulation");
    fuelSim.registerRobot(
        DriveConstants.mapleSimConfig.bumperWidthY, // from left to right in meters
        DriveConstants.mapleSimConfig.bumperLengthX, // from front to back in meters
        Units.Inches.of(7), // from floor to top of bumpers in meters
        driveSimulation::getSimulatedDriveTrainPose, // Supplier<Pose2d> of robot pose
        driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative);

    // Register intake on the back of the robot
    double halfLength = DriveConstants.mapleSimConfig.bumperLengthX.in(Meters) / 2.0;
    double halfWidth = DriveConstants.mapleSimConfig.bumperWidthY.in(Meters) / 2.0;
    double intakeReach = 0.1; // meters beyond bumper
    fuelSim.registerIntake(
        halfLength,
        halfLength + intakeReach,
        -halfWidth,
        halfWidth,
        () -> intakeActive && fuelCount < 60,
        () -> fuelCount++);

    fuelSim.spawnStartingFuel();
    fuelSim.setLoggingFrequency(20);
    fuelSim.start();

    addObstacleToSim(new Pose2d(new Translation2d(9.8, 5.385), new Rotation2d()));
  }

  // Get sim state from RobotContainer

  public List<Pose2d> getObstaclePositions() {
    return obstacles.stream().map((obstacle) -> obstacle.getSimulatedDriveTrainPose()).toList();
  }

  public void addObstacleToSim(Pose2d pose) {
    SwerveDriveSimulation obstacle = new SwerveDriveSimulation(DriveConstants.obstacleConfig, pose);
    obstacles.add(obstacle);
    SimulatedArena.getInstance().addDriveTrainSimulation(obstacle);
  }

  // Singleton instance
  private static RobotSimState instance = null;

  public static RobotSimState getInstance() {
    if (Constants.getRobotType() != RobotType.SIM) {
      // idiot proofing
      System.out.println(
          "WARNING: YOU ARE TRYING TO ACCESS ROBOT SIM STATE FROM AN ACTUAL ROBOT -- THIS IS A CODE"
              + " ERROR");
    }
    if (instance == null) instance = new RobotSimState();
    return instance;
  }

  // Drive simulation
  private SwerveDriveSimulation driveSimulation;

  // Terrain contact state — updated each loop after simulationPeriodic()
  private TrenchTerrainSim.TerrainContactState terrainState =
      new TrenchTerrainSim.TerrainContactState(
          DriveConstants.DRIVE_CONFIG.wheelRadius(), 0.0, 0.0, 0.0, new double[4]);


  public SwerveDriveSimulation getDriveSimulation() {
    return driveSimulation;
  }

  private FuelSim fuelSim;

  public FuelSim getFuelSim() {
    return fuelSim;
  }

  // Get attributes of physical drivebase
  public Pose2d getRobotPose2d() {
    return driveSimulation.getSimulatedDriveTrainPose();
  }

  public Pose3d getRobotPose3d() {
    Pose2d robotPose2d = driveSimulation.getSimulatedDriveTrainPose();
    return new Pose3d(
        new Translation3d(robotPose2d.getX(), robotPose2d.getY(), 0.0),
        new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));
  }

  public ChassisSpeeds getChassisSpeedsFieldRelative() {
    return driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
  }

  /**
   * Updates the terrain contact state from the robot's current simulated pose. Call this once per
   * loop <em>after</em> {@code SimulatedArena.getInstance().simulationPeriodic()}.
   */
  public void updateTerrainState() {
    terrainState =
        TrenchTerrainSim.compute(
            driveSimulation.getSimulatedDriveTrainPose(),
            DriveConstants.MODULE_TRANSLATIONS,
            DriveConstants.DRIVE_CONFIG.wheelRadius(),
            DriveConstants.DRIVE_CONFIG.trackWidth(), // front-to-rear (wheelbase)
            DriveConstants.DRIVE_CONFIG.trackLength()); // left-to-right (track width)
  }

  /**
   * Injects a gravity resistance force into the Dyn4j physics engine to simulate the robot climbing
   * or descending the trench bump. Call this once per loop <em>before</em> {@code
   * SimulatedArena.getInstance().simulationPeriodic()}.
   *
   * <p>The force is scaled by the number of physics sub-ticks because Dyn4j clears the {@code
   * Body.force} accumulator after each sub-tick integration step. A single {@code applyForce()}
   * call placed before {@code simulationPeriodic()} would otherwise only affect the first sub-tick.
   */
  public void applyTerrainGravityForce() {
    double gradientX = terrainState.avgGradientX();
    if (gradientX == 0.0) return; // flat ground — no bump force needed

    double mass = driveSimulation.getMass().getMass(); // kg, from Dyn4j Body
    double subTickDtSec =
        SimulatedArena.getSimulationDt().in(edu.wpi.first.units.Units.Seconds); // typically 0.004 s
    int nSubTicks = (int) Math.round(Constants.PERIODIC_LOOP_SEC / subTickDtSec); // typically 5

    // F = −m · g · dZ/dX (opposing forward motion on an uphill ramp)
    // Multiply by nSubTicks so the impulse integrates to the correct total over the full 20 ms loop
    double fx = -mass * 9.81 * gradientX * nSubTicks;
    driveSimulation.applyForce(new Vector2(fx, 0.0));
  }

  // Shooting utilities
  public void shootFuel(
      Angle launchAngle,
      Transform3d shooterTransform3d,
      LinearVelocity launchVelocity,
      Distance shooterWidth) {
    if (fuelCount <= 0) return; // no fuel to shoot
    fuelCount--;

    // Build a transform that includes the shooter's position and combines the hood pitch with the
    // shooter's yaw
    Transform3d shooterOffset =
        new Transform3d(
            new Translation3d(0, (Math.random() * 2 - 1) * (shooterWidth.in(Units.Meters) * .5), 0),
            new Rotation3d());

    Transform3d launchTransform =
        new Transform3d(
                shooterTransform3d.getTranslation(),
                new Rotation3d(0, 0, shooterTransform3d.getRotation().getZ()))
            .plus(shooterOffset);

    fuelSim.launchFuel(launchVelocity, launchAngle, launchTransform);
  }

  // Intake simulation (backed by FuelSim)

  public void setIntakeState(boolean extended) {
    intakeActive = extended;
  }

  public int getFuelCount() {
    return fuelCount;
  }
  
  public void logScoring() {
    int blue = FuelSim.Hub.BLUE_HUB.getScore();
    int red = FuelSim.Hub.RED_HUB.getScore();
    int scored = RobotState.getInstance().isAllianceRed() ? red : blue;

    Logger.recordOutput("Field Simulation/Blue Score", blue);
    Logger.recordOutput("Field Simulation/Red Score", red);
    Logger.recordOutput("Field Simulation/Score", scored);
    Logger.recordOutput("Field Simulation/Held Fuel", fuelCount);
  }

  public Pose3d[] getIntakeGamePieces() {
    Pose3d[] gamePiecePoses = new Pose3d[fuelCount];
    double spacing = Units.Inches.of(5.91).in(Units.Meters);
    for (int i = 0; i < fuelCount; i++) {
      gamePiecePoses[i] =
          new Pose3d(
              new Translation3d(
                  driveSimulation.getSimulatedDriveTrainPose().getX(),
                  driveSimulation.getSimulatedDriveTrainPose().getY(),
                  i * spacing),
              new Rotation3d());
    }
    return gamePiecePoses;
  }

  // Automatic shooter state tracking
  private boolean isShooterRunning = false;
  private double lastShootTime = 0.0;
  private double shootIntervalSeconds = 0.0;

  /**
   * Tells the RobotSimState that the shooter is currently running and should shoot fuel
   * automatically.
   *
   * @param shotsPerSecond The rate at which to shoot fuel (e.g., 2.0 for 2 shots per second)
   * @param shooterAngle The angle at which to shoot
   * @param shooterTransform3d The 3D transform of the shooter relative to the robot
   * @param launchVelocity The velocity at which to launch the fuel
   */
  public void setShooterRunning(
      boolean running,
      double shotsPerSecond,
      Transform3d shooterTransform3d,
      LinearVelocity launchVelocity,
      Distance shooterWidth) {
    if (running && !isShooterRunning) {
      // Starting the shooter
      isShooterRunning = true;
      shootIntervalSeconds = 1.0 / shotsPerSecond;
      lastShootTime = Timer.getFPGATimestamp();
    } else if (!running) {
      // Stopping the shooter
      isShooterRunning = false;
    }

    // Store the shooting parameters for use in periodic
    
    this.currentShooterTransform = shooterTransform3d;
    this.currentLaunchVelocity = launchVelocity;
    this.currentShooterWidth = shooterWidth;
  }

  // Store current shooting parameters
  final private Angle PERMANENT_UNCHANGEABLE_SHOOTER_ANGLE = Units.Degrees.of(75);
  private Transform3d currentShooterTransform = new Transform3d();
  private LinearVelocity currentLaunchVelocity = MetersPerSecond.of(0);
  private Distance currentShooterWidth = Meters.of(0);

  /**
   * Should be called periodically (e.g., in Robot.java's simulationPeriodic). Handles automatic
   * shooting when the shooter is running.
   */
  public void periodicShooter() {
    if (!isShooterRunning) {
      return;
    }

    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - lastShootTime >= shootIntervalSeconds) {
      Logger.recordOutput("THISISATESTTESTTEST", 1);
      // Time to shoot another ball
      shootFuel(
          PERMANENT_UNCHANGEABLE_SHOOTER_ANGLE, currentShooterTransform, currentLaunchVelocity, currentShooterWidth);
      lastShootTime = currentTime;
      Logger.recordOutput("Robot Sim State/Auto Shooter Active", true);
    }
  }

  @AutoLogOutput(key = "Robot Sim State/Shooter Running")
  public boolean isShooterRunning() {
    return isShooterRunning;
  }
}
