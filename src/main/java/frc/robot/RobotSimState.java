package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.utility.BlankSimulatedArena;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class RobotSimState {

  List<SwerveDriveSimulation> obstacles = new ArrayList<SwerveDriveSimulation>();

  private RobotSimState() {
    // init the arena (drive sim only, no game piece placement)
    BlankSimulatedArena arena = new BlankSimulatedArena();
    // arena.setEfficiencyMode(true);
    arena.clearGamePieces();

    // Add the drive simulation
    driveSimulation =
        new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, RobotState.getInstance().getEstimatedPose());
    arena.addDriveTrainSimulation(driveSimulation);

    SimulatedArena.overrideInstance(arena);

    // addObstacleToSim(new Pose2d(new Translation2d(10.162, 5.697), new Rotation2d()));
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

  public SwerveDriveSimulation getDriveSimulation() {
    return driveSimulation;
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
}
