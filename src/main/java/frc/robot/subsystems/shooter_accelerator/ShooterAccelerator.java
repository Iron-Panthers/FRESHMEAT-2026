package frc.robot.subsystems.shooter_accelerator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.utility.LoggableMechanism3d;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.utility.LoggableMechanism3d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterAccelerator extends GenericSuperstructure<ShooterAccelerator.ShooterAcceleratorTarget>//Doesn't work cause of superstructure
    implements LoggableMechanism3d {
        public enum ShooterAcceleratorTarget implements GenericSuperstructure.PositionTarget {
            GONOW(0,ShooterAcceleratorConstants.CURRENT_LIMIT_AMPS),
            NOGO(50, ShooterAcceleratorConstants.CURRENT_LIMIT_AMPS);
        
        private double position;
        private double supplyCurrenLimit;
        private Pose3d parentPosition;
        private Pose3d setParentPosition;
        private double currentEpsilon;
        private double positionNowIAmAngwy;
        private double uwuSupplyCurrentLimit;
        private void ShooterAccelerator(String GenericSuperstructure){

        }
        private ShooterAcceleratorTarget(double uwuSupplyCurrentLimit, double position, double supplyCurrentLimit, double currentEpsilon, double positionNowIAmAngwy) {
            this.position = position;
            this.supplyCurrenLimit = supplyCurrentLimit;
            this.currentEpsilon=currentEpsilon;
            this.positionNowIAmAngwy=positionNowIAmAngwy;
            this.uwuSupplyCurrentLimit=uwuSupplyCurrentLimit;
        }
        ShooterAcceleratorTarget(int i, String currentLimitAmps) {
                    //TODO Auto-generated constructor stub
                }
                public double getVelocity(double velocity){
            return velocity;
        }
        public double getSupplyCurrentLimit(double supplyCurrentLimit){
            return supplyCurrenLimit;
        }
         public Pose3d getParentPosition( Pose3d parentPosition){
            return parentPosition;
        }
        @Override
        public double getEpsilon(){
            return this.currentEpsilon;
        }
        @Override
        public double getPosition() {
            return this.positionNowIAmAngwy;
        }
        @Override
        public double getSupplyCurrentLimit() {
            return this.uwuSupplyCurrentLimit;
        }
    }
        
        //public ShooterAccelerator(String shooteracceleratorconfig) {
            
        //}

        @Override
        public Pose3d getParentPosition() {
            return new Pose3d();
            }

        @Override
        public void setParent(LoggableMechanism3d parent) {}        

        @Override
        public Pose3d getDisplayPose3d() {
            return new Pose3d();
            }
        public ShooterAccelerator(ShooterAcceleratorIO io) {
        super("Shooter/Shooter Accelerator", io);
    }
        
       // public AngularVelocity getCurrentVelocity() {
    //return Units.RadiansPerSecond.of(inputs.velocityRadsPerSec);
  //}
}