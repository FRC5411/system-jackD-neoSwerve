
package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;

public class SwerveSubsystem extends SubsystemBase {

    private SwerveModule[] swerveMods;
    private SwerveModulePosition[] swerveModPoses;
    private SwerveDriveOdometry swerveOdometry;

    private Pigeon2 gyro;
    private Pose2d _robotPose = new Pose2d();
    private Field2d field;

    private double _translationKp = 0;
    private double _translationKi = 0;
    private double _translationKd = 0;
    private double _rotationKp = 0;
    private double _rotationKi = 0;
    private double _rotationKd = 0;
    
    private double ROBOT_WIDTH = 0.6858;

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(ROBOT_WIDTH/2, ROBOT_WIDTH/2),
    new Translation2d(ROBOT_WIDTH/2, -ROBOT_WIDTH/2),
    new Translation2d(-ROBOT_WIDTH/2, ROBOT_WIDTH/2),
    new Translation2d(-ROBOT_WIDTH/2, -ROBOT_WIDTH/2)
    );

    private SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(m_kinematics, new Rotation2d(0), getPositions(), new Pose2d());
    

    public SwerveSubsystem() {
        swerveMods = new SwerveModule[] {
            new SwerveModule(0, 
                Swerve.Mod0.constants),
            new SwerveModule(1, 
                Swerve.Mod1.constants),
            new SwerveModule(2, 
                Swerve.Mod2.constants),
            new SwerveModule(3, 
                Swerve.Mod3.constants)
        };

        swerveModPoses = new SwerveModulePosition[] {
            new SwerveModulePosition(swerveMods[0].getDriveEncoderPosition(), 
                swerveMods[0].getAngle()),
            new SwerveModulePosition(swerveMods[1].getDriveEncoderPosition(), 
                swerveMods[1].getAngle()),
            new SwerveModulePosition(swerveMods[2].getDriveEncoderPosition(), 
                swerveMods[2].getAngle()),
            new SwerveModulePosition(swerveMods[3].getDriveEncoderPosition(), 
                swerveMods[3].getAngle())
        };

        gyro = new Pigeon2(Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(Swerve.swerveKinematics, getYaw(), swerveModPoses);

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public void swerveDrive(Translation2d translation, double rotation, 
        boolean fieldRelative, boolean isOpenLoop) {
        
        SwerveModuleState[] swerveModuleStates = Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.maxSpeed);
        
        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleID], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.maxSpeed);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleID], false);
        }
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), swerveModPoses, pose);
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Rotation2d getYaw() {
        return (Swerve.invertGyro) 
            ? Rotation2d.fromDegrees(360 - gyro.getYaw()) 
            : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (SwerveModule mod : swerveMods) {
            states[mod.moduleID] = mod.getState();
        }

        return states;
    }

    public SwerveModulePosition[] getPositions() {
        System.out.print("All SwerveMods");
        System.out.println(swerveMods);
        for (SwerveModule mod : swerveMods) {
            swerveModPoses[mod.moduleID] = new SwerveModulePosition(
                mod.getDriveEncoderPosition(), 
                mod.getAngle());
        }

        return swerveModPoses;
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getPositions());
        field.setRobotPose(getPose());

        SmartDashboard.putData(field);

        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Module " + mod.moduleID + " Cancoder ", 
                mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Module " + mod.moduleID + " Integrated ", 
                mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Module " + mod.moduleID + " Velocity ", 
                mod.getState().speedMetersPerSecond);
        }
    }
    /*public Command PPmoveToPositionCommand () {
        Pose2d actualPose = _robotPose;
        return PPpathToCommand( closest );
    }*/
    public Command PPpathToCommand (Pose2d target) {
        PathPlannerTrajectory _alignToTarget = PathPlanner.generatePath(
          new PathConstraints(1, 0.5),
          new PathPoint(new Translation2d(
            m_odometry.getEstimatedPosition().getX(), 
            m_odometry.getEstimatedPosition().getY()), 
            new Rotation2d(Math.toRadians(gyro.getYaw())),
            // You can get rid of this if you want
            2),
    
          new PathPoint(
            new Translation2d(
              m_odometry.getEstimatedPosition().getX(), 
              target.getY()), 
              target.getRotation()
            )
    
            
        );
    
        PathPlannerTrajectory _toTarget = PathPlanner.generatePath(
          new PathConstraints(1, 0.5),
          new PathPoint(
            new Translation2d(
              m_odometry.getEstimatedPosition().getX(), 
              target.getY()), 
              target.getRotation(), 
              2),
    // In Order to fuse everything into one path just take this path point add it to the align to target path point
    // and then only use the align command not a sequential command, using 2 paths for testing purposes
          new PathPoint(
            new Translation2d(
              target.getX(), 
              target.getY()), 
              target.getRotation()
            )
        );
    
        PIDController tPID = new PIDController(_translationKp, _translationKi, _translationKd);
        tPID.setTolerance(0);
        tPID.setIntegratorRange(-0, 0);
    
        PIDController rPID = new PIDController(_rotationKp, _rotationKi, _rotationKd);
        rPID.setTolerance(0);
        rPID.setIntegratorRange(-0, 0);
    
    
    
        Command align = new PPSwerveControllerCommand(
          _alignToTarget,
          () -> m_odometry.getEstimatedPosition(), // Pose2d supplier
          this.m_kinematics, // SwerveDriveKinematics
          tPID, // PID constants to correct for translation error (used to create the X and Y PID controllers)
          tPID, // PID constants to correct for rotation error (used to create the rotation controller)
          rPID, // PID constants to correct for rotation error (used to create the rotation controller)
          this::setModuleStates, // Module states consumer used to output to the drive subsystem
          (Subsystem) this
        );
    
        Command toGoal = new PPSwerveControllerCommand(
          _toTarget,
          () -> m_odometry.getEstimatedPosition(), // Pose2d supplier
          this.m_kinematics, // SwerveDriveKinematics
          tPID, // PID constants to correct for translation error (used to create the X and Y PID controllers)
          tPID, // PID constants to correct for rotation error (used to create the rotation controller)
          rPID, // PID constants to correct for rotation error (used to create the rotation controller)
          this::setModuleStates, // Module states consumer used to output to the drive subsystem
          (Subsystem) this
        );
    
        return new SequentialCommandGroup(align, toGoal);
    }
    public Command getAutonomousCommand () {
        
    
        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the grou
          List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test",4,3);
            
          HashMap<String, Command> eventMap = new HashMap<>();
        
    
          // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
          SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            () -> m_odometry.getEstimatedPosition(), // Pose2d supplier
            pose -> m_odometry.resetPosition(new Rotation2d(Math.toRadians(gyro.getYaw())), getPositions(), pose), // Pose2d consumer, used to reset odometry at the beginning of auto
            this.m_kinematics, // SwerveDriveKinematics
            new PIDConstants(_translationKp, _translationKi, _translationKd), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(_rotationKp, _rotationKi, _rotationKd), // PID constants to correct for rotation error (used to create the rotation controller)
            this::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            (Subsystem) this // The drive subsystem. Used to properly set the requirements of path following commands
          );
    
          return autoBuilder.fullAuto(pathGroup);
          
        
    
        
    }
}
