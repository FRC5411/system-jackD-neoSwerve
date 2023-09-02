package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;

public class SwerveSubsystem extends SubsystemBase {

    private SwerveModule[] swerveMods;
    private SwerveModulePosition[] swerveModPoses;
    private SwerveDriveOdometry swerveOdometry;

    private Pigeon2 gyro;

    private Field2d field;
    
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

        resetModules();
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

    public void resetPose(Pose2d pose) {
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
            states[mod.moduleID] = mod.getActualState();
        }

        return states;
    }

    public SwerveModulePosition[] getPositions() {
        for (SwerveModule mod : swerveMods) {
            swerveModPoses[mod.moduleID] = new SwerveModulePosition(
                mod.getDriveEncoderPosition(), 
                mod.getAngle());
        }

        return swerveModPoses;
    }

    public void resetModules() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Swerve.swerveKinematics.toChassisSpeeds(getStates());
    }

    public Command followPathGroup(HashMap<String, Command> eventMap, boolean useAlliance, String pathName) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            pathName, 
            PathPlanner.getConstraintsFromPath(pathName),
            PathPlanner.getConstraintsFromPath(pathName)
            );

        PathPlannerTrajectory mapTrajectory = PathPlanner.loadPath(
            pathName, 
            PathPlanner.getConstraintsFromPath(pathName));
        
        field.getObject(pathName).setTrajectory(mapTrajectory);

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
            this::getPose, 
            this::resetPose, 
            Swerve.swerveKinematics, 
            new PIDConstants(0, 0, 0), 
            new PIDConstants(0, 0, 0), 
            this::setModuleStates, 
            eventMap, 
            useAlliance, 
            this);

        return builder.fullAuto(pathGroup);
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getPositions());
        field.setRobotPose(getPose());

        SmartDashboard.putData(field);
        
        for (SwerveModule mod : swerveMods) {
            mod.Telemtry();
        }

        SmartDashboard.putNumber("Swerve/Chassis/Gyro/Yaw", getYaw().getDegrees());
        SmartDashboard.putNumber("Swerve/Chassis/Gyro/Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Swerve/Chassis/Gyro/Roll", gyro.getRoll());

        SmartDashboard.putNumber("Swerve/Chassis/X", getPose().getX());
        SmartDashboard.putNumber("Swerve/Chassis/Y", getPose().getY());
        SmartDashboard.putNumber("Swerve/Chassis/Rotation", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("Swerve/Chassis/Speed/X", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis/Speed/Y", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis/Speed/Rot", getChassisSpeeds().omegaRadiansPerSecond);
    }
}
