
package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.Interfaces.SwerveModuleInterface;
import frc.robot.Constants.Swerve;

public class SwerveSubsystem extends SubsystemBase {

    private SwerveModuleInterface[] swerveMods;
    private SwerveModulePosition[] swerveModPoses;
    private SwerveDriveOdometry swerveOdometry;

    private Pigeon2 gyro;

    private Field2d field;
    
    public SwerveSubsystem() {
        swerveMods = new SwerveModuleInterface[] {
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

        SmartDashboard.putBoolean("Field Oriented", false);
    }

    public void swerveDrive(Translation2d translation, double rotation, 
        boolean fieldRelative, boolean isOpenLoop) {
        
        SwerveModuleState[] swerveModuleStates = Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.maxSpeed);
        SmartDashboard.putBoolean("Field Oriented", fieldRelative);
        
        for (SwerveModuleInterface mod : swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.getModuleID()], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.maxSpeed);

        for (SwerveModuleInterface mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.getModuleID()], false);
        }
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), swerveModPoses, pose);
    }

    public void resetModules() {
        for (SwerveModuleInterface mod :swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Rotation2d getYaw() {
        return (Swerve.invertGyro) 
            ? Rotation2d.fromDegrees((360 - gyro.getYaw()) % 360)
            : Rotation2d.fromDegrees(gyro.getYaw() % 360);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (SwerveModuleInterface mod : swerveMods) {
            states[mod.getModuleID()] = mod.getState();
        }

        return states;
    }

    public SwerveModulePosition[] getPositions() {
        for (SwerveModuleInterface mod : swerveMods) {
            swerveModPoses[mod.getModuleID()] = new SwerveModulePosition(
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

        for (SwerveModuleInterface mod : swerveMods) {
            SmartDashboard.putNumber("Module " + mod.getModuleID() + " Cancoder ", 
                mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Module " + mod.getModuleID() + " Cancoder Offset ", 
                mod.getCanCoderOffset().getDegrees());
            SmartDashboard.putNumber("Module " + mod.getModuleID() + " Integrated ", 
                mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Module " + mod.getModuleID() + " Velocity ", 
                mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("Yaw ", gyro.getYaw());
    }
}
