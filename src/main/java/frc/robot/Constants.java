
package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.libs.config.SwerveModuleConstraints;

public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.2;

    public static final int pigeonID = 1;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.75);
    public static final double wheelBase = Units.inchesToMeters(21.75);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = ((150/7.0) / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int azimuthContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.035;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0001; //0.0025;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.15;
    public static final double driveKV = 0.0;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double azimuthConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 2 * Math.PI;

    /* Neutral Modes */
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;
    public static final IdleMode azimuthNeutralMode = IdleMode.kCoast;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean azimuthInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    public static final PIDConstants tranConstants = new PIDConstants(
      11, 
      0, 
      0);

    public static final PIDConstants rotConstants = new PIDConstants(
      0.0, 
      0, 
      0);

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 11;
      public static final int azimuthMotorID = 21;
      public static final int canCoderID = 31;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(40.166016);
      public static final SwerveModuleConstraints constants =
          new SwerveModuleConstraints(driveMotorID, azimuthMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 12;
      public static final int azimuthMotorID = 22;
      public static final int canCoderID = 32;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(114.257812);
      public static final SwerveModuleConstraints constants =
          new SwerveModuleConstraints(driveMotorID, azimuthMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 13;
      public static final int azimuthMotorID = 23;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(302.695312);
      public static final SwerveModuleConstraints constants =
          new SwerveModuleConstraints(driveMotorID, azimuthMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 14;
      public static final int azimuthMotorID = 24;
      public static final int canCoderID = 34;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(358.154297);
      public static final SwerveModuleConstraints constants =
          new SwerveModuleConstraints(driveMotorID, azimuthMotorID, canCoderID, angleOffset);
    }
  }
}
