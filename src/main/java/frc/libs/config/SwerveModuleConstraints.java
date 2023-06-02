
package frc.libs.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstraints {
    
    public final int driveMotorID;
    public final int azimutMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;
  
    /**
     * Swerve Module Constants to be used when creating swerve modules.
     *
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstraints(int driveMotorID, int azimutMotorID, int canCoderID, Rotation2d angleOffset) {
      this.driveMotorID = driveMotorID;
      this.azimutMotorID = azimutMotorID;
      this.cancoderID = canCoderID;
      this.angleOffset = angleOffset;
    }
}
