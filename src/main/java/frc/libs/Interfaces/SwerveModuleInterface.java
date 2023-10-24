package frc.libs.Interfaces;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleInterface {
    public int getModuleID();

    private void configureDriveMotor() {};

    private void configureAzimuthMotor() {};

    public Rotation2d getCanCoderOffset();

    private void configureAngleEncoder() {};

    public void resetToAbsolute();

    public double zeroTo360Scope(double degrees);

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {};

    private void setAngle(SwerveModuleState desiredState) {};

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);

    public Rotation2d getAngle();

    public Rotation2d getCanCoder();

    public SwerveModuleState getState();

    public double getDriveEncoderPosition();
}
