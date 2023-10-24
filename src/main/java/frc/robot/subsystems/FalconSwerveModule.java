
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.libs.Interfaces.SwerveModuleInterface;
import frc.libs.config.SwerveModuleConstraints;
import frc.robot.Robot;
import frc.robot.Constants.Swerve;

public class FalconSwerveModule implements SwerveModuleInterface{
    public int moduleID;

    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private WPI_TalonFX driveMotor;
    private WPI_TalonFX azimuthMotor;

    private CANCoder angleEncoder;

    private final SimpleMotorFeedforward feedForward;

    public FalconSwerveModule(int moduleID, SwerveModuleConstraints moduleConstants) {
        this.moduleID = moduleID;
        angleOffset = moduleConstants.angleOffset;

        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configureAngleEncoder();

        driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID, "drivetrain");
        configureDriveMotor();

        azimuthMotor = new WPI_TalonFX(moduleConstants.azimutMotorID, "drivetrain");
        configureAzimuthMotor();
        
        feedForward = new SimpleMotorFeedforward(Swerve.driveKS, 
            Swerve.driveKV, 
            Swerve.driveKA);

        lastAngle = getState().angle;
    }

    private void configureDriveMotor() {

    }

    private void configureAzimuthMotor() {
        resetToAbsolute(); 
    }

    public Rotation2d getCanCoderOffset() {
        return Rotation2d.fromDegrees(zeroTo360Scope(getCanCoder().getDegrees()));
    }

    private void configureAngleEncoder() {
        angleEncoder.configFactoryDefault();

        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    public void resetToAbsolute() {
        double absolutePosition = zeroTo360Scope(getCanCoder().getDegrees());

        azimuthMotor.setSelectedSensorPosition(absolutePosition);
    }

    public double zeroTo360Scope(double degrees) {
        double x = degrees - angleOffset.getDegrees();
        if (x < 0) x = 360 + x;
        return x;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        setSpeed(desiredState, isOpenLoop);
        setAngle(desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Swerve.maxSpeed;

            driveMotor.set(percentOutput);
        }
        else {
            driveMotor.set(ControlMode.Velocity,  desiredState.speedMetersPerSecond, 
            DemandType.ArbitraryFeedForward, feedForward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = 
            (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve.maxSpeed * 0.01)) // Prevent jittering
                ? lastAngle 
                : desiredState.angle;

        azimuthMotor.set(ControlMode.Position, zeroTo360Scope(angle.getDegrees()));
        lastAngle = angle;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(azimuthMotor.getSelectedSensorPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition() % 360);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), getAngle());
    }

    public double getDriveEncoderPosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public int getModuleID() {
        return moduleID;
    }
}
