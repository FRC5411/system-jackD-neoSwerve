
package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.libs.config.SwerveModuleConstraints;
import frc.libs.math.OnboardModuleState;
import frc.robot.Robot;
import frc.robot.Constants.Swerve;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class SwerveModule {

    public int moduleID;

    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private CANSparkMax driveMotor;
    private CANSparkMax azimuthMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder azimuthEncoder;
    private CANCoder angleEncoder;

    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController azimuthController;

    private final SimpleMotorFeedforward feedForward;

    public SwerveModule(int moduleID, SwerveModuleConstraints moduleConstants) {
        this.moduleID = moduleID;
        angleOffset = moduleConstants.angleOffset;

        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configPosition(angleEncoder, -moduleConstants.angleOffset.getDegrees());

        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, 
            MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configureDriveMotor();

        azimuthMotor = new CANSparkMax(moduleConstants.azimutMotorID, 
            MotorType.kBrushless);
        azimuthEncoder = azimuthMotor.getEncoder();
        azimuthController = azimuthMotor.getPIDController();
        configureAzimuthMotor();
        
        feedForward = new SimpleMotorFeedforward(Swerve.driveKS, 
            Swerve.driveKV, 
            Swerve.driveKA);

        lastAngle = getState().angle;

        Timer.delay(1);
        resetToAbsolute();
    }

    private void configureDriveMotor() {
        driveMotor.restoreFactoryDefaults();

        driveMotor.setSmartCurrentLimit(Swerve.driveContinuousCurrentLimit);
        driveMotor.enableVoltageCompensation(Swerve.voltageComp);

        driveMotor.setInverted(Swerve.driveInvert);
        driveMotor.setIdleMode(Swerve.driveNeutralMode);

        driveEncoder.setPosition(0.0);
        driveEncoder.setPositionConversionFactor(Swerve.driveConversionPositionFactor);
        driveEncoder.setVelocityConversionFactor(Swerve.driveConversionVelocityFactor);

        driveController.setP(Swerve.driveKP);
        driveController.setI(Swerve.driveKI);
        driveController.setD(Swerve.driveKD);
        driveController.setFF(Swerve.driveKFF);

        driveController.setFeedbackDevice(driveEncoder);

        driveMotor.burnFlash();
    }

    private void configureAzimuthMotor() {
        azimuthMotor.restoreFactoryDefaults();

        azimuthMotor.setSmartCurrentLimit(Swerve.azimuthContinuousCurrentLimit);
        azimuthMotor.enableVoltageCompensation(Swerve.voltageComp);

        azimuthMotor.setInverted(Swerve.azimuthInvert);
        azimuthMotor.setIdleMode(Swerve.azimuthNeutralMode);

        azimuthEncoder.setPosition(0.0);
        azimuthEncoder.setPositionConversionFactor(Swerve.azimuthConversionFactor);

        azimuthController.setP(Swerve.angleKP);
        azimuthController.setI(Swerve.angleKI);
        azimuthController.setD(Swerve.angleKD);
        azimuthController.setFF(Swerve.angleKFF);

        azimuthController.setFeedbackDevice(azimuthEncoder);

        azimuthMotor.burnFlash();

        resetToAbsolute(); 
    }

    // private void configureAngleEncoder() {
    //     angleEncoder.configFactoryDefault();

    //     angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    // }

    private void configPosition (CANCoder encoder, double offset) {
        encoder.configFactoryDefault();
        encoder.configMagnetOffset(offset);
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.setPositionToAbsolute();
        System.out.println(encoder.getDeviceID());
      }

    public void resetToAbsolute() {
 //       double absolutePosition = getCanCoder().getDegrees(); //- angleOffset.getDegrees();

        REVLibError error = azimuthEncoder.setPosition(angleEncoder.getAbsolutePosition());
        SmartDashboard.putString(moduleID+"/RESET", error.toString());
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Swerve.maxSpeed;

            driveMotor.set(percentOutput);
        }
        else {
            driveController.setReference(
                desiredState.speedMetersPerSecond, 
                ControlType.kVelocity, 
                0, 
                feedForward.calculate(desiredState.speedMetersPerSecond
            ));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = 
            (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve.maxSpeed * 0.01)) // Prevent jittering
                ? lastAngle 
                : desiredState.angle;
        
        azimuthController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

        setSpeed(desiredState, isOpenLoop);
        setAngle(desiredState);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(azimuthEncoder.getPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    public void canCoderTelemtry() {

        SmartDashboard.putNumber(moduleID + "/CANCODER/ABSPOS", angleEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(moduleID + "/CANCODER/POS", angleEncoder.getPosition());
        SmartDashboard.putString(moduleID + "/CANCODER/Error", angleEncoder.getLastError().toString());

    }
}