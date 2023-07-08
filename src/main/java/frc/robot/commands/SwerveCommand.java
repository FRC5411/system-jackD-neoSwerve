
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends CommandBase {

    private SwerveSubsystem robotSwerve;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    private BooleanSupplier robotCentricSup;

    private SlewRateLimiter translationLimiter;
    private SlewRateLimiter strafeLimiter;
    private SlewRateLimiter rotationLimiter;
    
    public SwerveCommand(SwerveSubsystem robotSwerve, DoubleSupplier translationSup, 
        DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        
        this.robotSwerve = robotSwerve;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        this.robotCentricSup = robotCentricSup;

        translationLimiter = new SlewRateLimiter(3.0);
        strafeLimiter = new SlewRateLimiter(3.0);
        rotationLimiter = new SlewRateLimiter(8.0);

        addRequirements(robotSwerve);
    }

    @Override
    public void execute() {
        double translationVal = translationLimiter.calculate(
            MathUtil.applyDeadband(translationSup.getAsDouble(), Swerve.stickDeadband)
        );
        double strafeVal = strafeLimiter.calculate(
            MathUtil.applyDeadband(strafeSup.getAsDouble(), Swerve.stickDeadband)
        );
        double rotationVal = rotationLimiter.calculate(
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Swerve.stickDeadband)
        );

        robotSwerve.swerveDrive(
            new Translation2d(translationVal, strafeVal).times(Swerve.maxSpeed), 
            rotationVal * Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true);
    } 
}
