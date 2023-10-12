package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Swerve;

public class AutonManager {
    private final SwerveSubsystem m_robotSwerve;

    public AutonManager(SwerveSubsystem driveSubsystem) 
    {
        m_robotSwerve = driveSubsystem;
    }

    public Command followPathCommand(String path) 
    {
        List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup(path, 
            PathPlanner.getConstraintsFromPath(path));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            m_robotSwerve::getPose, 
            m_robotSwerve::resetOdometry, 
            Swerve.swerveKinematics, 
            Swerve.tranConstants, 
            Swerve.rotConstants, 
            m_robotSwerve::setModuleStates, 
            new HashMap<String, Command>(), 
            false, 
            m_robotSwerve);

        return autoBuilder.fullAuto(trajectory);
    }

    public Command alignToTargetCommand(Pose2d target)
    {
        final PathConstraints kPathConstraints = new PathConstraints(2.0, 1.0);

        final PIDController kTranslationController = new PIDController(
            SmartDashboard.getNumber("/alignToTagCommand/tP", 0.0), 
            SmartDashboard.getNumber("/alignToTagCommand/tI", 0.0), 
            SmartDashboard.getNumber("/alignToTagCommand/tD", 0.0)
        );
        final PIDController kRotationController = new PIDController(
            SmartDashboard.getNumber("/alignToTagCommand/rP", 0.0), 
            SmartDashboard.getNumber("/alignToTagCommand/rI", 0.0), 
            SmartDashboard.getNumber("/alignToTagCommand/rD", 0.0)
        );

        System.out.println("/auton/alignToTargetCommand/ Set translation values: " + kTranslationController.getP() + kTranslationController.getI() + kTranslationController.getD());

        PathPlannerTrajectory alignTrajectory = PathPlanner.generatePath(
            kPathConstraints,
            // Robot Pose
            new PathPoint(new Translation2d(
                m_robotSwerve.getPose().getX(),
                m_robotSwerve.getPose().getY()),
                m_robotSwerve.getYaw()
            ),
            // Target in view
            new PathPoint(new Translation2d(
                m_robotSwerve.getPose().getX(),
                target.getY()),
                target.getRotation()
            )
        );

        Command alignToTargetCommand = new PPSwerveControllerCommand(
            alignTrajectory, 
            () -> m_robotSwerve.getPose(), 
            kTranslationController, 
            kTranslationController, 
            kRotationController, 
            m_robotSwerve::driveFromChassisSpeeds, 
            m_robotSwerve
        );

        SwerveModuleState[] initialStates = new SwerveModuleState[4];

        for (int i = 0; i < initialStates.length; i++)
        {
            initialStates[i] = new SwerveModuleState(0.0, new Rotation2d());
        }

        Command resetWheels = new InstantCommand(
            () -> m_robotSwerve.setModuleStates(initialStates)).repeatedly().withTimeout(0.5);

        return new SequentialCommandGroup(resetWheels, alignToTargetCommand);
    }

    public Command goToTargetCommand(Pose2d target)
    {
        final PathConstraints kPathConstraints = new PathConstraints(2.0, 1.0);

        final PIDController kTranslationController = new PIDController(
            SmartDashboard.getNumber("/alignToTagCommand/tP", 0.0), 
            SmartDashboard.getNumber("/alignToTagCommand/tI", 0.0), 
            SmartDashboard.getNumber("/alignToTagCommand/tD", 0.0)
        );
        final PIDController kRotationController = new PIDController(
            SmartDashboard.getNumber("/alignToTagCommand/rP", 0.0), 
            SmartDashboard.getNumber("/alignToTagCommand/rI", 0.0), 
            SmartDashboard.getNumber("/alignToTagCommand/rD", 0.0)
        );

        System.out.println("/auton/goToTargetCommand/ Set translation values: " + kTranslationController.getP() + kTranslationController.getI() + kTranslationController.getD());

        PathPlannerTrajectory alignTrajectory = PathPlanner.generatePath(
            kPathConstraints,
            // Robot Pose
            new PathPoint(new Translation2d(
                m_robotSwerve.getPose().getX(),
                m_robotSwerve.getPose().getY()),
                m_robotSwerve.getYaw()
            ),
            // Target in view
            new PathPoint(new Translation2d(
                m_robotSwerve.getPose().getX(),
                target.getY()),
                target.getRotation()
            )
        );

        PathPlannerTrajectory driveToTargetTrajectory = PathPlanner.generatePath(
            kPathConstraints,
            new PathPoint(new Translation2d(
                // Robot pose in relation to the target on the X
                m_robotSwerve.getPose().getX(),
                target.getY()),
                target.getRotation()
            ),
            new PathPoint(new Translation2d(
                // Target pose
                target.getX(),
                target.getY()),
                target.getRotation()
            )
        );

        Command alignToTargetCommand = new PPSwerveControllerCommand(
            alignTrajectory, 
            () -> m_robotSwerve.getPose(), 
            kTranslationController, 
            kTranslationController, 
            kRotationController, 
            m_robotSwerve::driveFromChassisSpeeds, 
            m_robotSwerve
        );

        Command driveToTargetCommand = new PPSwerveControllerCommand(
            driveToTargetTrajectory, 
            () -> m_robotSwerve.getPose(), 
            kTranslationController, 
            kTranslationController, 
            kRotationController, 
            m_robotSwerve::driveFromChassisSpeeds, 
            m_robotSwerve
        );

        SwerveModuleState[] initialStates = new SwerveModuleState[4];

        for (int i = 0; i < initialStates.length; i++)
        {
            initialStates[i] = new SwerveModuleState(0.0, new Rotation2d());
        }

        Command resetWheels = new InstantCommand(
            () -> m_robotSwerve.setModuleStates(initialStates)).repeatedly().withTimeout(0.5);

        return new SequentialCommandGroup(resetWheels, alignToTargetCommand, driveToTargetCommand);
    }
}
