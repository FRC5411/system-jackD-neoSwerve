package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;

public class AutonManager {
    private SwerveSubsystem driveSubsystem;

    public AutonManager(SwerveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    public Command followPathCommand(String path) {
        List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup(path, 
            PathPlanner.getConstraintsFromPath(path));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            driveSubsystem::getPose, 
            driveSubsystem::resetOdometry, 
            Swerve.swerveKinematics, 
            Swerve.tranConstants, 
            Swerve.rotConstants, 
            driveSubsystem::setModuleStates, 
            new HashMap<String, Command>(), 
            false, 
            driveSubsystem);

        return autoBuilder.fullAuto(trajectory);
    }
}
