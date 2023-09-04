// In Java We Trust

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.Swerve;

public class RobotContainer {

    private SwerveSubsystem robotSwerve;

    private CommandXboxController controller;

    public RobotContainer() {
        robotSwerve = new SwerveSubsystem();

        controller = new CommandXboxController(0);

        configureBindings();

        robotSwerve.setDefaultCommand(new SwerveCommand(
            robotSwerve, 
            () -> controller.getLeftY() * 0.5, 
            () -> - controller.getLeftX() * 0.5, 
            () -> - controller.getRightX(), 
            () -> Constants.Swerve.fieldRelative
        ));
    }


    private void configureBindings() {
        controller.x().whileTrue(new InstantCommand( () -> { 
            robotSwerve.resetModules(); 
        }));

        controller.b().onTrue(new InstantCommand( () -> { 
            robotSwerve.zeroGyro();
        }));

        controller.a().onTrue(new InstantCommand( () -> { 
            Swerve.fieldRelative = !Swerve.fieldRelative;
        }));
    }

    public Command getAutonomousCommand() {
        return robotSwerve.followPathGroup(new HashMap<String, Command>(), true, "testPath");
    }

    public SwerveSubsystem getDrive() {
        return robotSwerve;
    }
}
