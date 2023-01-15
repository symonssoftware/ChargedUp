
package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;

public class FollowTrajectoryCommand extends SequentialCommandGroup {

    public FollowTrajectoryCommand(DrivebaseSubsystem drivebaseSubsystem, String pathName, HashMap<String, Command> eventMap,
                                    double maxVelocity, double maxAcceleration, boolean isFirstPath) {

        PathPlannerTrajectory path = PathPlanner.loadPath( pathName, new PathConstraints(1.5, 1));
        Command swerveCommand = new PPSwerveControllerCommand(
                path, 
                drivebaseSubsystem::getPose, // Pose supplier
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDController(0.1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0.12, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(2, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                drivebaseSubsystem::setModuleStates, // Module states consumer
                drivebaseSubsystem // Requires the drive subsystem
        );
        FollowPathWithEvents command = new FollowPathWithEvents(
            swerveCommand,
            path.getMarkers(),
            eventMap
        );
        addCommands (
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                if(isFirstPath) {
                    drivebaseSubsystem.getOdometry().resetPosition(drivebaseSubsystem.getYaw(), drivebaseSubsystem.getModulePositions(), path.getInitialHolonomicPose());
                }
            }),
            command
            
        );
    }
}