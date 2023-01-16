package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Controllers 
    private final XboxController m_xboxController = new XboxController(0);
    private final Joystick m_joystick = new Joystick(1);
 
    // Drive Controls 
    private final int m_translationAxis = XboxController.Axis.kLeftY.value;
    private final int m_strafeAxis = XboxController.Axis.kLeftX.value;
    private final int m_rotationAxis = XboxController.Axis.kRightX.value;

    // Joystick Buttons 
    private final JoystickButton m_zeroGyroJoystickButton = new JoystickButton(m_joystick,
            XboxController.Button.kY.value);
    private final JoystickButton m_robotCentricJoystickButton = new JoystickButton(m_joystick,
            XboxController.Button.kLeftBumper.value);

    // Xbox Buttons 
    private Trigger m_zeroGryoXboxControllerButton = new Trigger(m_xboxController::getBackButton);
    private Trigger m_robotCentricXboxControllerButton = new Trigger(m_xboxController::getLeftBumper);

    // Subsystems 
    private final static DrivebaseSubsystem m_drivebaseSubsystem = new DrivebaseSubsystem();
    private final static AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
    private final static LEDStripSubsystem m_ledStripSubsystem = new LEDStripSubsystem();

    public RobotContainer() {

        // Joystick
        m_drivebaseSubsystem.setDefaultCommand(
                new TeleopSwerveDriveCommand(
                        m_drivebaseSubsystem,
                        () -> -m_joystick.getRawAxis(m_translationAxis) * m_drivebaseSubsystem.getForwardAdjustment()
                                * Constants.Swerve.maxSpeed,
                        () -> -m_joystick.getRawAxis(m_strafeAxis) * m_drivebaseSubsystem.getSidewaysAdjustment()
                                * Constants.Swerve.maxSpeed,
                        () -> -m_joystick.getRawAxis(m_rotationAxis) * m_drivebaseSubsystem.getRotationalAdjustment()
                                * Constants.Swerve.maxAngularVelocity,
                        () -> m_robotCentricJoystickButton.getAsBoolean()));

        // Xbox Controller
        m_drivebaseSubsystem.setDefaultCommand(
                new TeleopSwerveDriveCommand(
                        m_drivebaseSubsystem,
                        () -> -m_xboxController.getLeftY() * m_drivebaseSubsystem.getForwardAdjustment()
                                * Constants.Swerve.maxSpeed,
                        () -> -m_xboxController.getLeftX() * m_drivebaseSubsystem.getSidewaysAdjustment()
                                * Constants.Swerve.maxSpeed,
                        () -> -m_xboxController.getRightX() * m_drivebaseSubsystem.getRotationalAdjustment()
                                * Constants.Swerve.maxAngularVelocity,
                        () -> m_robotCentricXboxControllerButton.getAsBoolean()));

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new InstantCommand(() -> {
            m_ledStripSubsystem.rainbow();
        }));

        // Joystick Buttons 
        m_zeroGyroJoystickButton.onTrue(new InstantCommand(() -> m_drivebaseSubsystem.zeroGyro()));

        // XboxController Buttons 
        m_zeroGryoXboxControllerButton.onTrue(new InstantCommand(() -> m_drivebaseSubsystem.zeroGyro()));

        new Trigger(m_xboxController::getYButton)
                .onTrue(new DockWithAprilTagCommand(m_xboxController, m_drivebaseSubsystem, m_aprilTagSubsystem, false,
                        1.0));

        new Trigger(m_xboxController::getBButton)
                .onTrue(new InstantCommand(m_drivebaseSubsystem::stopMotors, m_drivebaseSubsystem));

        new Trigger(m_xboxController::getAButton)
                .onTrue(new SequentialCommandGroup(
                        /*new InstantCommand(m_drivebaseSubsystem::setDoingTeleOpAuto),*/
                        new InstantCommand(m_drivebaseSubsystem::setMotorsToBrake),
                        new FollowTrajectoryCommand(m_drivebaseSubsystem, "NewStraight", eventMap, 4.0, 3.0, true),
                        new FollowTrajectoryCommand(m_drivebaseSubsystem, "NewStraightBack", eventMap, 4.0, 3.0, true)/*,
                        new InstantCommand(() -> {
                            m_drivebaseSubsystem.setNotDoingTeleOpAuto();
                        } m_drivebaseSubsystem)*/));
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new ExampleAuto(m_drivebaseSubsystem);
    }

    public static DrivebaseSubsystem getDrivebaseSubsystem() {
        return m_drivebaseSubsystem;
    }

    public static AprilTagSubsystem getAprilTagSubsystem() {
        return m_aprilTagSubsystem;
    }

    public static LEDStripSubsystem getLEDStripSubsystem() {
        return m_ledStripSubsystem;
    }
}
