package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerveDriveCommand extends CommandBase {
    private DrivebaseSubsystem m_drivebaseSubsystem;
    private DoubleSupplier m_translationSup;
    private DoubleSupplier m_strafeSup;
    private DoubleSupplier m_rotationSup;
    private BooleanSupplier m_robotCentricSup;

    public TeleopSwerveDriveCommand(DrivebaseSubsystem drivebaseSubystems, DoubleSupplier translationSup,
            DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        m_drivebaseSubsystem = drivebaseSubystems;
        addRequirements(drivebaseSubystems);

        m_translationSup = translationSup;
        m_strafeSup = strafeSup;
        m_rotationSup = rotationSup;
        m_robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(-m_translationSup.getAsDouble(), Constants.joystickDeadband);
        double strafeVal = MathUtil.applyDeadband(-m_strafeSup.getAsDouble(), Constants.joystickDeadband);
        double rotationVal = MathUtil.applyDeadband(-m_rotationSup.getAsDouble(), Constants.joystickDeadband);

        /* Drive */
        m_drivebaseSubsystem.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !m_robotCentricSup.getAsBoolean(),
                false);
    }
}