package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utils.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivebaseSubsystem extends SubsystemBase {
    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_SwerveMods;
    // public Pigeon2 m_gyro;
    public PigeonIMU m_gyro;

    private GenericEntry m_forwardAdjustmentTableEntry;
    private GenericEntry m_sidewaysAdjustmentTableEntry;
    private GenericEntry m_rotationalAdjustmentTableEntry;

    private static final double INITIAL_INPUT_ADJUSTMENT = 0.25;

    private double m_tempEncoderCount = 0;
    private int m_encoderIteration = 0;
    private double m_encoderRateOfChange = 0;
    private int m_encoderUpdateCounter = 0;
    private static final double ROC_DT_SECONDS = 0.02;

    public DrivebaseSubsystem() {
        ShuffleboardTab m_drivebaseTab = Shuffleboard.getTab("Drivebase");

        // m_gyro = new Pigeon2(Constants.Swerve.pigeonID);
        m_gyro = new PigeonIMU(Constants.Swerve.pigeonID);
        m_gyro.configFactoryDefault();
        zeroGyro();

        m_SwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        m_swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

        for (SwerveModule mod : m_SwerveMods) {
            DriverStation.reportError(
                    "CANcoder on Module " + mod.m_moduleNumber + " took " + mod.CANcoderInitTime + " ms to be ready.",
                    false);
        }

        // Add widgets to adjust controller input values and robot-v-field orientation
        m_forwardAdjustmentTableEntry = m_drivebaseTab.add("Forward Adj", INITIAL_INPUT_ADJUSTMENT)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", INITIAL_INPUT_ADJUSTMENT, "max", 1))
                .withSize(2, 1)
                .withPosition(0, 3)
                .getEntry();

        m_sidewaysAdjustmentTableEntry = m_drivebaseTab.add("Sideways Adj", INITIAL_INPUT_ADJUSTMENT)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", INITIAL_INPUT_ADJUSTMENT, "max", 1))
                .withSize(2, 1)
                .withPosition(2, 3)
                .getEntry();

        m_rotationalAdjustmentTableEntry = m_drivebaseTab.add("Rotational Adj", INITIAL_INPUT_ADJUSTMENT)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", INITIAL_INPUT_ADJUSTMENT, "max", 1))
                .withSize(2, 1)
                .withPosition(4, 3)
                .getEntry();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : m_SwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
        }
    }

    // Used by SwerveControllerCommand in Auto 
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : m_SwerveMods) {
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveDriveOdometry getOdometry() {
        return m_swerveOdometry;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_SwerveMods) {
            positions[mod.m_moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        m_gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw())
                : Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    public double getForwardAdjustment() {
        return m_forwardAdjustmentTableEntry.getDouble(INITIAL_INPUT_ADJUSTMENT);
    }

    public double getSidewaysAdjustment() {
        return m_sidewaysAdjustmentTableEntry.getDouble(INITIAL_INPUT_ADJUSTMENT);
    }

    public double getRotationalAdjustment() {
        return m_rotationalAdjustmentTableEntry.getDouble(INITIAL_INPUT_ADJUSTMENT);
    }

    // This method is used to determine if the robot has stopped moving
    // during an autonomous command being run inside a thread. We need
    // a way of killing the tread if the robot is obstructed for some
    // unforseen reason. We'll just pick one of the drive motors to
    // monitor its movement.
    private double getEncoderCount() {
        return m_SwerveMods[0].getDriveMotor().getSelectedSensorPosition();
    }

    private void calculateEncoderRoC() {
        if(m_encoderIteration == (ROC_DT_SECONDS * 50)) {

            double encoderCount = getEncoderCount();

            m_encoderRateOfChange = (encoderCount - m_tempEncoderCount) / ROC_DT_SECONDS;
            m_encoderIteration = 0;
            m_tempEncoderCount = encoderCount;
        } else {
            m_encoderIteration++;
        }
    }

    public double getEncoderRateOfChange() {
        return m_encoderRateOfChange;
    }

    public void stopMotors() {
        System.out.println("stopMotors");
        for (SwerveModule mod : m_SwerveMods) {
            mod.getDriveMotor().set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

    public void setMotorsToCoast() {
        System.out.println("setMotorsToCoast");
        for (SwerveModule mod : m_SwerveMods) {
            mod.getDriveMotor().setNeutralMode(NeutralMode.Coast);
            mod.getAngleMotor().setNeutralMode(NeutralMode.Coast);
        }
    }

    public void setMotorsToBrake() {
        System.out.println("setMotorsToBrake");
        for (SwerveModule mod : m_SwerveMods) {
            mod.getDriveMotor().setNeutralMode(NeutralMode.Brake);
            mod.getAngleMotor().setNeutralMode(NeutralMode.Brake);
        }
    }

    @Override
    public void periodic() {

        // Calling calculateEncoderRoc every period causes loop overruns
        // so we'll only do it once a second.
        if (m_encoderUpdateCounter > 50) {
            calculateEncoderRoC();
            m_encoderUpdateCounter = 0;
        }

        m_swerveOdometry.update(getYaw(), getModulePositions());

        for (SwerveModule mod : m_SwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}