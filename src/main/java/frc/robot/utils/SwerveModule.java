package frc.robot.utils;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int m_moduleNumber;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private TalonFX m_angleMotor;
    private TalonFX m_driveMotor;
    private CANCoder m_angleEncoder;

    public double CANcoderInitTime = 0.0;

    SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS,
            Constants.Swerve.driveKV,
            Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        m_moduleNumber = moduleNumber;
        m_angleOffset = moduleConstants.angleOffset;

        // Angle Encoder Config
        m_angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        // Angle Motor Config
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        // Drive Motor Config
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        m_lastAngle = getState().angle;
    }

    /*
     * This is a custom optimize function, since default WPILib optimize assumes
     * continuous controller which CTRE and Rev onboard is not
     */ 
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    m_feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? m_lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        m_angleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        m_lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition());
    }

    private void waitForCanCoder() {
        // Wait for up to 1000 ms for a good CANcoder signal.
        // This prevents a race condition during program startup
        // where we try to synchronize the Falcon encoder to the
        // CANcoder before we have received any position signal
        // from the CANcoder.
        for (int i = 0; i < 100; ++i) {
            m_angleEncoder.getAbsolutePosition();
            if (m_angleEncoder.getLastError() == ErrorCode.OK) {
                break;
            }
            Timer.delay(0.010);
            CANcoderInitTime += 10;
        }
    }

    public void resetToAbsolute() {
        waitForCanCoder(); // Need to remove this if we're going to call reset from Robot.java and DrivebaseSubsystem.java
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - m_angleOffset.getDegrees(),
                Constants.Swerve.angleGearRatio);
        m_angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        m_angleEncoder.configFactoryDefault();
        m_angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        m_angleMotor.configFactoryDefault();
        m_angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        m_angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        m_angleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        m_driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        m_driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference,
                        Constants.Swerve.driveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(),
                        Constants.Swerve.wheelCircumference,
                        Constants.Swerve.driveGearRatio),
                getAngle());
    }

    public TalonFX getDriveMotor() {
        return m_driveMotor;
    }

    public TalonFX getAngleMotor() {
        return m_angleMotor;
    }
}