package frc.robot.utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;

public class DockWithAprilTag implements Runnable {

    private XboxController m_xboxController;
    private DrivebaseSubsystem m_drivebaseSubsystem;
    private AprilTagSubsystem m_aprilTagSubsystem;
    private double m_aprilTagId;
    private boolean m_isCameraForward;

    private boolean m_hasStartedMoving;

    private static double kDt = 0.02;

    // Distance to Target Correction
    private static final double MAX_FORWARD_DOCKING_VELOCITY = 1.2;
    private static final double MAX_FORWARD_DOCKING_ACCELERATION = 0.3;

    private final TrapezoidProfile.Constraints m_forwardConstraints = new TrapezoidProfile.Constraints(
            MAX_FORWARD_DOCKING_VELOCITY, MAX_FORWARD_DOCKING_ACCELERATION);

    private final double FORWARD_P = 0.6;
    private final double FORWARD_D = 0.0;
    private final ProfiledPIDController m_forwardController = new ProfiledPIDController(FORWARD_P, 0.0, FORWARD_D,
            m_forwardConstraints, kDt);

    // Sideways Correction
    private static final double MAX_SIDEWAYS_DOCKING_VELOCITY = 1.0;
    private static final double MAX_SIDEWAYS_DOCKING_ACCELERATION = 0.3;

    private final TrapezoidProfile.Constraints m_sidewaysConstraints = new TrapezoidProfile.Constraints(
            MAX_SIDEWAYS_DOCKING_VELOCITY, MAX_SIDEWAYS_DOCKING_ACCELERATION);

    private final double SIDEWAYS_P = 0.6;
    private final double SIDEWAYS_D = 0.0;
    private final ProfiledPIDController m_sidewaysController = new ProfiledPIDController(SIDEWAYS_P, 0.0, SIDEWAYS_D,
            m_sidewaysConstraints, kDt);

    // We'll make this a little larger to give the AprilTag detector some time to
    // process
    private static final double DOCKING_DISTANCE_GOAL_METERS = Units.inchesToMeters(20.0);

    private static final double MIN_FORWARD_VELOCITY = 0.2;
    private static final double MIN_SIDEWAYS_VELOCITY = 0.2;

    // If the AprilTag detection loss lasts this amount of time, then we give up
    // trying to reacquire the AprilTag
    private static final double MAX_DETECTION_LOST_TIME_SEC = 0.3;

    private double m_startTime = 0;

    public DockWithAprilTag(XboxController xboxController,
            DrivebaseSubsystem drivebaseSubsystem,
            AprilTagSubsystem aprilTagSubsystem,
            boolean isCameraForward,
            double aprilTagId) {

        m_xboxController = xboxController;
        m_drivebaseSubsystem = drivebaseSubsystem;
        m_aprilTagSubsystem = aprilTagSubsystem;
        m_isCameraForward = isCameraForward;
        m_aprilTagId = aprilTagId;
    }

    public void run() {
        m_hasStartedMoving = false;

        if (m_aprilTagSubsystem.getTagID() == m_aprilTagId) {

            m_startTime = Timer.getFPGATimestamp();

            m_forwardController.setGoal(0.0); // DOCKING_DISTANCE_GOAL_METERS?
            m_sidewaysController.setGoal(0.0);

            double detectionLostTime = 0.0;

            while (true) {

                // If the AprilTag comes up as 2228 that means the detector can't see a
                // tag. We should wait a bit to see if the detection loss is just transitory
                // before giving up on it.
                if ((m_aprilTagSubsystem.getTagID() == Constants.BAD_APRIL_TAG_ID) &&
                        (detectionLostTime == 0.0)) {
                    detectionLostTime = Timer.getFPGATimestamp();
                } else if (m_aprilTagSubsystem.getTagID() == m_aprilTagId) {
                    detectionLostTime = 0.0;
                }

                if ((detectionLostTime != 0.0) &&
                        ((Timer.getFPGATimestamp() - detectionLostTime) > MAX_DETECTION_LOST_TIME_SEC)) {
                    System.out.println("Completely Lost April Tag Detection...");
                    break;
                }

                if (m_xboxController.getBButton()) {
                    System.out.println("Driver cancelled command...");
                    break;
                }

                if (m_drivebaseSubsystem.getEncoderRateOfChange() > 0) {
                    m_hasStartedMoving = true;
                }

                // If we've started moving but then stop moving due to some unforseen issue
                // like being blocked by another robot or field element, we need to kill the
                // thread.
                if (m_hasStartedMoving && m_drivebaseSubsystem.getEncoderRateOfChange() == 0) {
                    System.out.println("Robot has stopped moving...");
                    break;
                }

                double distanceToTarget = m_aprilTagSubsystem.getTZ();
                double offsetTargetDistance = m_aprilTagSubsystem.getTX();

                double forwardSpeed = -m_forwardController.calculate(distanceToTarget);
                double sidewaysSpeed = m_sidewaysController.calculate(offsetTargetDistance);

                // For velocity PID control, getPositionError returns the velocity error and
                // getVelocity error returns the acceleration error ... kind of messed up
                // naming.
                // double forwardVelocityError = m_forwardController.getPositionError();
                // double forwardAccelerationError = m_forwardController.getVelocityError();
                // double sidewaysVelocityError = m_sidewaysController.getPositionError();
                // double sidewaysAccelerationError = m_sidewaysController.getVelocityError();

                // System.out.printf("FVE: %.2f FAE: %.2f SVE: %.2f SAE: %.2f\n",
                // forwardVelocityError,
                // forwardAccelerationError,
                // sidewaysVelocityError,
                // sidewaysAccelerationError);

                double forwardVelocity = forwardSpeed * MAX_FORWARD_DOCKING_VELOCITY;
                double sidewaysVelocity = sidewaysSpeed * MAX_SIDEWAYS_DOCKING_VELOCITY;

                // Need to ensure minimum velocities that are high enough to move the robot
                if (forwardVelocity < MIN_FORWARD_VELOCITY) {
                    forwardVelocity = MIN_FORWARD_VELOCITY;
                }

                if ((sidewaysVelocity > 0.0) && (sidewaysVelocity < MIN_SIDEWAYS_VELOCITY)) {
                    sidewaysVelocity = MIN_SIDEWAYS_VELOCITY;
                }

                if ((sidewaysVelocity < 0.0) && (sidewaysVelocity > -MIN_SIDEWAYS_VELOCITY)) {
                    sidewaysVelocity = -MIN_SIDEWAYS_VELOCITY;
                }

                m_drivebaseSubsystem.drive(new Translation2d(forwardVelocity, sidewaysVelocity), 0.0,
                        !m_isCameraForward, false);

                // Check to see if we're within docking distance
                if (distanceToTarget < DOCKING_DISTANCE_GOAL_METERS) {
                    System.out.println("Docked with target. Yipee!!!!!");
                    System.out
                            .println("Command completed in " + (Timer.getFPGATimestamp() - m_startTime) + " seconds");
                    break;
                }

                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        } else {
            System.out.printf("April tag: %.0f not detected!\n", m_aprilTagId);
        }

        m_drivebaseSubsystem.stopMotors();
    }
}
