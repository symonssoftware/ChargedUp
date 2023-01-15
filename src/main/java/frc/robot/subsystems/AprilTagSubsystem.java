package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagSubsystem extends SubsystemBase {

    static NetworkTable m_aprilTagTable = NetworkTableInstance.getDefault().getTable("AprilTag");

    static NetworkTableEntry m_pitchEntry = m_aprilTagTable.getEntry("Pitch");
    static NetworkTableEntry m_txEntry = m_aprilTagTable.getEntry("TX");
    static NetworkTableEntry m_tzEntry = m_aprilTagTable.getEntry("TZ");
    static NetworkTableEntry m_tagIdEntry = m_aprilTagTable.getEntry("Tag ID");

    public AprilTagSubsystem() {
    }

    public double getPitch() {
        return m_pitchEntry.getDouble(Constants.BAD_APRIL_TAG_ID);
    }

    public double getTX() {
        return m_txEntry.getDouble(Constants.BAD_APRIL_TAG_ID);
    }

    public double getTZ() {
        return m_tzEntry.getDouble(Constants.BAD_APRIL_TAG_ID);
    }

    public double getTagID() {
        return m_tagIdEntry.getDouble(Constants.BAD_APRIL_TAG_ID);
    }

    @Override
    public void periodic() {
        // System.out.println("TZ: " + getTZ() + " | TagID: " + getTagID());
        // System.out.println("TagID last_change: " + m_tagID.getInfo().last_change);
    }
}