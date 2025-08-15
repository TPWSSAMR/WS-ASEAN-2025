package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Odom {

    private Pose2d m_pose;

    public Odom() {
        m_pose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    }

    public Odom(Pose2d initialPose) {
        m_pose = initialPose;
    }

    public void resetPose() {
        m_pose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    }

    public void setPose(Pose2d pose) {
        m_pose = pose;
    }

    public Pose2d getPose() {
        return m_pose;
    }

    public void update(double dx, double dy, double dth) {
        Translation2d newxy = m_pose.getTranslation().plus(new Translation2d(dx, dy));
        Rotation2d newth = m_pose.getRotation().plus(new Rotation2d(dth));

        m_pose = new Pose2d(newxy, newth);
    }

}