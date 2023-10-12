// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase
{
    private NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");

    private NetworkTableEntry m_tx = m_limelight.getEntry("tx");
    private NetworkTableEntry m_ty = m_limelight.getEntry("ty");
    private NetworkTableEntry m_ta = m_limelight.getEntry("ta");

    public VisionSubsystem() 
    {
        m_limelight.getEntry("pipeline").setNumber(0); // Default pipeline
    }

    @Override
    public void periodic() 
    {
        SmartDashboard.putNumber("/vision/x", getLimelightX());
        SmartDashboard.putNumber("/vision/y", getLimelightY());
        SmartDashboard.putNumber("/vision/area", getLimelightArea());

        SmartDashboard.putNumber("/vision/latency", getLimelightLatency());
    }

    public double getLimelightX()
    {
        return m_tx.getDouble(0.0);
    }

    public double getLimelightY()
    {
        return m_ty.getDouble(0.0);
    }

    public double getLimelightArea()
    {
        return m_ta.getDouble(0.0);
    }

    public double getLimelightLatency()
    {
        double tableLatency = m_limelight.getEntry("tl").getDouble(0.0);
        double cameraLatency = m_limelight.getEntry("cl").getDouble(0.0);

        return (tableLatency + cameraLatency) / 1000.0;
    }

     public Pose2d getEstimatedPose()
     {
        var estimatedPose = m_limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        Translation2d translation = new Translation2d(estimatedPose[0], estimatedPose[1]);
        Rotation2d rotation = new Rotation2d(Math.toRadians(estimatedPose[3]));

        return new Pose2d(translation, rotation);
     }

     public Pose2d getTarget()
     {
        var estimatedTargetPose = m_limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        Translation2d translation = new Translation2d(estimatedTargetPose[0], estimatedTargetPose[1]);
        Rotation2d rotation = new Rotation2d(Math.toRadians(estimatedTargetPose[3]));

        return new Pose2d(translation, rotation);
     }
}
