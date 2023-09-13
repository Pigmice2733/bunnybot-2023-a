// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("HD_Pro_Webcam_C920");
    private PhotonTrackedTarget currentTarget;

    public Vision() {

    }

    @Override
    public void periodic() {
        var results = camera.getLatestResult();
        if (!results.hasTargets()) {
            currentTarget = null;
            return;
        }

        currentTarget = results.getBestTarget();
        SmartDashboard.putNumber("yaw", currentTarget.getYaw());
        SmartDashboard.putNumber("pitch", currentTarget.getPitch());
        SmartDashboard.putNumber("skew", currentTarget.getSkew());
        SmartDashboard.putNumber("area", currentTarget.getArea());
    }

    public PhotonTrackedTarget getCurrentTarget() {
        return currentTarget;
    }
}
