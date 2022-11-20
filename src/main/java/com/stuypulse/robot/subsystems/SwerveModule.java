package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    public String getId();
    public Translation2d getLocation();

    public SwerveModuleState getState();

    // in rotations and rotations/s
    public double getDistance();
    public double getVelocity();
    
    public void setVoltage(double voltage);
}
