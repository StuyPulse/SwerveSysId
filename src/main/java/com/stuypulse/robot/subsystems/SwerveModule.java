package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;

public interface SwerveModule {
    public String getId();
    public Translation2d getLocation();

    public double getVelocity();
    public double getDistance();
    
    public void setVoltage(double voltage);
}
