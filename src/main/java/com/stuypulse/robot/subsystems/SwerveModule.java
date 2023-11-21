package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveModule extends SubsystemBase {
    public abstract Translation2d getLocation();

    public abstract SwerveModuleState getState();
    public abstract SwerveModulePosition getPosition();

    // in rotations and rotations/s
    public abstract double getDistance();
    public abstract double getVelocity();
    
    public abstract void setVoltage(double voltage);
}
