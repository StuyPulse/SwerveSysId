package com.stuypulse.robot.subsystems.module;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.SwerveModule;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimVoltageSwerveModule extends SubsystemBase implements SwerveModule {

    private interface Turn {
        SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 3.5);
        SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0.0);
        SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.0);
		
        double kS = 0.14;
        double kV = 0.25;
        double kA = 0.007;
    }

	private interface Drive {
        double kS = 0.11114;
        double kV = 2.7851;
        double kA = 0.30103;
	}

    // module data

    private final String id;
    private final Translation2d location;

    // turn

	private final LinearSystemSim<N2, N1, N1> turnSim;

    private final AngleController turnController;

    // drive
	private final LinearSystemSim<N1, N1, N1> driveSim;
	private double distance;

    private double voltage;

    public SimVoltageSwerveModule(String id, Translation2d location) {
        // module data
        this.id = id;
        this.location = location;

        // turn

		turnSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(Turn.kV, Turn.kA));

        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD);

        // drive

        voltage = 0.0;

		driveSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(Drive.kV, Drive.kA));
		distance = 0.0;
    }

    @Override
    public String getId() {
        return id;
    }

    @Override
    public Translation2d getLocation() {
        return location;
    }

    @Override
    public double getVelocity() {
        return driveSim.getOutput(0);
    }

    private Rotation2d getRotation2d() {
        return new Rotation2d(turnSim.getOutput(0));
    }


    @Override
    public double getDistance() {
        return distance;
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void periodic() {
        turnSim.setInput(turnController.update(Angle.kZero, Angle.fromRotation2d(getRotation2d())));
        driveSim.setInput(voltage);

		distance += getVelocity() * Settings.dT;

		turnSim.update(Settings.dT);
		driveSim.update(Settings.dT);

        SmartDashboard.putNumber(id + "/Angle", getRotation2d().getDegrees());
        SmartDashboard.putNumber(id + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber(id + "/Angle Voltage", turnController.getOutput());

        SmartDashboard.putNumber(id + "/Velocity", getVelocity());

        SmartDashboard.putNumber(id + "/Drive Voltage", voltage);
    }

}