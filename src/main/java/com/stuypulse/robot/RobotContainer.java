/************************ PROJECT PHIL ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.Dynamic;
import com.stuypulse.robot.commands.Quasistatic;
import com.stuypulse.robot.subsystems.Logger;
import com.stuypulse.robot.subsystems.VoltageSwerve;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Subsystem
    public VoltageSwerve swerve = new VoltageSwerve();
    public Logger logger = new Logger(swerve);

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    private static SmartNumber rampRate = new SmartNumber("Quasistatic Ramp Rate (V per s)", 0.25);
    private static SmartNumber dynamic = new SmartNumber("Dynamic Step Voltage (V)", 7);

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {}

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {}

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Quasistatic Forward", new Quasistatic(this, rampRate));
        autonChooser.addOption("Quasistatic Backward", new Quasistatic(this, -rampRate));
        autonChooser.addOption("Dynamic Forward", new Dynamic(this, dynamic));
        autonChooser.addOption("Dynamic Backward", new Dynamic(this, -dynamic));

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
