// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.LoaderV2;
import frc.robot.subsystems.LauncherV2;
import frc.robot.subsystems.Launcher;


public class LauncherIntake extends Command {
    DistanceSensor distanceSensor;
    Loader loaderSubsystem;
    Launcher launcherSubsystem;
    int threshold;
    int stage;
    double speed;
    public LauncherIntake(DistanceSensor distanceSensor, LoaderV2 loaderSubsystem, LauncherV2 launcherSubsystem, int threshold, double speed) {
        addRequirements(loaderSubsystem, launcherSubsystem);
        this.distanceSensor = distanceSensor;
        this.loaderSubsystem = loaderSubsystem;
        this.launcherSubsystem = launcherSubsystem;
        this.threshold = threshold;
        this.speed = -speed;
    }
    public void initialize() {
        stage = 0;
    }
    //wait for threshold to be crossed twice before stopping
    public void execute() {
        if (stage == 0 && distanceSensor.getDistance() < threshold) {
            stage = 1;
        } else if (stage == 1 && distanceSensor.getDistance() > threshold) {
            stage = 2;
        }
        if (stage == 2) {
            loaderSubsystem.run(0);
            launcherSubsystem.setSpeed(0, 0);
        } else {
            loaderSubsystem.run(speed-0.075);
            launcherSubsystem.setSpeed(speed, speed);
        }
    }
    @Override
    public boolean isFinished() {
        if (stage == 2) {
            stage = 0;
            return true;
        }
        return false;
    }

    public class LauncherIntakeFactory {
        DistanceSensor distanceSensor;
        LoaderV2 loaderSubsystem;
        LauncherV2 launcherSubsystem;
        int threshold;
        double speed;
        public LauncherIntakeFactory(DistanceSensor distanceSensor, LoaderV2 loaderSubsystem, LauncherV2 launcherSubsystem, int threshold, double speed) {
            this.distanceSensor = distanceSensor;
            this.loaderSubsystem = loaderSubsystem;
            this.launcherSubsystem = launcherSubsystem;
            this.threshold = threshold;
            this.speed = speed;
        }

        public LauncherIntake create() {
            return new LauncherIntake(distanceSensor, loaderSubsystem, launcherSubsystem, threshold, speed);
        }
    }
}
