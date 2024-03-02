package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherV2;



public class VelocityLauncher extends Command {
    LauncherV2 launcherSubsystem;
    DoubleSupplier leftVelocitySupplier;
    DoubleSupplier rightVelocitySupplier;

    public VelocityLauncher(LauncherV2 launcherSubsystem, DoubleSupplier leftVelocitySupplier, DoubleSupplier rightVelocitySupplier) {
        this.launcherSubsystem = launcherSubsystem;
        this.leftVelocitySupplier = leftVelocitySupplier;
        this.rightVelocitySupplier = rightVelocitySupplier;
        addRequirements(launcherSubsystem);
    }

    @Override
    public void execute() {
        launcherSubsystem.setVelocity(leftVelocitySupplier.getAsDouble(), rightVelocitySupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        launcherSubsystem.setSpeed(0,0);
    }
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }

    @Override
    public void initialize() {
        launcherSubsystem.setFF( 0.1 * ((leftVelocitySupplier.getAsDouble() + rightVelocitySupplier.getAsDouble()) /2));
        launcherSubsystem.reset();
        
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}