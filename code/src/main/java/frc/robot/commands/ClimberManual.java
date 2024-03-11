package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberManual extends Command {
    Climber climber;
    DoubleSupplier speed;
    public ClimberManual(Climber climber, DoubleSupplier speed) {
        this.climber = climber;
        this.speed = speed;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.runRaw(speed.getAsDouble());
    }

    @Override 
    public void end(boolean interrupted) {
        climber.runRaw(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    public class ClimberManualFactory {
        Climber climber;
        DoubleSupplier speed;
        public ClimberManualFactory(Climber climber, DoubleSupplier speed) {
            this.climber = climber;
            this.speed = speed;
        }

        public ClimberManual create() {
            return new ClimberManual(climber, speed);
        }
    }
}
