package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberLimited extends Command {
    private double speed;
    private Climber climber;

    public ClimberLimited(Climber climber, double speed) {
        this.climber = climber;
        this.speed = speed;
        addRequirements(climber);
    }
    
    @Override
    public void execute() {
        climber.runLimited(speed);
    }

    @Override
    public void end(boolean interrupted) {
        climber.run(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public class ClimberLimitedFactory {
        Climber climber;
        double speed;
        public ClimberLimitedFactory(Climber climber, double speed) {
            this.climber = climber;
            this.speed = speed;
        }

        public ClimberLimited create() {
            return new ClimberLimited(climber, speed);
        }
        
    }
}
