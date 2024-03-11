package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command{
    Intake intake;
    double speed;

    public RunIntake( Intake intake, double speed) {
        this.speed = speed;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.run(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.run(0);
    }
    public class RunIntakeFactory {
        Intake intake;
        double speed;
        public RunIntakeFactory(Intake intake, double speed) {
            this.intake = intake;
            this.speed = speed;
        }

        public RunIntake create() {
            return new RunIntake(intake, speed);
        }
    
        
    }
}
