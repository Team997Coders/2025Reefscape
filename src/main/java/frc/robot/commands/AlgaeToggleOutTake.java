package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;

public class AlgaeToggleOutTake extends Command {
    private Algae m_algae;

    public AlgaeToggleOutTake(Algae algae){
        m_algae = algae;
    }

    @Override
    public void execute(){
        m_algae.AlgaeOuttake();
    }

    @Override 
    public void end(boolean interrupted) {
        m_algae.AlgaeStop();

    }
    
}
