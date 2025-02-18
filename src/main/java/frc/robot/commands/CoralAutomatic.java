package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Coral;

public class CoralAutomatic extends Command {
    private Coral m_coral;

    private Trigger m_y_button;

    private Trigger m_beambreak1;
    private Trigger m_beambreak2;

    public CoralAutomatic(Coral coral, Trigger y_button, Trigger beambreak1, Trigger beambreak2){
        m_coral = coral;
        m_y_button = y_button;

        m_beambreak1 = beambreak1;
        m_beambreak2 = beambreak2;

        addRequirements(coral);
    }

    @Override
    public void execute(){
        m_beambreak1.onTrue(m_coral.manualMoveCoralMotorsIntake()).onFalse(m_coral.CoralStop());
        m_beambreak1.and(m_beambreak2).onTrue(m_coral.manualMoveCoralMotorsIntake()).onFalse(m_coral.CoralStop());
        m_beambreak2.and(m_y_button).onTrue(m_coral.manualMoveCoralMotorsOutake()).onFalse(m_coral.CoralStop());  
    }
    
}
