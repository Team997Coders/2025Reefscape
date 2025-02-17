package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;

public class AlgaeToggleIntake extends Command {
    private Algae m_algae;
    private BooleanSupplier m_a_button;
    private Boolean running;
    private Boolean last_a;
    private BooleanSupplier m_b_button;

    public AlgaeToggleIntake(Algae algae, BooleanSupplier a_button, BooleanSupplier b_button){
        m_algae = algae;
        m_a_button = a_button;
        m_b_button = b_button;
        running = false;
        last_a = false;

        addRequirements(algae);
    }

    @Override
    public void execute(){
        if (m_b_button.getAsBoolean()){
            m_algae.SpinMotor(-0.5);
        } else
        {
        if (m_a_button.getAsBoolean() && !last_a){
            running = !running;
        }
        if (running){
            m_algae.SpinMotor(0.7);
        } else {
            m_algae.SpinMotor(0);
        }
    }
        last_a = m_a_button.getAsBoolean();
    }

    @Override 
    public void end(boolean interrupted) {
        m_algae.AlgaeStop();
    }
    
}
