package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;

public class CoralDefaultCommand extends Command {
    private Coral m_coral;
    private BooleanSupplier m_x_button;
    private BooleanSupplier m_y_button;
    private BooleanSupplier m_right_bumper;
    private Boolean automatic;
    private Boolean last_right_bumper;

    public CoralDefaultCommand(Coral coral, BooleanSupplier x_button, BooleanSupplier y_button, BooleanSupplier right_bumper){
        m_coral = coral;
        m_x_button = x_button;
        m_y_button = y_button;
        m_right_bumper = right_bumper;
        automatic = false;
        last_right_bumper = false;

        addRequirements(coral);
    }

    @Override
    public void execute(){
        if (!last_right_bumper && m_right_bumper.getAsBoolean())
        {
            automatic = !automatic;
        }
        if (automatic)
        {   
            if (!m_coral.BeamBrake1() )
            {
                m_coral.spinBothMotors(-0.5);
            } else 
            {
                if (!m_coral.BeamBrake2())
                {
                    m_coral.spinBothMotors(0);
                    if (m_y_button.getAsBoolean())
                    {
                        m_coral.spinBothMotors(-1);
                    }
                } else
                {
                    m_coral.spinBothMotors(0);
                }
            }
        } else 
        {
            m_coral.spinBothMotors(0);
            if (m_x_button.getAsBoolean())
            {
                m_coral.spinBothMotors(-0.5);
            }
            if (m_y_button.getAsBoolean())
            {
                m_coral.spinBothMotors(-1);
            }
        }

        last_right_bumper = m_right_bumper.getAsBoolean();
    }

    @Override 
    public void end(boolean interrupted) {
        m_coral.spinBothMotors(0);
    }
    
}
