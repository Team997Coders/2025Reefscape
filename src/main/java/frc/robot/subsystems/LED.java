package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private final LEDPattern red;
    private final AddressableLEDBuffer m_buffer;
    private final AddressableLEDBufferView m_left;
    private final AddressableLEDBufferView m_right;
    private final AddressableLED m_led;
    private final LEDPattern blue;
    private final LEDPattern purple;

    public LED() {
        m_buffer = new AddressableLEDBuffer(120);
        m_left = m_buffer.createView(0, 59);
        m_right = m_buffer.createView(60, 119).reversed();
        m_led = new AddressableLED(0);
        
        // Create an LED pattern that sets the entire strip to solid red
        red = LEDPattern.solid(Color.kRed);
        blue = LEDPattern.solid(Color.kBlue);
        purple = LEDPattern.solid(Color.kPlum);
    }

    public void SetColor() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                // Apply the LED pattern to the data buffer
                red.applyTo(m_buffer);
            }
            if (ally.get() == Alliance.Blue) {
                blue.applyTo(m_buffer);
            }
        } else {
            purple.applyTo(m_buffer);
        }
        // Write the data to the LED strip
        m_led.setData(m_buffer);
    }

}
