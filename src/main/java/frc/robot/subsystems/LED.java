package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Algae;
import frc.robot.commands.AlgaeToggleIntake;


import java.util.Optional;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LEDCommand;


public class LED extends SubsystemBase{

    private final LEDPattern red;
    private final AddressableLEDBuffer m_buffer;
    private final AddressableLED m_led;
    private final LEDPattern blue;
    private final LEDPattern purple;
    private final LEDPattern gray;
    private final LEDPattern green;
    private final LEDPattern redPattern;
    private final LEDPattern bluePattern;



//     // Create an LED pattern that displays a red-to-blue gradient, then scroll at one quarter of the LED strip's length per second.
// // For a half-meter length of a 120 LED-per-meter strip, this is equivalent to scrolling at 12.5 centimeters per second.
// Distance ledSpacing = Meters.of(1 / 120.0);
// LEDPattern base = LEDPattern.discontinuousGradient(Color.kRed, Color.kBlue);
// LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
// LEDPattern absolute = base.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSpacing);

// // Apply the LED pattern to the data buffer
// pattern.applyTo(m_ledBuffer);

// // Write the data to the LED strip
// m_led.setData(m_ledBuffer);
    public LED(Coral coral){
        
        m_buffer = new AddressableLEDBuffer(120);
        m_led = new AddressableLED(0);

        red = LEDPattern.gradient(GradientType.kContinuous, Color.kFirstRed, Color.kOrangeRed);
        blue = LEDPattern.gradient(GradientType.kContinuous, Color.kFirstBlue, Color.kLightBlue);
        purple = LEDPattern.solid(Color.kPlum);
        gray = LEDPattern.solid(Color.kGainsboro);
        green = LEDPattern.solid(Color.kLawnGreen);


        redPattern = red.scrollAtRelativeSpeed(Percent.per(Second).of(25));
        bluePattern = blue.scrollAtRelativeSpeed(Percent.per(Second).of(25));


    }
// Use int for LEDS when we have Coral, Alege or nothing
     public void ItemPickedUp(int x) {
        if (x == 1)
            LEDPattern.solid(Color.kGainsboro);
        }
        if (x == 2){
             LEDPattern.solid(Color.kLawnGreen);
        }
        else{
            SetColor();
        }
 }
            

                 
            
            
 private void print(String string) {
// TODO Auto-generated method stub
throw new UnsupportedOperationException("Unimplemented method 'print'");
                }
            
                    public void SetColor() {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    // Apply the LED pattern to the data buffer
                    redPattern.applyTo(m_buffer);
                }
                if (ally.get() == Alliance.Blue) {
                    bluePattern.applyTo(m_buffer);
                }
            }
             else {
                purple.applyTo(m_buffer);
            }
            // Write the data to the LED strip
            m_led.setData(m_buffer);
        }

}




