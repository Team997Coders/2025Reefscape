package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Algae extends SubsystemBase{
    
    private final SparkMax spinnyMotor;
    private final SparkMaxConfig spinnyMotorConfig;
    private final DigitalInput proximitySensor;

    public Algae() {
        spinnyMotor = new SparkMax(Constants.Algae.spinnyMotorID, MotorType.kBrushless);
        spinnyMotorConfig = new SparkMaxConfig();

        spinnyMotorConfig.smartCurrentLimit(Constants.Algae.spinnyMotorConfig);
        spinnyMotor.configure(spinnyMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        proximitySensor = new DigitalInput(Constants.Algae.proximitySensorID);
    
    }
    @Override
    public void periodic() {
        loggers();
    }

    public void SpinMotor(double speed){
        spinnyMotor.set(speed);
    }
    
    public boolean getProximitySensor() {
        return proximitySensor.get();
    }

    public Command AlgaeIntake(double speed) {
        spinnyMotorConfig.smartCurrentLimit(Constants.Algae.spinnyMotorConfig);
        spinnyMotor.configure(spinnyMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return this.runOnce(() -> SpinMotor(speed));
    }

    public Command AlgaeHold() {
        spinnyMotorConfig.smartCurrentLimit(Constants.Algae.spinnyMotorConfig/10);
        spinnyMotor.configure(spinnyMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return this.runOnce(() -> SpinMotor(0.1));
    }

    public Command AlgaeOuttake(double speed) {
        spinnyMotorConfig.smartCurrentLimit(Constants.Algae.spinnyMotorConfig);
        spinnyMotor.configure(spinnyMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return this.runOnce(() -> SpinMotor(-speed));
    }

    public void loggers() {
        SmartDashboard.putBoolean("algae proximity sensor", getProximitySensor());
    }
}

