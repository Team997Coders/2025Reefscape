package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Algae extends SubsystemBase{
    
    private final SparkMax spinnyMotor;
    private final SparkMaxConfig spinnyMotorConfig;
    private final DigitalInput beamBreak;

    public Algae() {
        spinnyMotor = new SparkMax(Constants.Algae.spinnyMotorID, MotorType.kBrushless);
        spinnyMotorConfig = new SparkMaxConfig();

        spinnyMotorConfig.smartCurrentLimit(Constants.Algae.spinnyMotorConfig);
        spinnyMotor.configure(spinnyMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        beamBreak = new DigitalInput(Constants.Algae.beamBreakID);
    
    }
    @Override
    public void periodic() {
        loggers();
    }

    public void SpinMotor(double speed){
        spinnyMotor.set(speed);
    }
    
    public boolean getBeamBreakStatus() {
        return beamBreak.get();
    }

    public Command AlgaeIntake() {
        spinnyMotorConfig.smartCurrentLimit(Constants.Algae.spinnyMotorConfig);
        spinnyMotor.configure(spinnyMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return this.startEnd(() -> SpinMotor(Constants.Algae.motorSpin), () -> SpinMotor(0));
    }



    public Command AlgaeHold() {
        spinnyMotorConfig.smartCurrentLimit(Constants.Algae.spinnyMotorConfig/10);
        spinnyMotor.configure(spinnyMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return this.runOnce(() -> SpinMotor(Constants.Algae.motorHold));
    }

    public Command AlgaeOuttake() {
        spinnyMotorConfig.smartCurrentLimit(Constants.Algae.spinnyMotorConfig);
        spinnyMotor.configure(spinnyMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return this.startEnd(() -> SpinMotor(-Constants.Algae.motorSpin), () -> SpinMotor(0));
    }

    public Command AlgaeStop() {
        return this.runOnce(() -> SpinMotor(0));
    }

    public void loggers() {
        SmartDashboard.putBoolean("algae beambreak", getBeamBreakStatus());
    }
}
