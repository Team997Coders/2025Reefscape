package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.exceptions.unfilledConstant;

public class Elevator extends SubsystemBase {

    private final SparkMax leftSparkMax;
    private final SparkMax rightSparkMax;

    private final SparkBaseConfig leftConfig;
    private final SparkBaseConfig rightConfig;

    private final RelativeEncoder m_encoder;

    private final DigitalInput bottomSwitch;

    public ElevatorState elevatorState;

    public Trigger m_firstBeamBrake;
    public Trigger m_secondBeamBrake;
    
    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.ProfiledPID.kMaxVelocity,
            ElevatorConstants.ProfiledPID.kMaxAcceleration);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(ElevatorConstants.ProfiledPID.kP,
            ElevatorConstants.ProfiledPID.kI, ElevatorConstants.ProfiledPID.kD, m_constraints,
            ElevatorConstants.ProfiledPID.kDt);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.ProfiledPID.kS,
            ElevatorConstants.ProfiledPID.kG, ElevatorConstants.ProfiledPID.kV);

    public Elevator(Trigger firstBeamBrake, Trigger SecondBeamBreak) {
        leftSparkMax = new SparkMax(ElevatorConstants.leftSparkMaxID, MotorType.kBrushless);
        rightSparkMax = new SparkMax(ElevatorConstants.rightSparkMaxID, MotorType.kBrushless);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        leftConfig.inverted(ElevatorConstants.leftSparkMaxInverted);
        rightConfig.inverted(ElevatorConstants.rightSparkMaxInverted);

        leftConfig.smartCurrentLimit(40);
        rightConfig.smartCurrentLimit(40);

        leftSparkMax.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightSparkMax.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = leftSparkMax.getEncoder();

        bottomSwitch = new DigitalInput(ElevatorConstants.bottomSwitchID);

        m_controller.setTolerance(ElevatorConstants.atTargetOffset);
        m_controller.setIZone(5);

        m_firstBeamBrake = firstBeamBrake;
        m_secondBeamBrake = SecondBeamBreak;

        setState(ElevatorState.SOURCE);

    }

    // pidloop
    private double encoderPosition;
    private double goal;

    @Override
    public void periodic() {
        loggers();
        double output = m_controller.calculate(m_encoder.getPosition())
        + m_feedforward.calculate(m_controller.getSetpoint().velocity);
        setOutput(output);
    }

    public void pidControl() {
        double output = m_controller.calculate(m_encoder.getPosition())
        + m_feedforward.calculate(m_controller.getSetpoint().velocity);
        setOutput(output);
    }

    // elevator states
    public enum ElevatorState {
        SOURCE("SOURCE", ElevatorConstants.SetpointRotations.SOURCE, 0),
        L1("L1", ElevatorConstants.SetpointRotations.L1, 1),
        L2("L2", ElevatorConstants.SetpointRotations.L2, 2),
        L3("L3", ElevatorConstants.SetpointRotations.L3, 3),
        L4("L4", ElevatorConstants.SetpointRotations.L4, 4);

        double rotations;
        String name;
        int index;

        ElevatorState(String name, double rotations, int index) {
            this.rotations = rotations;
            this.name = name;
            this.index = index;
        }

        public static ElevatorState findByIndex(int index) {
            ElevatorState[] values = ElevatorState.values();
            for (int i = 0; i < values.length; i++) {
                if (values[i].index == index) {
                    return values[i];
                }
            }
            throw new Error();
        }
    }

    /* FUNCTIONS */

    // set motor outputs
    public void setOutput(double output) {
        rightSparkMax.set(output);
        leftSparkMax.set(output);
    }

    // set pid goal
    public void setGoal(double newgoal) {
        if (!m_firstBeamBrake.getAsBoolean()) {
            goal = newgoal;
        }
    }

    // manual control that's iffy
    public void manualControl(double input) {
        if (input > 0 && goal + input <= ElevatorConstants.kMaxElevatorHeightRotations) {
            setGoal(goal + input);
        } else if (input < 0 && goal + input >= ElevatorConstants.kMinElevatorHeightRotations) {
            if (goal > 0 && goal < 1) {
                setGoal(0);
            } else {
                setGoal(goal + input);
            }

        }
    }

    // encoder stuff
    public void setEncoderPosition(double position) {
        m_encoder.setPosition(position);
    }

    public double getEncoderPosition() {
        if (getBottomSwitch()) {
            setEncoderPosition(0);
        }
        return m_encoder.getPosition();
    }

    // limit switch
    public boolean getBottomSwitch() {
        return !bottomSwitch.get();
    }

    // State machine stuff
    public ElevatorState getElevatorState() {
        return elevatorState;
    }

    public int getElevatorStateIndex() {
        return elevatorState.index;
    }

    public ElevatorState findNearestState() {
        double position = getEncoderPosition();
        double diff1;
        double currentClosestDiff = 0;
        ElevatorState closestState = ElevatorState.SOURCE;

        for (int i = 0; i <= ElevatorState.values().length; i++) {
            diff1 = Math.abs(position - ElevatorState.findByIndex(i).rotations);

            if (diff1 < currentClosestDiff) {
                currentClosestDiff = diff1;

                closestState = ElevatorState.findByIndex(i);
            }

        }
        return closestState;
    }

    public void setState(ElevatorState state) {
        elevatorState = state;
        setGoal(elevatorState.rotations);
    }

    public void setStateByIndex(int desiredIndex) {
        elevatorState = ElevatorState.findByIndex(desiredIndex);
        setGoal(elevatorState.rotations);
    }

    public void moveStateUp() {
        int state = elevatorState.index;

        if (state < 4) {
            state += 1;
        }

        elevatorState = ElevatorState.findByIndex(state);
        setGoal(elevatorState.rotations);
    }

    public void moveStateDown() {
        int state = elevatorState.index;

        if (state > 0) {
            state -= 1;
        }

        elevatorState = ElevatorState.findByIndex(state);
        setGoal(elevatorState.rotations);
    }

    // for automatic subsystems
    public boolean elevatorAtTarget() throws unfilledConstant {
        double offset = ElevatorConstants.atTargetOffset;
        if (offset == 0) {
            throw new unfilledConstant(
                    "The atTargetOffset elevator constants is set to zero meaning nothing will work ever");
        }
        if (encoderPosition > goal - offset && encoderPosition < goal + offset) {
            return true;
        }
        return false;
    }

    /* LOGGERS */

    private void loggers() {
        SmartDashboard.putNumber("elevator encoder position", getEncoderPosition());
        SmartDashboard.putNumber("elevator pid goal", goal);
        SmartDashboard.putString("elevatorstate", getElevatorState().toString());
        // SmartDashboard.putNumber("elevator kp", ElevatorConstants.PID.kP);
        // SmartDashboard.putNumber("elevator ki", ElevatorConstants.PID.kI);
        // SmartDashboard.putNumber("elevator kd", ElevatorConstants.PID.kD);
        SmartDashboard.putBoolean("elevator bottom switch", getBottomSwitch());
        SmartDashboard.putBoolean("pid at goal", m_controller.atSetpoint());
    }

    /* RUNNABLE ACTIONS FOR BUTTON BOX */

    public Command moveMotorsNoPID(double output) {
        return this.runOnce(() -> setOutput(output));
    }

    public Command goToStateCommand(ElevatorState state) {
        return this.runOnce(() -> setState(state));
    }

    public Command stateUp() {
        return this.runOnce(() -> moveStateUp());
    }

    public Command stateDown() {
        return this.runOnce(() -> moveStateDown());
    }

    public Command goToPosition(double position) {
        return this.runOnce(() -> setGoal(position));
    }

    public Command manualUp() {
        return this.run(() -> manualControl(1));
    }

    public Command manualDown() {
        return this.run(() -> manualControl(-1));
    }

    public Command StopManual() {
        return this.runOnce(() -> manualControl(0));
    }

}
