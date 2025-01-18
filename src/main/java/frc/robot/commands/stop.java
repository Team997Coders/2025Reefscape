package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class stop extends Command {
  
  private Command command;

  public stop(Command command) {
    this.command = command;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    command.cancel();
    this.cancel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
