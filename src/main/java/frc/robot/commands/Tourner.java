// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

public class Tourner extends CommandBase {
  boolean stop;
  double angle;
  //double marge;
  BasePilotable basePilotable;

  /** Creates a new Tourner. */
  public Tourner(double angle,BasePilotable basePilotable) {
    stop = false;
    this.basePilotable = basePilotable;
    this.angle = angle;
    //marge = 0.01
    
    addRequirements(basePilotable);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(basePilotable.getAngle()<angle-1 /*marge*/){
      basePilotable.autoConduire(0,0.2);

    }
    else if(basePilotable.getAngle()>angle+1 /*marge*/){
      basePilotable.autoConduire(0,-0.2);
    }
  else{
    basePilotable.autoConduire(0, 0);
    stop = true;//#endregion.
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
