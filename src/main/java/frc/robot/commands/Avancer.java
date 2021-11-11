// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

 
public class Avancer extends CommandBase {
  boolean stop;
  double distance;
  double marge;

  BasePilotable basePilotable;

  public Avancer(double distance,BasePilotable basePilotable) {
  
    stop = false;
    marge = 0.01;
    this.basePilotable = basePilotable;
    this.distance = distance;

    addRequirements(basePilotable);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    basePilotable.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (basePilotable.getPosition() > distance + marge) {

      basePilotable.autoConduire(-1, 0);
    }

    else if (basePilotable.getPosition() < distance - marge) {

      basePilotable.autoConduire(1, 0);  
    }

    else {

      basePilotable.stop();  
      stop = true;
      // Avancer le robot jusqu'à ce que les roues est fait la distance en tour de roue et le respecte dans la marge 
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
    
  }
}
