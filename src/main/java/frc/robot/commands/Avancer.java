// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

 
public class Avancer extends CommandBase {
  boolean stop;
  double distance;
  double vitesse;
  double marge;
  double ajustementRotation;
  double angleDirection;

  BasePilotable basePilotable;

  public Avancer(double distance, double vitesse, BasePilotable basePilotable) {
  
    stop = false;
    marge = 0.01;
    this.basePilotable = basePilotable;
    this.distance = distance;
    this.vitesse = vitesse;

    addRequirements(basePilotable);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    basePilotable.resetEncoder();
    angleDirection = basePilotable.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ajustementRotation = (angleDirection-basePilotable.getAngle()) * 0.05
     /* à calibrer */; 


    if (basePilotable.getPosition() > distance + marge) {

      basePilotable.autoConduire(-vitesse, ajustementRotation);
    }

    else if (basePilotable.getPosition() < distance - marge) {

      basePilotable.autoConduire(vitesse, ajustementRotation);  
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
