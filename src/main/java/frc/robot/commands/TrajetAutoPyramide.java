// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BasePilotable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajetAutoPyramide extends SequentialCommandGroup {
  int side;

  /** Creates a new TrajetAutoJaunePyramide. */
  public TrajetAutoPyramide(int side, BasePilotable basePilotable) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    //Brake on ramp = 0.1
    new Avancer(0.25, 0.2, basePilotable), // Monter le bras 

    new WaitCommand(0.5), // Remplacer par attraper le tube

    new Tourner (-100*side, basePilotable),

    new Avancer(2.25, 0.4, basePilotable),

    new Tourner(-45*side, basePilotable),

    new Avancer( 0.95, 0.4, basePilotable)

    // Coder le retour pour chercher l'autre cylindre

    );
  }
}
