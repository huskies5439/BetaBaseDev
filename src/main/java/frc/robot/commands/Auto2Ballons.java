// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Pince;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2Ballons extends SequentialCommandGroup {
  /** Creates a new Auto2Ballons. */

  public Auto2Ballons(BasePilotable basePilotable, Pince pince) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Trajectory one = basePilotable.creerTrajectoire("2bV2-1");
    Trajectory two = basePilotable.creerTrajectoire("2bV2-2");
    addCommands(
      //initialisation
      new InstantCommand(() -> basePilotable.resetOdometry(one.getInitialPose())),
      new InstantCommand(() -> basePilotable.setRamp(0)),
      new InstantCommand(() -> basePilotable.setBrake(true)),
      //trajet
      basePilotable.ramseteSimple(one),
      basePilotable.ramseteSimple(two)

    );
  }
}