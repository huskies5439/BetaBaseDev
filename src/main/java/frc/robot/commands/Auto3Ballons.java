package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Pince;

public class Auto3Ballons extends SequentialCommandGroup {

    public Auto3Ballons(BasePilotable basePilotable, Pince pince) {
    Trajectory one = basePilotable.creerTrajectoire("3b-1");
    addCommands(
    //initialisation
    new InstantCommand(() -> basePilotable.resetOdometry(one.getInitialPose())),
    new InstantCommand(() -> basePilotable.setRamp(0)),
    new InstantCommand(() -> basePilotable.setBrake(true)),
    //trajet
    basePilotable.ramseteSimple(one)

    );
  }
}

//supprimer cette commande et remplacer par V2