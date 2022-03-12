package frc.robot.commands.auto;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TournerLimelight;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pince;

public class Auto3Ballons extends SequentialCommandGroup {

    public Auto3Ballons(BasePilotable basePilotable, Pince pince, Limelight limelight) {
    Trajectory trajet = basePilotable.creerTrajectoire("3ballons");
    addCommands(
      //Lancer
       //new TournerLimelight(basePilotable, limelight),
       new WaitCommand(1),
       new InstantCommand(() -> pince.fermerPince()),
       new WaitCommand(1),
       new InstantCommand(() -> pince.ouvrirPince()),
    //initialisation
    new InstantCommand(() -> basePilotable.resetOdometry(trajet.getInitialPose())),
    new InstantCommand(() -> basePilotable.setRamp(0)),
    new InstantCommand(() -> basePilotable.setBrake(true)),
    //trajet
    basePilotable.ramseteSimple(trajet),//parallèle avec gobeur

    //Lancer
    new TournerLimelight(basePilotable, limelight),
    new WaitCommand(1),
    new InstantCommand(() -> pince.fermerPince()),
    new WaitCommand(1),
    new InstantCommand(() -> pince.ouvrirPince())
    );
  }
}

//supprimer cette commande et remplacer par V2