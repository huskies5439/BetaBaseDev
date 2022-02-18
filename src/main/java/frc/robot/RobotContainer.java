// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of basePilotable project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auto3Ballons;
import frc.robot.commands.CaracteriserDrive;
import frc.robot.commands.TournerAuto;
import frc.robot.commands.TrajetAuto;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Pince;


/**
 * basePilotable class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  BasePilotable basePilotable = new BasePilotable();
  Pince pnice = new Pince();

  XboxController joystick = new XboxController(0);

  private final SendableChooser <Command> chooser = new SendableChooser<>();
 
  private final Command retourne = new TrajetAuto("retourne", basePilotable);
  private final Command Auto2Ballons = new Auto3Ballons(basePilotable, pnice);
  private final Command trajetVide = new WaitCommand(14);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
   chooser.addOption("retourne", retourne);
  
    chooser.setDefaultOption("Trajet Vide", trajetVide);
    chooser.addOption("2Ballons", Auto2Ballons);


    SmartDashboard.putData(chooser);

basePilotable.setDefaultCommand(new RunCommand(() -> basePilotable.conduire(joystick.getLeftY(), joystick.getRightX()), basePilotable));

    
  }

  /**
   * Use basePilotable method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick, Button.kA.value).whenPressed(new InstantCommand(pnice::ouvrirPince));
    new JoystickButton(joystick, Button.kB.value).whenPressed(new InstantCommand(pnice::fermerPince));

  }

  /**
   * Use basePilotable to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    //return new TournerAuto(-180, basePilotable).andThen(new WaitCommand(1)).andThen(new TournerAuto(90, basePilotable)).andThen(new WaitCommand(1)).andThen(new TournerAuto(270, basePilotable));

    return chooser.getSelected();


}
  }
