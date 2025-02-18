// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drivebase.CenterOnAprilTag;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Arm;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.SwerveInputStream;

public class RobotContainer {
   // The robot's subsystems and commands are defined here...
   private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
   private final Arm m_arm = new Arm();
   // Replace with CommandPS4Controller or CommandJoystick if needed
   private final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.XBOX_DRIVER_PORT);
   private final CommandXboxController operatorXbox = new CommandXboxController(OperatorConstants.XBOX_OPERATOR_PORT);
   private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
         () -> driverXbox.getLeftY() *-1.0,
         () -> driverXbox.getLeftX() *-1.0)
         .withControllerRotationAxis(()->driverXbox.getRightX() * -1.0)
         .deadband(OperatorConstants.DEADBAND)
         .scaleTranslation(0.8)
         .allianceRelativeControl(true);

   /**
    * Clone's the angular velocity input stream and converts it to a fieldRelative
    * input stream.
    */
   SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
         driverXbox::getRightY)
         .headingWhile(true);

   public RobotContainer() {
      // Configure the trigger bindings
      configureBindings();
      DriverStation.silenceJoystickConnectionWarning(true);
      NamedCommands.registerCommand("test", Commands.print("I EXIST"));
   }

   private void configureBindings()
   {
      SmartDashboard.putData("Clear CCd Faults", m_arm.clearCancoderFaultsCmd());

      SmartDashboard.putData("Align with Tag Cmd", new CenterOnAprilTag(drivebase));
      Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
      Command elevNeutralCmd = m_arm.setElevPositionCmd(ArmConstants.ELEVATOR_POS_NEUTRAL);
      
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      m_arm.setDefaultCommand(m_arm.armStopCmd());

      // DRIVER CONTROLS   
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      

      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());

      /*
       operatorXbox.axisGreaterThan(
                                    XboxController.Axis.kLeftY.value, 0.1).or(
                                    operatorXbox.axisLessThan(XboxController.Axis.kLeftY.value, -0.1)).whileTrue(
                                       m_shooter.armByXboxCommand(() -> -operatorXbox.getLeftY()));

       */
      // OPERATOR CONTROLS
      operatorXbox.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1)
         .or(operatorXbox.axisLessThan(XboxController.Axis.kLeftY.value, -0.1))
         .whileTrue(m_arm.armByXboxCmd(()->operatorXbox.getLeftY()*-1, ()->operatorXbox.getRightY()*-1.0));

      operatorXbox.axisGreaterThan(XboxController.Axis.kRightY.value, 0.1)
      .or(operatorXbox.axisLessThan(XboxController.Axis.kRightY.value, -0.1))
         .whileTrue(m_arm.armByXboxCmd(()->operatorXbox.getLeftY()*-1, ()->operatorXbox.getRightY()*-1.0));

      operatorXbox.b().onTrue(m_arm.setElevPositionCmd(10.0));
      operatorXbox.a().onTrue(m_arm.setElevPositionCmd(1.0));
      //driverXbox.leftTrigger(0.01).whileTrue(m_arm.armByXboxCmd(()->driverXbox.getLeftTriggerAxis()))
      //.onFalse(m_arm.armStopElevator());
      //driverXbox.rightTrigger(0.01).whileTrue(m_arm.armByXboxCmd(()->driverXbox.getRightTriggerAxis()*-1.0))
      //.onFalse(m_arm.armStopElevator());

   
   }

   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
   public Command getAutonomousCommand() {
      // An example command will be run in autonomous
      return Autos.exampleAuto(m_exampleSubsystem);
   }

   public void setMotorBrake(boolean brake) {
      drivebase.setMotorBrake(brake);
   }
}
