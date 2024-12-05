// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
   private static final String kDefaultAuto = "Default";
   private static final String kCustomAuto = "My Auto";
   private String m_autoSelected;
   private final SendableChooser<String> m_chooser = new SendableChooser<>();

   private static AnalogEncoder steerEncoder1 = new AnalogEncoder(0);
   private static AnalogEncoder steerEncoder2 = new AnalogEncoder(1);
   private static AnalogEncoder steerEncoder3 = new AnalogEncoder(2);
   private static AnalogEncoder steerEncoder4 = new AnalogEncoder(3);

   private static TalonFX driveKraken1 = new TalonFX(15);

   private static XboxController xbox1 = new XboxController(0);

   private static Pigeon2 pidgey = new Pigeon2(20);

   /**
    * This function is run when the robot is first started up and should be used
    * for any
    * initialization code.
    */
   @Override
   public void robotInit() {
      m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
      m_chooser.addOption("My Auto", kCustomAuto);
      SmartDashboard.putData("Auto choices", m_chooser);
   }

   /**
    * This function is called every 20 ms, no matter the mode. Use this for items
    * like diagnostics
    * that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>
    * This runs after the mode specific periodic functions, but before LiveWindow
    * and
    * SmartDashboard integrated updating.
    */
   @Override
   public void robotPeriodic() {
   }

   /**
    * This autonomous (along with the chooser code above) shows how to select
    * between different
    * autonomous modes using the dashboard. The sendable chooser code works with
    * the Java
    * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
    * chooser code and
    * uncomment the getString line to get the auto name from the text box below the
    * Gyro
    *
    * <p>
    * You can add additional auto modes by adding additional comparisons to the
    * switch structure
    * below with additional strings. If using the SendableChooser make sure to add
    * them to the
    * chooser code above as well.
    */
   @Override
   public void autonomousInit() {
      m_autoSelected = m_chooser.getSelected();
      // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
      System.out.println("Auto selected: " + m_autoSelected);
   }

   /** This function is called periodically during autonomous. */
   @Override
   public void autonomousPeriodic() {
      switch (m_autoSelected) {
         case kCustomAuto:
            // Put custom auto code here
            break;
         case kDefaultAuto:
         default:
            // Put default auto code here
            break;
      }
   }

   /** This function is called once when teleop is enabled. */
   @Override
   public void teleopInit() {
      driveKraken1.stopMotor();
   }

   /** This function is called periodically during operator control. */
   @Override
   public void teleopPeriodic()
   {
      SmartDashboard.putNumber("Enc1 Abs", steerEncoder1.getAbsolutePosition());
      SmartDashboard.putNumber("Enc1 Raw", steerEncoder1.get());

      SmartDashboard.putNumber("Enc2 Abs", steerEncoder2.getAbsolutePosition());
      SmartDashboard.putNumber("Enc2 Raw", steerEncoder2.get());

      SmartDashboard.putNumber("Enc3 Abs", steerEncoder3.getAbsolutePosition());
      SmartDashboard.putNumber("Enc3 Raw", steerEncoder3.get());

      SmartDashboard.putNumber("Enc4 Abs", steerEncoder4.getAbsolutePosition());
      SmartDashboard.putNumber("Enc4 Raw", steerEncoder4.get());

      SmartDashboard.putNumber("Pidgey Heading", pidgey.getAngle());

      
      double joyLY = xbox1.getLeftY();
      if (joyLY > -0.09 && joyLY < 0.09 )
         joyLY = 0.0;

      SmartDashboard.putNumber("XboxLEftY", joyLY);

      driveKraken1.set(joyLY);
   }

   /** This function is called once when the robot is disabled. */
   @Override
   public void disabledInit() {
      driveKraken1.stopMotor();
   }

   /** This function is called periodically when disabled. */
   @Override
   public void disabledPeriodic() {
   }

   /** This function is called once when test mode is enabled. */
   @Override
   public void testInit() {
   }

   /** This function is called periodically during test mode. */
   @Override
   public void testPeriodic() 
   {

      
   }

   /** This function is called once when the robot is first started up. */
   @Override
   public void simulationInit() {
   }

   /** This function is called periodically whilst in simulation. */
   @Override
   public void simulationPeriodic() {
   }
}
