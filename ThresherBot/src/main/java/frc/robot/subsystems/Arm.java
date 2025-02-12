// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase
{
   // ELEVATOR
   private final SparkMax m_elevatorMaster = new SparkMax(Constants.ELEVATOR_MASTER_CAN_ID, MotorType.kBrushless);
   private final SparkMax m_elevatorSlave = new SparkMax(Constants.ELEVATOR_SLAVE_CAN_ID, MotorType.kBrushless);
   private final SparkClosedLoopController m_elevatorPID = m_elevatorMaster.getClosedLoopController();
   private final RelativeEncoder m_elevatorEncoder = m_elevatorMaster.getEncoder();
   // WRIST
   private final TalonFX m_wrist = new TalonFX(Constants.ARM_WRIST_CAN_ID);
   private final CANcoder m_wristEncoder = new CANcoder(Constants.ARM_ENCODER_CAN_ID); 
   private final PositionVoltage m_wristVoltage = new PositionVoltage(0);
   //private final DutyCycleEncoder m_wristRevEnc = new DutyCycleEncoder(0);
   //private final LaserCan m_laser = new LaserCan(Constants.LASER_CAN_A_ID);

   private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
   private GenericEntry dbLaserEntry = tab.add("Laser Distance", 0).getEntry();
   private GenericEntry dbWristAngle = tab.add("Wrist Angle", 0).getEntry();
   private GenericEntry dbWristSetpoint = tab.add("Wrist Set", 0).getEntry();
   private GenericEntry dbWristCurrent = tab.add("Wrist Amps", 0).getEntry();
   private GenericEntry dbWristMotorV = tab.add("Wrist Volts", 0).getEntry();
   
   private GenericEntry dbElvOutput = tab.add("Elv Output", 0).getEntry();
   private GenericEntry dbElvEncRaw = tab.add("Elv Enc Raw", 0).getEntry();
   private GenericEntry dbElvEncVel = tab.add("Elv Enc Vel", 0).getEntry();
   private GenericEntry dbElvSetpoint = tab.add("Elv Setpoint", 0).getEntry();



   private final StatusSignal<Boolean> f_fusedSensorOutOfSync = m_wrist.getFault_FusedSensorOutOfSync(false);
   private final StatusSignal<Boolean> sf_fusedSensorOutOfSync = m_wrist.getStickyFault_FusedSensorOutOfSync(false);
   private final StatusSignal<Boolean> f_remoteSensorInvalid = m_wrist.getFault_RemoteSensorDataInvalid(false);
   private final StatusSignal<Boolean> sf_remoteSensorInvalid = m_wrist.getStickyFault_RemoteSensorDataInvalid(false);

   private final StatusSignal<Angle> fx_pos = m_wrist.getPosition(false);
   private final StatusSignal<AngularVelocity> fx_vel = m_wrist.getVelocity(false);
   private final StatusSignal<Angle> cc_pos = m_wristEncoder.getPosition(false);
   private final StatusSignal<AngularVelocity> cc_vel = m_wristEncoder.getVelocity(false);
   private final StatusSignal<Angle> fx_rotorPos = m_wrist.getRotorPosition(false);

   private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0);

   private int printCount = 0;

   private double m_wristSetpoint = 0.0;
   private double m_elevatorSetpoint = 0.0;
   private boolean m_elevatorUsePid = false;

   public Arm()
   {
      ConfigureWrist();
      ConfigureElevator();

   }
   // GUIDES: https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/closed-loop-guide.html
   // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/PositionClosedLoop/src/main/java/frc/robot/Robot.java

   @Override
   public void periodic()
   {
      // This method will be called once per scheduler run
      //ExecuteWristPIDPeriodic();
   
      UpdateSmartDashboard();

   }

   public void setElevatorPosition(double position)
   {
      System.out.println("Set Elevator Position: " + position);
      double currentPos = m_elevatorEncoder.getPosition();
      m_elevatorSetpoint = position;
      // TODO: some kind of offsets here possible?

      // TODO: if you need to apply arbFF do it here, not for elevator i think
      m_elevatorPID.setReference(m_elevatorSetpoint, ControlType.kPosition);
   }

   private void moveElevatorOpenLoop(double speed)
   {
      m_elevatorMaster.set(speed);
   }

   
   public Command clearCancoderFaultsCmd()
   {
      return run( ()-> clearCancoderFaults())
               .withName("ClearFaults");
   }


   public Command armByXboxCmd(DoubleSupplier elevValue)
   {
      return run( ()-> moveElevatorOpenLoop(elevValue.getAsDouble()))
               .withName("ElevByXbox");
   }

   public Command armStopElevator()
   {
      return run( ()-> moveElevatorOpenLoop(0))
               .withName("ElevStop");
   }

   public Command setElevPositionCmd(double setpoint)
   {
      return new InstantCommand(()->setElevatorPosition(setpoint)).withName("ElevSetPos");
   }

   public void ConfigureElevator()
   {
      SparkMaxConfig masterConfig = new SparkMaxConfig();
      SparkMaxConfig slaveConfig = new SparkMaxConfig();

      // TODO; do we setup PID here? Need to check REV lib
      masterConfig.closedLoop.pidf(  ArmConstants.ELEVATOR_PID_P,
                                     ArmConstants.ELEVATOR_PID_I,
                                     ArmConstants.ELEVATOR_PID_D,
                                     ArmConstants.ELEVATOR_PID_FF);
      m_elevatorMaster.configure(masterConfig, SparkBase.ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

      // Slave is just set to follow
      slaveConfig.follow(Constants.ELEVATOR_MASTER_CAN_ID, true);
      m_elevatorSlave.configure(slaveConfig,SparkBase.ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);  

   }

   public void ConfigureWrist()
   {
      CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
      //cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
      cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      cc_cfg.MagnetSensor.withMagnetOffset(Rotations.of(0.0));
      m_wristEncoder.getConfigurator().apply(cc_cfg);

      TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
      fx_cfg.Feedback.FeedbackRemoteSensorID = m_wristEncoder.getDeviceID();
      fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
      fx_cfg.Feedback.RotorToSensorRatio = 25.0;

      fx_cfg.Slot0.kP = ArmConstants.WRIST_PID_P;
      fx_cfg.Slot0.kI = ArmConstants.WRIST_PID_I;
      fx_cfg.Slot0.kD = ArmConstants.WRIST_PID_D;

      fx_cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
         .withPeakReverseVoltage(Volts.of(-2));


      StatusCode status = m_wrist.getConfigurator().apply(fx_cfg);

      if (!status.isOK())
      {
         System.out.println("Wrist configuration failed: " + status);
      }

      m_wrist.setPosition(0);
   }

   public void ExecuteWristPIDPeriodic()
   {
      if (++printCount >= 10)
      {
         printCount = 0;

         BaseStatusSignal.refreshAll(
         f_fusedSensorOutOfSync,
         sf_fusedSensorOutOfSync,
         f_remoteSensorInvalid,
         sf_remoteSensorInvalid,
         fx_pos, fx_vel,
         cc_pos, cc_vel
         );

         // If any faults happen, print them out. Sticky faults will always be present if live-fault occurs
         boolean anyFault = sf_fusedSensorOutOfSync.getValue() || sf_remoteSensorInvalid.getValue();
         SmartDashboard.putBoolean("CANCODER FAULT?", anyFault);
         if (anyFault)
         {
            System.out.println("A fault has occurred:");
            /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
            if (f_fusedSensorOutOfSync.getValue())
            {
               System.out.println("Fused sensor out of sync live-faulted");
            } 
            else if (sf_fusedSensorOutOfSync.getValue())
            {
               System.out.println("Fused sensor out of sync sticky-faulted");
            }
            /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
            if (f_remoteSensorInvalid.getValue())
            {
               System.out.println("Missing remote sensor live-faulted");
            } 
            else if (sf_remoteSensorInvalid.getValue())
            {
               System.out.println("Missing remote sensor sticky-faulted");
            }
         }

         //if (m_joystick.getAButton()) {
         /* Clear sticky faults */
         //  m_fx.clearStickyFaults();
         //}

         /* Print out current position and velocity */
         //System.out.println("FX Position: " + fx_pos + " FX Vel: " + fx_vel);
         //System.out.println("CC Position: " + cc_pos + " CC Vel: " + cc_vel);
         System.out.println("");
      }
       
      //m_mechanism.update(fx_rotorPos, cc_pos, fx_pos);
   }

   private void clearCancoderFaults()
   {
      m_wristEncoder.clearStickyFaults();
   }

   private void UpdateSmartDashboard()
   {
      dbLaserEntry.setDouble(0);
      dbWristAngle.setDouble(m_wristEncoder.getPosition().getValueAsDouble());
      dbWristSetpoint.setDouble(m_wristSetpoint);
      dbWristCurrent.setDouble(m_wrist.getTorqueCurrent().getValueAsDouble());
      dbWristMotorV.setDouble(m_wrist.getMotorVoltage().getValueAsDouble());

      dbElvOutput.setDouble(m_elevatorMaster.getAppliedOutput());
      dbElvEncRaw.setDouble(m_elevatorEncoder.getPosition());
      dbElvEncVel.setDouble(m_elevatorEncoder.getVelocity());
      dbElvSetpoint.setDouble(m_elevatorSetpoint);

      //sbtShooterSetpoint.setDouble(m_shooterSetpoint);
      //sbtArmSpeed.setDouble((m_armMotor.get()));
      //sbtArmPosition.setDouble(m_armMotor.getSelectedSensorPosition());
      //stbArmSetpoint.setDouble(m_armSetpoint);
      //stbArmError.setDouble(m_armMotor.getClosedLoopError());
   }

}
