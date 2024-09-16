// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TejuinoBoard;
import java.lang.*;
import edu.wpi.first.wpilibj.GenericHID;
import java.util.TimeZone;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  int numeroleds = 16;
  double potenciaChassis;
  boolean avanzar;
  private final VictorSP motorcito = new VictorSP(2);
  private final VictorSP m_leftDrive = new VictorSP(0);
  private final VictorSP m_rightDrive = new VictorSP(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
private final GenericHID control = new GenericHID(1);
  private final PS4Controller m_controller = new PS4Controller(0);
  
  private final Timer m_timer = new Timer();
  
  private final TejuinoBoard tejuino = new TejuinoBoard();
  private final Encoder m_encoder = new Encoder(0, 1, false, CounterBase.EncodingType.k4X);
  private final AnalogInput supersonic = new AnalogInput(1);



  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);

    tejuino.init(1);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   * 
   */

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead

    SmartDashboard.getNumber("potencia chassis", 0.5);
    m_rightDrive.setInverted(true);
    m_encoder.setSamplesToAverage(5);

    m_encoder.setDistancePerPulse(1.0 / 360.0 *Math.PI * 6);

    m_encoder.setMinRate(1.0);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    tejuino.turn_off_all_leds(1);
    tejuino.turn_off_all_leds(0);
CameraServer.startAutomaticCapture();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
   m_timer.start();
   

   SmartDashboard.putNumber("Encoder Distance inches", Math.round(m_encoder.getDistance()*100)/100d);
   SmartDashboard.putNumber("Encoder Distance CM", Math.round(2.54 * m_encoder.getDistance()*100)/100d);
   SmartDashboard.putNumber("Encoder Rate", m_encoder.getRate());
   SmartDashboard.putNumber("Encoder raw", m_encoder.getRaw());
   SmartDashboard.putNumber("Distancia en cm", ((supersonic.getVoltage()/1024)*5));
//this section sets up the axis of a cim motor according to
//the sensibility of a  controller trigger

   double potenciamotorcitopositivo = m_controller.getR2Axis();
    double potenciamotorcitonegativo = m_controller.getL2Axis();
    double operacionpositiva = ((potenciamotorcitopositivo + 1) / 2);
    double operacionnegativa = (((potenciamotorcitonegativo + 1) / 2) * -1);
    potenciaChassis = SmartDashboard.getNumber("potencia chassis", potenciaChassis);


    m_robotDrive.arcadeDrive(-m_controller.getLeftY() * potenciaChassis, -m_controller.getRightX() * potenciaChassis);
    if (-m_controller.getLeftY() >= 0.1) {

      avanzar = true;
    } else {
      avanzar = false;
    }
    
    SmartDashboard.putBoolean("adelante", avanzar);
    SmartDashboard.putNumber("potencia chassis", potenciaChassis);
// this section activates the movement of the motor
    
    if (m_controller.getR2Button()) {
      motorcito.set(operacionpositiva);
      System.out.println(operacionpositiva);
    } else {
      if (m_controller.getL2Button()) {
        motorcito.set(operacionnegativa);
      } else {
        motorcito.set(0);
      }
    }
    if (m_controller.getL2Button() && m_controller.getR2Button()) {

      motorcito.stopMotor();
    }


//this section activates the tejuino board, a 
// single led control a second, or the amount of time
// that you want, by multiplying "tiempoentero"

double tiempo = m_timer.get();
int tiempoentero = (int)tiempo;


if(true){
tejuino.single_led_control(0, tiempoentero, 255, 2, 1);
}

tejuino.all_leds_blue(1);







 }

/*if (numeroleds>tiempoentero && i< tiempoentero){
tejuino.single_led_control(0, tiempoentero, 255, 0, 0); 

System.out.println("Hola, soy un led");
i++;

}
 System.out.println(tiempoentero);
System.out.println(i);

 * 
 * 
 */





  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

}