// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.Command; 
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsytem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkMax motor;
  
  private final SparkMaxConfig motorConfig;

	public Command out() {
		
    return Commands.runOnce(() -> spinOut());// calls the spinOut() method
    
     // 'Commands.runOnce()' means "create a command that will do something one time and then finish."
     // '() -> spinOut()' is a short way of saying "when this command runs, call the 'spinOut()' method."
	}

  public Command in(){
    return Commands.runOnce(() -> spinIn()); // calls the spinIn() method
  }
  
  public Command stopCommand(){ 
    return Commands.runOnce(() -> stop()); // Calls stop() method
}
  
  public ShooterSubsytem() {
    
    motor = new SparkMax(25, MotorType.kBrushed);
    motorConfig = new SparkMaxConfig();
    motorConfig.smartCurrentLimit(40);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void spinOut(){
    motor.set(0.5);
  //makes motor spin outward
  }
  // makes shooter spin in
  
  public void spinIn(){
    motor.set(-0.5);
  
  //-1 makes motor spin inwards
  }
 
  public void stop(){
    motor.set(0);
  } 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


