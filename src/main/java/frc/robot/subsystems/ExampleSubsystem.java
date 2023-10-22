// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon; 


public class ExampleSubsystem extends SubsystemBase {

  //motors
  private final WPI_TalonSRX motorTop; 
  private final WPI_TalonSRX motorBottom;
  private final CANSparkMax flyWheel;

  //sensors
  private final AnalogPotentiometer irBottom; //ir camera on the side
  private final AnalogPotentiometer irTop; //ir camera on the side
  private final ColorSensorV3 colorSensor; //color sensor
  
  //boleans
  private boolean spaceAtTop; //chekcs if there is a ball at top
  private boolean toLaunch; //launching sequence once controller button pressed

  //controllers
  private final XboxController controller; 

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    //motors
    motorTop = new WPI_TalonSRX(14);
    motorBottom = new WPI_TalonSRX(13);  
    flyWheel = new CANSparkMax(15, MotorType.kBrushed);
    motorTop.configFactoryDefault();
    motorBottom.configFactoryDefault();
    
    //sensors
    irBottom = new AnalogPotentiometer(0); //IR sensor
    irTop = new AnalogPotentiometer(1);
    colorSensor = new ColorSensorV3(I2C.Port.kMXP); //color sensor

    //controller and boolean
    controller = new XboxController(0);
    spaceAtTop = true;
    toLaunch = false;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    System.out.println(irBottom.get());
    //System.out.println("Hello");
    double bottomBallTolerance = 0.08; //ideally 0.16
    double topBallTolerance = 0.4; //further away means smaller
    int middleBallTolerance = 9;  //add half of ball width, 1695-99(without)

    //checks if there is space above for the balls to travel up
    if(irTop.get() > topBallTolerance){
      spaceAtTop = false;
      motorBottom.set(0);
      motorTop.set(0);
      flyWheel.set(0);
    }

    //checks if there is a ball in 'waiting area' and if there is space for a ball to be rolled in
    if(spaceAtTop && irBottom.get() >= bottomBallTolerance){
        motorBottom.set(-0.2); //TEST SPEED

        //checks if there is a ball that needs to move up before another can be rolled in
        if(colorSensor.getProximity() > middleBallTolerance){
          motorTop.set(-0.1);
        }
    } else { //resets 
      motorBottom.set(0);
      motorTop.set(0);
    }

    //This method will be called once per scheduler run

    if(controller.getAButtonPressed()){
      toLaunch = true;
    }

    if(toLaunch){
      if(irTop.get() < topBallTolerance){
        motorTop.set(-0.3);
      } else {
        flyWheel.set(0.3);
        motorTop.set(0);
        toLaunch = false;
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    
    // This method will be called once per scheduler run during simulation
  }
}
