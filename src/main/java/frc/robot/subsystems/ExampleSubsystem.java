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
  private final CANSparkMax flyWheelMotor;

  //sensors
  private final AnalogPotentiometer irBottom; //ir camera on the bottom
  private final AnalogPotentiometer irTop; //ir camera on the top
  private final ColorSensorV3 colorSensor; //color sensor
  
  //boleans
  private boolean spaceAtTop; //checks if there is a ball at very top of robot
  private boolean toLaunch; //launching sequence once controller button pressed

  //controllers
  private final XboxController controller; 

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    //initalizing motors
    motorTop = new WPI_TalonSRX(14);
    motorBottom = new WPI_TalonSRX(13);  
    flyWheelMotor = new CANSparkMax(15, MotorType.kBrushed);

    //configs
    motorTop.configFactoryDefault();
    motorBottom.configFactoryDefault();
    
    //initalizing sensors
    irBottom = new AnalogPotentiometer(0); //IR sensor
    irTop = new AnalogPotentiometer(1);
    colorSensor = new ColorSensorV3(I2C.Port.kMXP); //color sensor

    //initializing controller and boolean
    controller = new XboxController(0);
    spaceAtTop = true; //with no balls at top of robot, space is automatically true
    toLaunch = false; //button is not pressed, so automatically false
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

    //if ball under sensor, then sensor.get() outputs 0.08
    double bottomIRSensorTolerance = 0.08;

    //if ball in front of sensor, output is 0.4 
    double topIRSensorTolerance = 0.4; 

    //if ball in front of sensor, output is 9
    int middleColorSensorTolerance = 9;  

    //checks if there is space above for the balls to travel up
    //stops moving motors if ball is already at top
    if(irTop.get() > topIRSensorTolerance){
      spaceAtTop = false;

      //resets all motor speeds
      motorBottom.set(0); 
      motorTop.set(0);
      flyWheelMotor.set(0);
    }

    /*checks if there is a ball in 'waiting area' 
    and if there is space for a ball to be rolled up*/
    if(spaceAtTop && (irBottom.get() >= bottomIRSensorTolerance)){
        motorBottom.set(-0.2); //sets bottom motor speed 

        /*checks if there is a ball aready in robot that needs to move up 
        before another can be rolled in from waiting area*/
        if(colorSensor.getProximity() > middleColorSensorTolerance){
          motorTop.set(-0.2); //sets top motor speed
        }

    } else { //resets all motors
      motorBottom.set(0);
      motorTop.set(0);
    }

    //initates launching sequence if button is pressed
    if(controller.getAButtonPressed()){
      toLaunch = true;
    }

    //launching sequence
    if(toLaunch){

      /*checks if ball is not all the way at the top of robot
      if not, moves it till sensor lies in empty space between two balls 
      this ensures ball is high enough to be launched*/
      if(irTop.get() > topIRSensorTolerance){
        motorTop.set(-0.3);
      } else { //if a ball is already high enough, ball is launched 
        flyWheelMotor.set(0.3);
        motorTop.set(0); 
        toLaunch = false; //launching sequence set to false
      }
    }
  }


  @Override
  public void simulationPeriodic() {
    
    // This method will be called once per scheduler run during simulation
  }
}
