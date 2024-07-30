// Author: Mark Lanthier (SN:100000001)
import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.TouchSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;

public class ProjectController3 {

  private static final byte    CAMERA_WIDTH = 64;
  private static final byte    CAMERA_HEIGHT = 64;
  private static final double  GRIPPER_MOTOR_MAX_SPEED = 0.1;
  
  // Various modes for the robot to be in
  static final byte AAAA  = 0;
  static final byte BBBB  = 1;
  static final byte CCCC  = 2;
    
  private static Supervisor      robot;
  private static Motor           leftMotor;
  private static Motor           rightMotor;
  private static Motor           gripperLift;
  private static Motor           gripperLeftSide;
  private static Motor           gripperRightSide;
  private static DistanceSensor  leftSideSensor; 
  private static DistanceSensor  rightSideSensor;
  private static DistanceSensor  leftAheadSensor; 
  private static DistanceSensor  rightAheadSensor;
  private static DistanceSensor  leftAngledSensor; 
  private static DistanceSensor  rightAngledSensor;
  private static TouchSensor     jarDetectedSensor;
  private static Compass         compass;
  private static Camera          camera;
  private static Field           TranslationField;
  
  // Wait for a certain number of milliseconds
  private static void delay(int milliseconds, int timeStep) {
    int elapsedTime = 0;
    while (elapsedTime < milliseconds) {
      robot.step(timeStep);
      elapsedTime += timeStep;
    }
  }
  
  // Put the gripper up/down to the given position
  // -0.0499 is "up all the way" and 0.001 is down as mush as it can
  public static void liftLowerGripper(float position) {
    gripperLift.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperLift.setPosition(position);
  }

  // Put the gripper open/closed to the given position
  // 0.099 is "open all the way" and 0.01 is closed as mush as it can
  public static void openCloseGripper(float position) {
    gripperLeftSide.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperRightSide.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperLeftSide.setPosition(position);
    gripperRightSide.setPosition(position);
  }

  // This is where it all begins
  public static void main(String[] args) {
    robot = new Supervisor();
    
    // Code required for being able to get the robot's location
    Node    robotNode = robot.getSelf();
    TranslationField = robotNode.getField("translation");
    
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Set up the motors
    leftMotor = robot.getMotor("left wheel");
    rightMotor = robot.getMotor("right wheel");
    gripperLift = robot.getMotor("lift motor");
    gripperLeftSide = robot.getMotor("left finger motor");
    gripperRightSide = robot.getMotor("right finger motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0); 
    
    // Get and enable the distance sensors
    leftSideSensor = robot.getDistanceSensor("so0"); 
    leftAngledSensor = robot.getDistanceSensor("so1"); 
    leftAheadSensor = robot.getDistanceSensor("so3"); 
    rightAheadSensor = robot.getDistanceSensor("so4"); 
    rightAngledSensor = robot.getDistanceSensor("so6"); 
    rightSideSensor = robot.getDistanceSensor("so7"); 
    leftAheadSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);
    leftAngledSensor.enable(timeStep);
    rightAngledSensor.enable(timeStep);
    leftSideSensor.enable(timeStep);
    rightSideSensor.enable(timeStep);
    
    // Prepare the camera
    camera = new Camera("camera");
    camera.enable(timeStep);
    
    // Prepare the Compass sensor
    compass = robot.getCompass("compass");
    compass.enable(timeStep);
    
    // Prepare the jar detecting sensor
    jarDetectedSensor= new TouchSensor("touch sensor");
    jarDetectedSensor.enable(timeStep);
    
    
    // Run the robot
    byte currentMode = AAAA;
    
    while (robot.step(timeStep) != -1) {
      // SENSE: Read the sensors


      // THINK: Make a decision as to what MODE to be in
      switch(currentMode) {
        case AAAA:
          break;
          
        case BBBB:
          break;
          
        case CCCC:
          break;
      }
           
      // REACT: Move motors according to the MODE
      switch(currentMode) {
        case AAAA: //System.out.println("Robot 3: AAAA");
          break;
          
        case BBBB: //System.out.println("Robot 3: BBBB");
          break;
          
        case CCCC: //System.out.println("Robot 3: CCCC");
          break;
      }
    }
  }
}
