//Zone 1: Tej Patel   101169687
//Zone 2: Shuqi Yang  101114181
//Zone 3: Vishal Parag Parmar

import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.TouchSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Lidar;

public class ProjectController2 {

  private static final byte    CAMERA_WIDTH = 64;
  private static final byte    CAMERA_HEIGHT = 64;
  private static final double  GRIPPER_MOTOR_MAX_SPEED = 0.1;
  
  // Various modes for the robot to be in
  static final byte    STRAIGHT = 0;
  static final byte    SPIN_LEFT = 1;
  static final byte    PIVOT_RIGHT = 2;
  static final byte    CURVE_LEFT = 3;
  static final byte    CURVE_RIGHT = 4;
  static final byte Find_Blue =5;
  static final byte SPIN_RIGHT =6;
  static final byte Wait =7;
  static final byte Find_Green = 8;
  static final byte Back =9;
  static final double  MAX_SPEED      = 2;
  
  private static Robot           robot;
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
  private static Accelerometer   accelerometer;
  private static Camera          camera;
  
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

  public static boolean isblue(int r, int g, int b){
    //System.out.println("(" + r + ", " + g + ", " + b + ")");
    return (r <30) && (g < 30) && (b > 60);
  }

  public static boolean isgreen(int r, int g, int b){
    //System.out.println("(" + r + ", " + g + ", " + b + ")");
    return (r <70) && (g > 70) && (b < 70);
  }

  // This is where it all begins
  public static void main(String[] args) {
    robot = new Robot();
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
    
    // Prepare the accelerometer
    accelerometer = new Accelerometer("accelerometer");
    accelerometer.enable(timeStep);
    
    // Prepare the camera
    camera = new Camera("camera");
    camera.enable(timeStep);
    
    // Prepare the Compass sensor
    compass = robot.getCompass("compass");
    compass.enable(timeStep);
    
    // Prepare the jar detecting sensor
    jarDetectedSensor= new TouchSensor("touch sensor");
    jarDetectedSensor.enable(timeStep);
    
    //  Prepare the Lidar sensor
    Lidar lidar = new Lidar("Sick LMS 291");
    lidar.enable(timeStep);
      
    // Run the robot
    byte currentMode = STRAIGHT;
    double leftSpeed, rightSpeed;
    leftSpeed = 10;
    rightSpeed = 10;

    int count =0;
    boolean getCup = false;
    boolean seaBlue = false;
    int wantbearing = 0;

    while (robot.step(timeStep) != -1) {
      // SENSE: Read the sensors
      double compassReadings[] = compass.getValues();
      double rad = Math.atan2(compassReadings[0],compassReadings[2]);
      double bearing = -((rad + Math.PI) / Math.PI * 180.0);
      if (bearing > 180){
        bearing = 360 - bearing;
      }
      if(bearing < -180){
        bearing = 360 + bearing;
      }

      double leftSide = leftSideSensor.getValue();
      double leftAngle = leftAngledSensor.getValue();
      double leftAhead = leftAheadSensor.getValue();

      double rightAhead =rightAheadSensor.getValue();
      double rightAngled = rightAngledSensor.getValue();
      double rightSide = rightSideSensor.getValue();

      int[] image = camera.getImage();
      int blueleftCount = 0;
      int bluerightCount = 0;
      int bluecenterCount = 0;
      int greenleftCount = 0;
      int greenrightCount = 0;
      int greencenterCount = 0;
      int EPSILON = 2;
      int h = (CAMERA_HEIGHT - 1)/2;
      for(int a =0; a<40; a++) {
        for (int w = 0; w < CAMERA_WIDTH; ++w) {
          int r = Camera.imageGetRed(image, CAMERA_WIDTH, w, a);
          int g = Camera.imageGetGreen(image, CAMERA_WIDTH, w, a);
          int b = Camera.imageGetBlue(image, CAMERA_WIDTH, w, a);
          if (isblue(r, g, b)) {
            if (w <= (CAMERA_WIDTH - 1) / 3){
              ++blueleftCount;}
              //Else if the pixel is in the right 1/3 of the image
            else if (w >= 2 * (((CAMERA_WIDTH) - 1) / 3)){
              ++bluerightCount;}
              //Else the pixel is in the center
            else{
              ++bluecenterCount;
            }
          }
        }
      }

      for(int a =30; a<CAMERA_WIDTH; a++) {
        for (int w = 0; w < CAMERA_WIDTH; ++w) {
          int r = Camera.imageGetRed(image, CAMERA_WIDTH, w, a);
          int g = Camera.imageGetGreen(image, CAMERA_WIDTH, w, a);
          int b = Camera.imageGetBlue(image, CAMERA_WIDTH, w, a);
          if (isgreen(r, g, b)) {
            if (w <= 19) {
              ++greenleftCount;
            }
              //Else if the pixel is in the right 1/3 of the image
            else if (w >= 42) {
              ++greenrightCount;
            }
              //Else the pixel is in the center
            else {
              ++greencenterCount;
            }
          }
        }
      }
      System.out.print("greenleftCount" + greenleftCount);
      System.out.print("  greenrightCount" + greenrightCount);
      System.out.print("  greencenterCount" + greencenterCount+ "\n");

      boolean findBlue = false;
      findBlue = (blueleftCount > (bluerightCount + EPSILON)) || (bluerightCount > (blueleftCount + EPSILON)) || (bluecenterCount > 20);

      boolean findGreen = false;
      //findGreen = (greenleftCount > (greenrightCount + EPSILON)) || (greenrightCount > (greenleftCount + EPSILON)) || (greencenterCount > 30);
      if(greenleftCount+greenrightCount+greencenterCount > 70)
      {
        findGreen = true;
      }

      System.out.print("Left side: "+leftSide+ "  left Angle: "+leftAngle+" left Ahead: "+leftAhead);
      System.out.print("--  right side: "+rightSide+ "  right Angle: "+ rightAngled+" right Ahead: "+rightAhead+" \n");
      //System.out.print("bearing "+ bearing+"\n");

      boolean sideTooClose  = rightSideSensor.getValue() < 0.4;
      boolean sideTooFar  = rightSideSensor.getValue() > 998;
      boolean frontTooClose  = (rightAheadSensor.getValue() <0.4) || (leftAheadSensor.getValue() <0.4) || (rightAngledSensor.getValue() <0.4) || (leftAngledSensor.getValue() <0.4);
      boolean lostContact  = rightSideSensor.getValue() > 1;

      // THINK: Make a decision as to what MODE to be in
      switch (currentMode) {
        case STRAIGHT:
          //System.out.println("STRAIGHT");
          if (findBlue)  {
            seaBlue = true;
            currentMode = Find_Blue;
            break;
          }
          if(rightSideSensor.getValue() <0.1){
            currentMode = CURVE_LEFT;
            break;
          }
          if(rightSideSensor.getValue()<1 && leftSideSensor.getValue()<1 && seaBlue){
            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
            delay(5, 1);
            openCloseGripper(0.099f);
            delay(50, 1);
            currentMode = Back;
          }

          if(rightAheadSensor.getValue() <0.8 && leftAheadSensor.getValue()<0.8 && leftSide < 1){currentMode = SPIN_RIGHT; break;}
          if(rightAheadSensor.getValue() <0.8 && leftAheadSensor.getValue()<0.8 && rightSide < 1){currentMode = SPIN_LEFT; break;}

          break;
        case CURVE_RIGHT:
          //System.out.println("CURVE_RIGHT");
          break;
        case PIVOT_RIGHT:
          //System.out.println("PIVOT_RIGHT");
          break;
        case SPIN_LEFT:
          //System.out.println("SPIN_LEFT");
          if ((bearing > -10 || bearing < 10) && seaBlue)
          {
            if(bearing > 89 && bearing <91){
              currentMode = STRAIGHT;
              wantbearing = 1;
              break;
            }
          }
          if(wantbearing==2){
            if(bearing > 89 && bearing <91){
              currentMode = STRAIGHT;
              wantbearing = 1;
              break;
            }
          }
          if(wantbearing == 3){
            if(bearing > -1 && bearing <1){
              currentMode = STRAIGHT;
              wantbearing = 2;
              break;
            }
          }
          if(wantbearing == 0){
            if(bearing > -91 && bearing <-89){
              currentMode = STRAIGHT;
              wantbearing = 3;
              break;
            }
          }
          if(wantbearing == 1){
            if(bearing > 179 || bearing <-179){
              currentMode = STRAIGHT;
              wantbearing = 0;
              break;
            }
          }
          break;
        case CURVE_LEFT:
          //System.out.println("CURVE_LEFT");
          if (rightSideSensor.getValue()>0.18) { currentMode = STRAIGHT;  break; }
          break;
        case Find_Blue:
          //System.out.println("Find_Blue");
          if(frontTooClose && (bearing > -135 && bearing < -45 )){ currentMode = SPIN_RIGHT; break; }
          if(rightAheadSensor.getValue() <1 && leftAheadSensor.getValue()<1 && (bearing > -10 && bearing < 10 ))
          {
            wantbearing = 2;
            currentMode = SPIN_LEFT;
            seaBlue = true;
            break;
          }
          break;
        case SPIN_RIGHT:
          //System.out.println("SPIN_RIGHT");
          //System.out.println("wantbearing "+ wantbearing);
          if ((bearing > 178 || bearing < -178) && seaBlue) {currentMode = Wait; break;}
          if(wantbearing==0){
            if(bearing > 89 && bearing <91){
              currentMode = STRAIGHT;
              wantbearing = 1;
              break;
            }
          }
          if(wantbearing == 1){
            if(bearing > -1 && bearing <1){
              currentMode = STRAIGHT;
              wantbearing = 2;
              break;
            }
          }
          if(wantbearing == 2){
            if(bearing > -91 && bearing <-89){
              currentMode = STRAIGHT;
              wantbearing = 3;
              break;
            }
          }
          if(wantbearing == 3){
            if(bearing > 179 || bearing <-179){
              currentMode = STRAIGHT;
              wantbearing = 0;
              break;
            }
          }
          break;
        case Wait:
          //System.out.println("Wait");
          if(findGreen) {
            openCloseGripper(0.999f);
            currentMode = Find_Green;
            delay(50, 1);
            break;
          }
          break;
        case Find_Green:
          //System.out.println("Find_Green");
          //System.out.println("greenleftCount "+ greenleftCount);
          //System.out.println("greenrightCount "+ greenrightCount);

          if (greenleftCount > (greenrightCount + EPSILON)){
            System.out.println("Curve left");
            rightSpeed = 4;
            leftSpeed = 2;

          }
          else if (greenrightCount > (greenleftCount + EPSILON)){
            System.out.println("Curve right");
            rightSpeed = 2;
            leftSpeed = 4;
          }
          else{
            //System.out.println("speed =2");
            leftSpeed  = 2;
            rightSpeed = 2;
          }
          if(jarDetectedSensor.getValue()>0){

            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
            delay(5, 1);
            openCloseGripper(0.02f);
            delay(50, 1);
            currentMode = Back;
            getCup = true;
            seaBlue = false;
            wantbearing = 0;
          }
          break;
        case Back:
          //System.out.println(Back);
          if(bearing >80 && bearing <100){
            if(leftSideSensor.getValue()>4){
              wantbearing = 0;
              currentMode = SPIN_LEFT;
              seaBlue = false;
              count ++;
            }
          }
          else{
            if(rightSideSensor.getValue()>4){
              currentMode = SPIN_RIGHT;
            }
          }
          break;
      }
           
      // REACT: Move motors according to the MODE
      switch(currentMode) {
        case SPIN_LEFT:
          leftSpeed  = -1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
        case SPIN_RIGHT:
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = -1 * MAX_SPEED;
          break;
        case PIVOT_RIGHT:
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = 0.25 * MAX_SPEED;
          break;
        case CURVE_LEFT:
          leftSpeed  = 0.9 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
        case CURVE_RIGHT:
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = 0.9 * MAX_SPEED;
          break;
        case Wait:
          leftSpeed  = 0 * MAX_SPEED;
          rightSpeed = 0 * MAX_SPEED;
          break;
        case Back:
          leftSpeed  = -12;
          rightSpeed = -12;
          break;
        case Find_Green:
          break;
        default:
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
      }

      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);

      if(count == 4){
        break;
      }
    }
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);
  }
}
