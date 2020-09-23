
package ca.mcgill.ecse211.project;


import lejos.hardware.Sound;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.ArrayList;
import java.util.Collections;

public class UltrasonicSensor extends Thread {

  /**
   * distance calculated by the Ultrasound sensor
   */
  private static int distance;     
  
  // ArrayList to store all the readings object
  public static  ArrayList<Reading> readings = new ArrayList<Reading>();

  /**
   * Buffer (array) to store US samples. Declared as an instance variable to avoid creating a new
   * array each time {@code readUsSample()} is called.
   */
  private float[] usData = new float[usSensor.sampleSize()];        

  private Navigation nav = new Navigation();

  /**
   * The Localization type.
   */
  private String Localization_type;            

  /**
   * Noise margin in cm
   */
  private final static double K = 2;         


  /**
   * the point at which the measured distance falls above D - K
   */
  private static double angle1=0;
  /**
   * the point at which the measured distance falls above D + K
   */
  private static double angle2=0;
  /**
   * average of angle1 and angle2
   */
  private static double avg_angle_12;
  /**
   * the point at which the measured distance falls below D + K
   */
  private static double angle3=0;
  /**
   * the point at which the measured distance falls below D - K
   */
  private static double angle4=0;
  /**
   * average of angle3 and angle4
   */
  private static double avg_angle_34;
  /**
   * angle calculated  by the equation given in the tutorial notes, this angle is used to correct the direction of the EV3
   */
  private static double deltaTheta;               
  
  // instantiate ColorSensor object
  private ColorSensor colorSensor = new ColorSensor();
  
  // to hold the x, y, t values of the odometer
  private double[] position;
  
  // offset angle, values between 0-359
  private double deltaAngle;
  
  // cummulative offset angle, values from 0-inf (used for US localization)
  private double totalDeltaAngle;

  // Reading object which will store the minimum distance and the offset angle associated to it
  private Reading minReading;


  /**
   * Constructor
   */
  public UltrasonicSensor() {
  }

  /*
   * Main run method.
   */
  public void run() {
    
    usLocalize();
     
    //DONE_CORNER_LOCALIZATION = true;
    
    
    
    while(!DONE_CORNER_LOCALIZATION) {
      continue;
    }
    
    Main.sleepFor(GENERAL_SLEEP);
    
    while(true) {
      detectObject();
    }
   
   
  } //end of run method

  /*
   * Method to continuously check if our US sensor detects an object.
   */
  public void detectObject() {

    int d = 100;
    int min_distance = 2;
    int max_distance = 4;
    boolean hasDetectedObject = false;
    
    // while US sensor has not detected an object...
    while(!hasDetectedObject) {
      
      // get US sensor reading
      d = readUsDistance();
      
      // when we come within around 2-4 cm away from the ring
      if(d >= min_distance && d <= max_distance) {
        hasDetectedObject = true; // update local flag
        colorSensor.objectDetected(); // trigger the objectDetected method in ColorSensor class
        nav.stopMotors();
      }

      Main.sleepFor(POLL_SLEEP_TIME);
      
    }
    
    hasDetectedObject = false; // update local flag

    return;

  }

  /**
   *    In this method, A rising edge is detected, its average theta is calculated,
   *    we then continue rotating in the same direction until a falling edge is detected,
   *    the average theta of the falling edge is then calculated. We then use the equation as explained
   *    in the tutorial notes to calculate delta theta which is then added to the angle of rotation at which the
   *    falling edge is detected ( new theta = delta theta + theta at which the falling edge is detected). We then
   *    turn by new theta in the opposite direction so that the EV3 facing in the correct direction , which is the 0 degree axis.                       
   *   
       In short : Detect a Rising edge, continue rotating in the same direction, detect a falling edge, rotate the EV3 to correct direction ( 0 degree axis)
   */
  public void Rising_Edge () {     
    (new Thread() {                         

      public void run() {                        
        leftMotor.setSpeed(ROTATION_SPEED);      
        rightMotor.setSpeed(ROTATION_SPEED);       

        while (distance < D - K) {               //Detect rising edge
          leftMotor.forward();
          rightMotor.backward();
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        angle1 = odometer.getXyt()[2];         // returns the angle rotated by EV3 from its starting position
        Sound.beep();


        while (distance < D + K) {      
          leftMotor.forward();
          rightMotor.backward();
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        angle2 = odometer.getXyt()[2];      // returns the angle rotated by EV3 from its starting position
        Sound.beep();
        avg_angle_12 = (angle1+angle2)/2.0;


        while (distance > D + K) {             //Detect falling edge
          leftMotor.forward();
          rightMotor.backward();
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        angle3 = odometer.getXyt()[2];
        Sound.beep();

        while (distance > D - K) {     
          leftMotor.forward();
          rightMotor.backward();
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        angle4 = odometer.getXyt()[2];
        Sound.beep();

        avg_angle_34 = (angle3+angle4)/2.0;

        if (avg_angle_34 > avg_angle_12) {
          deltaTheta = 50 - (avg_angle_34+avg_angle_12) / 2.0;  //used
        } else {
          deltaTheta = 235 - (avg_angle_34+avg_angle_12) / 2.0;     
        }

        double newTheta = odometer.getXyt()[2] + deltaTheta;
        turnBy(-(newTheta));

        odometer.setTheta(0);    

        leftMotor.stop(true);
        rightMotor.stop(false);

      }        //end of run method
    }).start();
  } //end of rising edge method

  //--------------------------------------------------------------------------------------

  /**
   *     
   In this method, A falling edge is detected, its average theta is calculated,
   we then continue rotating in the same direction until a rising edge is detected,
   the average theta of the rising edge is then calculated. We then use the equation as explained
   in the tutorial notes to calculate delta theta which is then added to the angle of rotation at which the
   rising edge is detected ( new theta = delta theta + theta at which the rising edge is detected). We then
   turn by new theta in the opposite direction so that the EV3 facing in the correct direction , which is the 0 degree axis.

   // In short : Detect a Falling edge, continue rotating in the same direction, detect a rising edge, rotate the EV3 to correct direction ( 0 degree axis)
   */
  public void Falling_Edge () {      

    (new Thread() {

      public void run() {
        leftMotor.setSpeed(ROTATION_SPEED);
        rightMotor.setSpeed(ROTATION_SPEED);

        while (distance > D + K) {              //Detect falling edge
          leftMotor.forward();
          rightMotor.backward();
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        angle3 = odometer.getXyt()[2];
        Sound.beep();


        while (distance > D - K) {      
          leftMotor.forward();
          rightMotor.backward();
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        angle4 = odometer.getXyt()[2];
        Sound.beep();

        avg_angle_34 = (angle3+angle4)/2.0;


        while (distance < D - K) {             //Detect rising edge
          leftMotor.forward();
          rightMotor.backward();
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        angle1 = odometer.getXyt()[2];
        Sound.beep();

        while (distance < D + K) {     
          leftMotor.forward();
          rightMotor.backward();
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        angle2 = odometer.getXyt()[2];
        Sound.beep();

        avg_angle_12 = (angle1+angle2)/2.0;

        if (avg_angle_34 > avg_angle_12) {
          deltaTheta = 52 - (avg_angle_34 + avg_angle_12) / 2.0;
        } else {
          deltaTheta = 230 - ((avg_angle_34 + avg_angle_12)) / 2.0; 
        }
        double newTheta = odometer.getXyt()[2] + deltaTheta;
        turnBy(-(newTheta));

        odometer.setTheta(0);

        leftMotor.stop(true);
        rightMotor.stop(false);

      } // end of run method
    }).start();

  }     //end of Falling edge method


  /**
   * Returns the distance between the US sensor and an obstacle in cm.
   * 
   * @return the distance between the US sensor and an obstacle in cm
   */
  public int readUsDistance() {         
    // extract from buffer, convert to cm, cast to int, and filter
    usSensor.fetchSample(usData, 0);  
    int distance = ((int) (usData[0] * 100.0));
    //System.out.println("US distance: " + distance);
    return distance;
  }

  /**
   * 
   * @return Returns the distance calculated by the Ultrasonic sensor
   */
  public static int getDistance() {     
    return distance;
  }

  /**
   * Turns the robot by a specified angle. Note that this method is different from {@code Navigation.turnTo()}. For
   * example, if the robot is facing 90 degrees, calling {@code turnBy(90)} will make the robot turn to 180 degrees, but
   * calling {@code Navigation.turnTo(90)} should do nothing (since the robot is already at 90 degrees).
   * 
   * @param angle the angle by which to turn, in degrees
   */
  public static void turnBy(double angle) {
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), false);
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle.
   * 
   * @param angle the input angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * BASE_WIDTH * angle / 360.0);
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance the input distance
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RADIUS));
  }
  
  public void usLocalize() {
    Reading intialSweep = sweep();

    // Now that we have exited the main loop, cut the motors
    nav.stopMotors();

    Main.sleepFor(GENERAL_SLEEP);

    // Orient the robot in the direction pointing towards the minimum distance
    nav.rotateRobotBy(intialSweep.getDeltaAngle());

    Main.sleepFor(GENERAL_SLEEP);

    // Rotate the robot about 90 degrees
    nav.rotateRobotBy(RIGHT_ANGLE);

    // Now, here the logic of the following:
    // after having turned 90 degrees in the previous step, if the next US sensor reading is large, then the robot must
    // be oriented towards the 0 degree line. If not, then another clockwise 90 degree rotation must be made

    // Start by taking the average of two readings for accuracy
    int dist = readUsDistance();
    if(dist < DIST_THRESHOLD) { // then we are facing against the other wall, at a very close similar minimum distance
      nav.rotateRobotBy(RIGHT_ANGLE);
      Main.sleepFor(GENERAL_SLEEP);
      nav.stopMotors();
    } else { // if not, then we are currently facing the 0 degree line!
      nav.stopMotors();  
    }

    Main.sleepFor(GENERAL_SLEEP); 

    // Note: at this point, the robot should be oriented towards the 0 degrees line (along the defined y-axis)

    //Button.waitForAnyPress();

    // Angle at around 45 degrees away from the y-axis
    nav.rotateRobotBy(HALF_RIGHT_ANGLE);

    Main.sleepFor(GENERAL_SLEEP);

    // Get to (1, 1) point on board
    double distanceToTravel = computeDistanceToOneOne(minReading.getDistance());
    nav.moveStraightFor(distanceToTravel);

    // Now orient wheel towards 0 degrees
    nav.rotateRobotBy(-HALF_RIGHT_ANGLE);
    
    // Init odometer at (1, 1)
    odometer.setXyt(TILE_SIZE_cm,TILE_SIZE_cm, 0);
    LightLocalizer.localize(); //starts taking samples
    Main.sleepFor(GENERAL_SLEEP);
    nav.turnTo_Localizer(0);
    LightLocalizer.localize2();
    
    // done corner localization
    DONE_CORNER_LOCALIZATION = true;

    // We should be all good at this point!
  }
  public Reading sweep() {

    while(totalDeltaAngle <= (DEGREES_MAX)) { // use the <= 360 degree angle condition if it works

      leftMotor.setSpeed(MOTOR_LOW);
      rightMotor.setSpeed(MOTOR_LOW);
      leftMotor.forward();
      rightMotor.backward();

      // Setup inputs for Reading object creation
      int d = readUsDistance();

      // Retrieve the angle
      position = odometer.getXyt();
      deltaAngle = position[2];
      totalDeltaAngle = position[3];

      // Initialize a Reading object
      Reading r = new Reading(deltaAngle, d);
      readings.add(r); // append reading data to local arraylist

      minReading = findMinDistance(readings); // find the minimum reading out of the arraylist of Reading objects

      //lcd.drawString("Min Distance: "+minReading.getDistance()+"", 0, 6);
      //lcd.drawString("Angle Offset: "+minReading.getDeltaAngle()+"", 0, 7);

      //Main.sleepFor(LOOP_SLEEP_TIME);

    }

    return minReading;
  }

  /**
   * Method to compute the distance remaining to travel by the robot to the (1, 1) point
   */
  public double computeDistanceToOneOne(int minDistance) {
    double hyp = ( (TILE_SIZE_cm - (minDistance + SENSOR_TO_CENTER_DIST))/ ( Math.sin( PI / 4 )) );
    return hyp;
  }

  /*
   * Method to find the minimum reading out of the arraylist of Reading objects.
   * Idea is simply to sort the arraylist by distance attribute and then pick the first element in the list.
   */
  public Reading findMinDistance(ArrayList<Reading> readings) {
    Collections.sort(readings);
    return readings.get(0);
  }
  
}