package ca.mcgill.ecse211.project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class is used to define static resources in one place for easy access and to avoid
 * cluttering the rest of the codebase. All resources can be imported at once like this:
 * 
 * <p>{@code import static ca.mcgill.ecse211.lab3.Resources.*;}
 */
public class Resources {

 public static final EV3ColorSensor colourSensor1 = new EV3ColorSensor(SensorPort.S3); // left
 public static final EV3ColorSensor colourSensor2 = new EV3ColorSensor(SensorPort.S4); // right
  /*
   * Ideal RGB values for the big and small rooms 
   * 
   */
  public static final int[] BIG_ROOM_BLUE = {23, 80, 96};
  public static final int[] BIG_ROOM_YELLOW = {167, 81, 30}; // 213, 115, 28
  public static final int[] BIG_ROOM_GREEN = {48, 78, 22}; 
  public static final int[] BIG_ROOM_ORANGE = {90, 21, 10}; // 164, 38, 12
  public static final int[] BIG_ROOM_BLACK = {31, 12, 12};


  public static final int[] SMALL_ROOM_BLUE = {25, 90, 105}; // 15, 54, 51
  public static final int[] SMALL_ROOM_YELLOW = {155, 74, 25}; // 168, 86, 22
  public static final int[] SMALL_ROOM_GREEN = {56, 89, 20}; // 40, 74, 12
  public static final int[] SMALL_ROOM_ORANGE = {96, 23, 9}; // 145, 35, 12
  public static final int[] SMALL_ROOM_BLACK = {36, 28, 33};


  /*
   * Standard deviations values for each color, for each rom
   */
  public static final int SD_BIG_BLUE = 10;
  public static final int SD_BIG_YELLOW = 10;
  public static final int SD_BIG_GREEN = 10;
  public static final int SD_BIG_ORANGE = 10;

  public static final int SD_SMALL_BLUE = 10;
  public static final int SD_SMALL_YELLOW = 10;
  public static final int SD_SMALL_GREEN = 10;
  public static final int SD_SMALL_ORANGE = 10;


  /*
   * Color Sensor Distance Threshold: used to determine the color being detected in ColorSensor.java
   */
  public static final int COLOR_DISTANCE_THRESHOLD = 25;

  /*
   * Define the room type in which the lab is performed.
   */
  public static final String CURRENT_ROOM = "SMALL";

  /*
   * Color names array
   */
  public static final String[] COLOR_NAMES = {"BLUE", "ORANGE", "YELLOW", "GREEN", "BLACK"};

  /**
   * Time for which the navigation thread sleeps for
   */
  public static final int NAVIGATION_SLEEP = 100;

  /**
   *  90 degrees
   */
  public static final int NINETY_DEGREES = 90;

  /** Period of sampling f (ms). */
  public static final int SAMPLING_INTERVAL_US = 25;       

  /** Period of display update (ms). */
  public static final int DISPLAY_SLEEP_PERIOD = 100; 

  /**
   * Time for which the method that returns the colour calculated by the colour sensor sleeps for (ms)
   */
  public static final int COLOUR_SENSOR_SLEEP = 50;

  public static final int DEGREES_MAX = 360;
  public static final int RIGHT_ANGLE = 90;
  public static final int HALF_RIGHT_ANGLE = 45;

  /**
   * Distance from end of US sensor to center of robot
   */
  public static final int SENSOR_TO_CENTER_DIST = 7;

  /**
   * PI
   */
  public static final double PI=3.1415927; 
  /**
   * The ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2);  

  /**
   * The color sensor.
   */
  public static final EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);

  /**
   * The poll sleep time, in milliseconds.
   */
  public static final int POLL_SLEEP_TIME = 35; 
  
  /*
   * Distance from color sensor to wheel center of robot.
   */
  public static final int COLOR_SENSOR_TO_CENTER_DIST = 6;

  /**
   * General sleep time.
   */
  public static final int GENERAL_SLEEP = 250; 

  /**
   * the line y = D as instructed in the tutorial notes
   */
  public static final int D = 35;

  /*
   * Absorbance threshold.
   */
  public static final int ABSORBANCE_THRESHOLD = 20;

  /**
   * The wheel radius in centimeters.
   */
  public static final double WHEEL_RADIUS = 2.13; // 2.098

  /*
   * Wheel radius to be used by the odometer.
   */
  public static final double WHEEL_RADIUS_ODO = 2.37;
  
  /**
   * The robot width in centimeters.
   */
  public static final double BASE_WIDTH = 8.55; // 8.88 (ideal)

  /**
   * The speed at which the robot moves forward in degrees per second.
   */
  public static final int FWD_SPEED = 250;

  /**
   * Higher/regular speed;
   */
  public static final int MOTOR_HIGH = 150;

  /**
   * Slow speed.
   */
  public static final int MOTOR_LOW = 90;

  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATION_SPEED = 60;          
  
  /**
   * Threshold to determine if looking at the corner
   */
  public static final int DIST_THRESHOLD = 30;
  
  /*
   * Black line threshold (for big room).
   */
  public static final int BLACK_LINE_THRESHOLD = 45;
  
  /*
   * Blue line threshold (for small room).
   */
  public static final int BLUE_LINE_THRESHOLD_L = 30;
  public static final int BLUE_LINE_THRESHOLD_R = 40;

  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int ACCELERATION_deg=2500;

  /**
   * Timeout period in milliseconds.
   */
  public static final int TIMEOUT_PERIOD_ms = 3000;

  /**
   * The tile size in centimeters. Note that 30.48 cm = 1 ft.
   */
  public static final double TILE_SIZE_cm = 30.48;
  /**
   * Number of degrees in one radian, equivalent approximately to 180/Math.PI.(used to convert to degrees)
   */
  public static final double DEGS_PER_1RAD= 57.2598;                    //new

  /**
   * Number of radians in one degree, equivalent approximately to Math.PI/180 (used to convert to radians)
   */
  public static final double RADS_PER_1DEG=0.01745329251;                 //new
  
  /*
   * Next waypoint coordinates
   */
  public static int NEXT_X;
  public static int NEXT_Y;
  
  /*
   * Global flag to that keeps track of whether or not we have detected a ring.
   */
  public static boolean HAS_DETECTED_OBJECT = false;
  
  /* 
   * To keep track of the number of rings detected.
   */
  public static int NUM_RINGS_DETECTED = 0;
  
  /*
   * To keep track of the rings color detected.
   */
  public static String RINGS_DETECTED_STRING = "";
  
  /*
   * To know when first corner localization is done.
   */
  public static boolean DONE_CORNER_LOCALIZATION = false;
  
  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);

  /**
   * The LCD.
   */
  public static final TextLCD lcd = LocalEV3.get().getTextLCD();

  /**
   * The odometer.
   */
  public static Odometer odometer = Odometer.getOdometer();
  
  
 /*   
  * For Threads
  */
  public static ColorSensor cSensor = new ColorSensor();
  public static UltrasonicSensor USSensor = new UltrasonicSensor();
  public static Odometer odo = new Odometer();
  public static Navigation navi = new Navigation();
  
  public static Thread colorSensorThread = new Thread(cSensor);
  public static Thread ultraSonicSensorThread = new Thread(USSensor);
  public static Thread odometerThread = new Thread(odo);
  public static Thread navigationThread = new Thread(navi);
  
}