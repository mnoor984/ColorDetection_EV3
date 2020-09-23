package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.Main.*;

public class ColorSensor extends Thread {
  /**
   * Use euclidean distance formula to determine if sample corresponds to objects with colors defined 
   * Note: formula in the requirements 
   * @param currentRGB the sample RGB reading
   * @param targetColor Colour of target allows for use in all distanceChecking, no matter the color of target
   * @param location adjustments due to ambient light 
   * @return errorDistance 
   */

  // to hold an RGB color reading from the light sensor
  private static int RGBData[] = new int[3];

  // instantiate a Navigation class
  private Navigation nav = new Navigation();

  /** 
   * (non-Javadoc)
   * @see java.lang.Thread#run()
   * Start ColorSensor. Stores RGB values in a static array which
   * can be accessed with getter getRBGdata(). 
   */
  //--------------------------------------------------------------------------
  public void run() {

    // Nothing needed in this run method since UltrasonicSensor class will trigger
    // the objectDetected method from this class.
    
    //analyzeColor();
    
  }

  //--------------------------------------------------------------------------


  /*
   * Method which will read from the light/color sensor and prints out String color name.
   */
  public String analyzeColor() {

    SampleProvider RGBSensor = Resources.colorSensor.getRGBMode();
    float buffer[] = new float[RGBSensor.sampleSize()];
    int sample = 0;
    int num_samples = 5;
    String color;
    String finalColor;
    String[] colorGuesses = new String[num_samples];

    // we take some samples for accuracy
    while (sample < num_samples) {

      // Read rgb values from color sensor
      RGBSensor.fetchSample(buffer, 0);
      RGBData[0] = (int)Math.round(buffer[0]*1000); // read Red
      RGBData[1] = (int)Math.round(buffer[1]*1000); // read Green
      RGBData[2] = (int)Math.round(buffer[2]*1000); // read Blue

      // call determineColor method to read color
      color = determineColor(RGBData, CURRENT_ROOM);
      colorGuesses[sample] = color;
      sample++; 

      Main.sleepFor(GENERAL_SLEEP);

    }

    finalColor = colorGuesses[sample-1];
    // Output color to LCD screen
    lcd.drawString("Color: " + finalColor, 0, 2);

    //System.out.println("Color: " + colorGuesses[sample-1]);

    return finalColor;

  }

  /*
   * Method to return the String color name given an RGB reading and the location type.
   */
  public String determineColor(int[] currentRGB, String location) {

    double[] colorDistances;
    int i; // this will be the index of the determined color
    int colorIndex = 0;
    // compute distances for each color for each R,G,B values
    colorDistances = distanceToIdealRGB(currentRGB, location);
    //System.out.println("Distances: [" +colorDistances[0]+", "+colorDistances[1]+", "+colorDistances[2]+"]");

    // determine "closest" color
    double currMin = colorDistances[0];
    for(i=0; i < colorDistances.length; i++) {
      if(colorDistances[i] < currMin) {
        currMin = colorDistances[i];
        colorIndex = i;
      }
    }

    return COLOR_NAMES[colorIndex];

  }

  /*
   * Returns array of distances for each R,G,B for a provided sample color reading.
   * Needs to have location type specified ie: 'BIG' or 'SMALL' lab room
   */
  public double[] distanceToIdealRGB(int [] currentRGB, String location) {
    double errorDistance=0; //updated inside the if statements.   
    double d;
    String colorName;
    int[] ideal_rgb_color;
    double red_squared;
    double green_squared;
    double blue_squared;
    double[] colorDistances = new double[COLOR_NAMES.length];           //COLOR_NAMES.length = 4

    // BIG room
    if(location.equals("BIG")) {

      // Compute distance from all 4 possible colors
      for(int i=0; i < COLOR_NAMES.length; i++) {
        colorName = COLOR_NAMES[i];
        if(colorName.equals("BLUE")) {
          ideal_rgb_color = BIG_ROOM_BLUE;
        } else if(colorName.equals("YELLOW")) {
          ideal_rgb_color = BIG_ROOM_YELLOW;
        } else if(colorName.equals("GREEN")) {
          ideal_rgb_color = BIG_ROOM_GREEN;
        } else if(colorName.equals("ORANGE")) { 
          ideal_rgb_color = BIG_ROOM_ORANGE;
        } else {
          ideal_rgb_color = BIG_ROOM_BLACK;
        }

        // compute deviation for each RGB
        red_squared = Math.pow((currentRGB[0] - ideal_rgb_color[0]), 2);
        green_squared = Math.pow((currentRGB[1] - ideal_rgb_color[1]), 2);
        blue_squared = Math.pow((currentRGB[2] - ideal_rgb_color[2]), 2);

        // compute distance
        d = Math.sqrt(red_squared + green_squared + blue_squared);

        // store in local array of distances to be able to determine the "closest" color
        colorDistances[i] = d;

      }

    } else { // SMALL
      
      // Compute distance from all 4 possible colors
      for(int i=0; i < COLOR_NAMES.length; i++) {
        colorName = COLOR_NAMES[i];
        if(colorName.equals("BLUE")) {
          ideal_rgb_color = SMALL_ROOM_BLUE;
        } else if(colorName.equals("YELLOW")) {
          ideal_rgb_color = SMALL_ROOM_YELLOW;
        } else if(colorName.equals("GREEN")) {
          ideal_rgb_color = SMALL_ROOM_GREEN;
        } else if(colorName.equals("ORANGE")) { 
          ideal_rgb_color = SMALL_ROOM_ORANGE;
        } else {
          ideal_rgb_color = SMALL_ROOM_BLACK;
        }

        // compute deviation for each RGB
        red_squared = Math.pow((currentRGB[0] - ideal_rgb_color[0]), 2);
        green_squared = Math.pow((currentRGB[1] - ideal_rgb_color[1]), 2);
        blue_squared = Math.pow((currentRGB[2] - ideal_rgb_color[2]), 2);

        // compute distance
        d = Math.sqrt(red_squared + green_squared + blue_squared);

        // store in local array of distances to be able to determine the "closest" color
        colorDistances[i] = d;

      }
      
    }

    return colorDistances;

  }

  /**
   * Procedure to follow when sensor detects ring
   * @return void 
   * Note:can change return to boolean
   * @throws InterruptedException 
   */
  public void objectDetected() {
    String ringColor;
    
    lcd.drawString("Object Detected.", 0, 1);
      
    // Update global variables
    NUM_RINGS_DETECTED += 1;
    HAS_DETECTED_OBJECT = true;

    // stop robot
    nav.stopMotors();

    //beep twice
    Sound.beep();
    Sound.beep();

    //move towards object closer, do not use the moveStraightFor() method
    leftMotor.setSpeed(MOTOR_LOW);
    rightMotor.setSpeed(MOTOR_LOW);
    Main.sleepFor(GENERAL_SLEEP);
    leftMotor.rotate(nav.convertDistance(2), true);
    rightMotor.rotate(nav.convertDistance(2), false);

    // Determine color of ring and print to LCD screen
    ringColor = analyzeColor(); 
    
    // Append to final ring colors output
    RINGS_DETECTED_STRING += ringColor+" ";
    
    //wait for 10 seconds 
    Main.sleepFor(10000);
    
    // Update global flag for moveStraightFor() method
    HAS_DETECTED_OBJECT = false;

    // resume navigation in moveStraightFor() method...

  }

  /*
   * Method return an array containing the RGB reading from the color sensor
   */
  public static float[] getRGBdata() {
    float data[] = new float[3];   
    data[0] = RGBData[0];
    data[1] =  RGBData[1] ;
    data[2] = RGBData[2];
    return data;
  }
  
}