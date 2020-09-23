package ca.mcgill.ecse211.project;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

public class ColorSensorCalibration {

//  public static final EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
  public static final TextLCD TEXT_LCD = LocalEV3.get().getTextLCD();

  private ArrayList<ArrayList<Double>> colorReadings = new ArrayList<ArrayList<Double>>();
  private float[] colorData = new float[50];
  static File file;
  String[] testData = new String[6];
  
  public void calibrate()  {
    int testIndex = 0;
  
    colorReadings.add(new ArrayList<Double>());
    colorReadings.add(new ArrayList<Double>());
    colorReadings.add(new ArrayList<Double>());
    
    while(testIndex < 5) {
     

      TEXT_LCD.drawString("Calibration #"+testIndex, 0, 0);
      TEXT_LCD.drawString("Press btn to calibrate.", 0, 1);
      Button.waitForAnyPress();
      TEXT_LCD.clear();
      ArrayList<Double> rgbReading = getColorData();
      
      colorReadings.get(0).add(new Double(rgbReading.get(0)));
      colorReadings.get(1).add(new Double(rgbReading.get(1)));
      colorReadings.get(2).add(new Double(rgbReading.get(2)));
      
      System.out.println("Reading: " + rgbReading);
      
    testData[testIndex] = (testIndex+1) + ", " + rgbReading.get(0).toString()+ ", " +
   		               rgbReading.get(1).toString()+", "+ rgbReading.get(2).toString() ;

    testIndex++;
      
    }
    
    createCSV(testData);
    
    System.out.println("Output: " + colorReadings);
   
  }


  public  ArrayList<Double> getColorData() {
    colorData =  ColorSensor.getRGBdata();
    ArrayList<Double> out = new ArrayList<Double>();
    // line below does not work, simply need to get RGB from color sensor as an array like: [R, G, B]
    // works now
    out.add(new Double(colorData[0]));
    out.add(new Double(colorData[1]));
    out.add(new Double(colorData[2]));
    return out;
  }
  
  /**
   *  checks if file has been created succesfully
   * @return
   */
  public static File createfile() {
	  return new File("color_character.csv");
  }

//-------------------------------Setting up a CSV File----------------------------------------//
  
  public static void createCSV(String[] arr) {
  file = createfile();
  try (BufferedWriter csvWriter = new BufferedWriter(new FileWriter(file))) {
		if (! file.exists()) {
			file.createNewFile();
		}
		csvWriter.append("Test # ");
		csvWriter.append(", ");
		csvWriter.append("RED");
		csvWriter.append(", ");
		csvWriter.append("GREEN");
		csvWriter.append(", ");
		csvWriter.append("BLUE");
		csvWriter.newLine();
		
		for ( String data : arr) {
			csvWriter.append(data);
			csvWriter.newLine();
		}
		
		csvWriter.flush();

	} catch (IOException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
  }
//--------------------------------------------------------------------------------------------//

}
