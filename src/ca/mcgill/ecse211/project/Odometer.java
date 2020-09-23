package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Main.sleepFor;
import static ca.mcgill.ecse211.project.Resources.*;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Odometer extends Thread implements Runnable {

  /**
   * The x-axis position in cm.
   */
  private volatile double x;

  /**
   * The y-axis position in cm.
   */
  private volatile double y; // y-axis position

  /**
   * The orientation in degrees.
   */
  public volatile double theta; // Head angle

  /*
   * The total summed up angle displacement
   */
  private volatile double totalTheta;



  /**
   * The (x, y, theta) position as an array.
   */
  private double[] position;

  // Thread control tools
  /**
   * Fair lock for concurrent writing.
   */
  private static Lock lock = new ReentrantLock(true);

  /**
   * Indicates if a thread is trying to reset any position parameters.
   */
  private volatile boolean isResetting = false;

  /**
   * Lets other threads know that a reset operation is over.
   */
  private Condition doneResetting = lock.newCondition();

  private static Odometer odo; // Returned as singleton

  // Motor-related variables
  int lastTachoL = 0;               
  int lastTachoR = 0;               

  /**
   * The odometer update period in ms.
   */ 

  private static final long ODOMETER_PERIOD = 25;


  /**
   * This is the default constructor of this class. It initiates all motors and variables once. It cannot be accessed
   * externally.
   */
  public Odometer() {
    setXyt(15.24, 15.24, 0);
    lastTachoL = leftMotor.getTachoCount();                
    lastTachoR = rightMotor.getTachoCount();               

  }


  /**
   * Returns the Odometer Object. Use this method to obtain an instance of Odometer.
   * 
   * @return the Odometer Object
   */
  public static synchronized Odometer getOdometer() {
    if (odo == null) {
      odo = new Odometer();
    }

    return odo;
  }

  /**
   * This method is where the logic for the odometer will run.
   */
  public void run() {

    long updateStart;
    long updateDuration;
    double tempTheta;        
    while (true) {
      tempTheta = odo.getXyt()[2] * (Math.PI/180);
      updateStart = System.currentTimeMillis();

      int nowTachoL = leftMotor.getTachoCount(); // get tacho counts
      int nowTachoR = rightMotor.getTachoCount();
      // compute L and R wheel displacements
      double distL = Math.PI * WHEEL_RADIUS * (nowTachoL - lastTachoL) / 180;
      double distR = Math.PI * WHEEL_RADIUS * (nowTachoR - lastTachoR) / 180;
      lastTachoL = nowTachoL; // save tacho counts for next iteration
      lastTachoR = nowTachoR;
      double deltaD = 0.5 * (distL + distR); // compute vehicle displacement
      double deltaT = (distL - distR) / BASE_WIDTH; // compute change in heading
      tempTheta = tempTheta +  deltaT;            

      double dX = deltaD * Math.sin(tempTheta); // compute x component of displacement
      double dY = deltaD * Math.cos(tempTheta); // compute y component of displacement

      odo.update(dX,dY,deltaT * (180/Math.PI));          

      // this ensures that the odometer only runs once every period
      updateDuration = System.currentTimeMillis() - updateStart;
      if (updateDuration < ODOMETER_PERIOD) {
        sleepFor(ODOMETER_PERIOD - updateDuration);            
      }
    }           //end of while loop
  } // end of run method

  // IT IS NOT NECESSARY TO MODIFY ANYTHING BELOW THIS LINE

  /**
   * Returns the Odometer data.
   * 
   * <p>
   * Writes the current position and orientation of the robot onto the odoData array.
   * {@code odoData[0] = x, odoData[1] = y; odoData[2] = theta;}
   * 
   * @return the odometer data.
   */
  public double[] getXyt() {
    double[] position = new double[4];
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;
      position[3] = totalTheta;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return position;
  }

  /**
   * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. Useful for odometry.
   * 
   * @param dx the change in x
   * @param dy the change in y
   * @param dtheta the change in theta
   */
  public void update(double dx, double dy, double dtheta) {
    lock.lock();
    isResetting = true;
    try {
      x += dx;
      y += dy;
      theta = (theta + (360 + dtheta) % 360) % 360;
      totalTheta += dtheta;
      isResetting = false;
      doneResetting.signalAll(); // Let the other threads know we are done resetting
    } finally {
      lock.unlock(); 
    }

  }

  /**
   * Overrides the values of x, y and theta. Use for odometry correction.
   * 
   * @param x the value of x
   * @param y the value of y
   * @param theta the value of theta in degrees
   */
  public void setXyt(double x, double y, double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      this.y = y;
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites x. Use for odometry correction.
   * 
   * @param x the value of x
   */
  public void setX(double x) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites y. Use for odometry correction.
   * 
   * @param y the value of y
   */
  public void setY(double y) {
    lock.lock();
    isResetting = true;
    try {
      this.y = y;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites theta. Use for odometry correction.
   * 
   * @param theta the value of theta
   */
  public void setTheta(double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }
  public void resetOdo() {
    Main.sleepFor(40);
    lock.lock();
    odometer.setTheta(0);
    lock.unlock(); 
  }

}
