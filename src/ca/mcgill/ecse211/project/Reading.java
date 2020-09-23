package ca.mcgill.ecse211.project;

import java.util.ArrayList;

public class Reading implements Comparable {

  private double deltaAngle;
  private int distance;

  public Reading(double deltaAngle, int distance) {
    this.deltaAngle = deltaAngle;
    this.distance = distance;
  }

  public double getDeltaAngle() {
    return this.deltaAngle;
  }

  public int getDistance() {
    return this.distance;
  }

  public void setDeltaAngle(double deltaAngle) {
    this.deltaAngle = deltaAngle;
  }
  
  public void setDistances(int distance) {
    this.distance = distance;
  }

  @Override
  public int compareTo(Object o) {
    int compared=((Reading) o).getDistance();
    /* For Ascending order*/
    return this.distance-compared;
  }

}
