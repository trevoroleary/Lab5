package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;

//import Odometer.Odometer;
//import Odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to Navigate through a series of way points
 */

public class Navigation extends Thread {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final int MOTOR_STRAIGHT = 80;
	private static final int MOTOR_ROTATE = 70;
	public static final double WHEEL_RAD = 2.12;
	public static final double TRACK = 16.05;
	private boolean isNavigating;
	private Odometer odometer;
	public static int pointcounter;

	// destination position
	private double x;
	private double y;

	// angle toward destination
	private double Theta;

	// current position
	private double currentX;
	private double currentY;
	private double currentTheta;

	// difference between current position and destination position
	private double dX; //
	private double dY; //
	private double dTheta;

	// distance between current position and destination position
	private double distance;

	/**
	 * This is the class constructor
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * 
	 */
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		try {
			this.odometer = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}

	}

	/**
	 * This method is used to calculate the distance to map point given Cartesian
	 * coordinates x and y
	 * 
	 * @param x
	 * @param y
	 * 
	 */
	public void travelTo(double x, double y) {
		isNavigating = true;

		this.x = x;
		this.y = y;

		// get current position
		currentX = odometer.getX();
		currentY = odometer.getY();

		// compute the difference
		dX = x - currentX;
		dY = y - currentY;

		distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
		Theta = (Math.atan2(dX, dY)) * 180 / Math.PI; // convert from radius to degree

		// rotate toward destination
		turnTo(Theta);

		// move straight
		rightMotor.setSpeed(MOTOR_STRAIGHT);
		leftMotor.setSpeed(MOTOR_STRAIGHT);
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);

		isNavigating = false;
	}

	/**
	 * This method is used to make sure robot rotates at the minimum angle when
	 * traveling to next waypoint
	 * 
	 */
	public void turnTo(double Theta) {
		currentTheta = odometer.getTheta(); // currentTheta is in degree
		dTheta = Theta - currentTheta;

		// avoid maximal angle turn
		if (dTheta > 180) {
			dTheta = 360 - dTheta;
		} else if (dTheta < -180) {
			dTheta = 360 + dTheta;
		}
		// turn minTheta degree
		leftMotor.setSpeed(MOTOR_ROTATE);
		rightMotor.setSpeed(MOTOR_ROTATE);
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dTheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dTheta), false);

	}
	
	public void turn(double Theta) {
		currentTheta = odometer.getTheta(); // currentTheta is in degree
		dTheta = Theta - currentTheta;

		leftMotor.setSpeed(MOTOR_ROTATE);
		rightMotor.setSpeed(MOTOR_ROTATE);
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, Theta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, Theta), true);

	}

	/**
	 * This method determines whether another thread has called travelTo and turnTo
	 * methods or not
	 * 
	 * @return
	 */

	public boolean isNavigating() {
		return isNavigating;
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of an angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
