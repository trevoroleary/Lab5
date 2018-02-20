package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class is used to Navigate through a series of way points
 */

public class Navigation extends Thread {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final int MOTOR_STRAIGHT = 80;
	public static final double WHEEL_RAD = 2.12;
	public static final double TRACK = 16.05;
	private boolean isNavigating;
	private Odometer odometer;
	public static int pointcounter;
	public lightLocalizer lightLocalizer;

	// angle toward destination
	private double Theta;

	// current position
	private double currentX;
	private double currentY;
	private double currentTheta;

	// difference between current position and destination position
	private double dX; //
	private double dY; //
	// distance between current position and destination position
	private double distance;

	/**
	 * This is the class constructor
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * 
	 */
	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, lightLocalizer lightLocalizer) {



		this.odometer = odo;
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.lightLocalizer = lightLocalizer;

		try {
			this.odometer = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	/**
	 * This method is used to calculate the distance to map point given
	 * Cartesian coordinates x and y
	 * 
	 * @param x
	 * @param y
	 * 
	 */
	public void travelTo(double x, double y) {
		isNavigating = true;

		x = x * Lab5.TILE_SIZE;
		y = y * Lab5.TILE_SIZE;

		// get current position
		currentX = odometer.getX();
		currentY = odometer.getY();

		// compute the difference
		dX = x - currentX;
		dY = y - currentY;

		distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
		Theta = Math.toDegrees((Math.atan2(dX, dY))); // convert from radius to
														// degree

		// rotate toward destination

		turnTo(Theta, true);

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
	public void turnTo(double theta, boolean localizer) {
		boolean turnleft = false;
		double currTheta = odometer.getTheta();
		double angle = theta - currTheta;

		if (angle < -180) {
			angle = angle + 360;
		}
		if (angle > 180) {
			angle = angle - 360;
		}
		if (angle < 0) {
			turnleft = true;
			angle = Math.abs(angle);
		} else {
			turnleft = false;
		}

		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);

		if (turnleft) {
			leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, angle), true);
			rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, angle), false);
		} else {
			leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, angle), true);
			rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, angle), false);
		}

		currentTheta = theta;
		if(localizer)
			lightLocalizer.correctLocation();

	}

	/**
	 * This method determines whether another thread has called travelTo and
	 * turnTo methods or not
	 * 
	 * @return
	 */

	public boolean isNavigating() {
		return isNavigating;
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of
	 * each wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of an angle to the total rotation of
	 * each wheel need to cover that distance.
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
