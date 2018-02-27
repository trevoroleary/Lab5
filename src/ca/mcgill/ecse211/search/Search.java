package ca.mcgill.ecse211.search;

import ca.mcgill.ecse211.color.colorSensor;
import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.Navigation;
import ca.mcgill.ecse211.lab5.USLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Search {

	private EV3LargeRegulatedMotor sensorMotor;
	private final int[] LL;
	private final int[] UR;

	private Odometer odometer;
	private Navigation navigator;
	private USLocalizer USData;
	private boolean isRight = false;
	private boolean navigating = false;
	private boolean foundSomething = false;

	public Search(int[] LL, int[] UR, colorSensor colorSensor, Odometer odometer, USLocalizer USData,
			Navigation navigator, EV3LargeRegulatedMotor sensorMotor) {
		this.sensorMotor = sensorMotor;
		this.LL = LL;
		this.UR = UR;
		this.odometer = odometer;
		this.navigator = navigator;
		this.USData = USData;

		/**
		 * This method initiates searching should only be called one the robot
		 * is in the LL of the search area.
		 * 
		 */

	}

	/**
	 * This method is the main method for Calling the helper methods in the
	 * Search class it also stops everything if the once and object of the right
	 * color is found. this method should only be called once the both is at LL.
	 * 
	 * @param
	 * @return void
	 */

	public void beginSearch() {

		while (!foundSomething) {
			if (colorSensor.targetColor == colorSensor.sensorColor) {
				// TODO this never becomes true ????? HELP DUDE (even though the
				// robot beeps twice after seeing the right color
				foundSomething = true;
				break;
			} else {
				goUp();
			}
		}
		navigator.goToUpperRight(UR);
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

	/**
	 * this method is what is used once the US sensor finds and object. this
	 * method goes close to block and scans block using the color sensor
	 * 
	 * @return void
	 * @param
	 */

	public void getBlock() {
		navigator.turn(90);
		sensorForward();

		navigating = true;
		Odometer.leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, Lab5.TILE_SIZE), true);
		Odometer.rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, Lab5.TILE_SIZE), true);

		while (navigating) {
			if (USData.getFilteredData() < 4) {
				Odometer.leftMotor.stop(true);
				Odometer.rightMotor.stop(false);
				navigating = false;
			}
			if (!Odometer.leftMotor.isMoving() && !Odometer.rightMotor.isMoving()) {
				navigating = false;
			}
			if (colorSensor.seeColor()) {
				Odometer.leftMotor.stop(true);
				Odometer.rightMotor.stop(false);
				navigating = false;
				LCD.drawString("Found" + colorSensor.getResponse(), 0, 6, false);
				foundSomething = true;

			}
		}

		navigator.travelToNearestEdge();

	}

	/**
	 * the method makes sure that the US sensor is looking right and if it is
	 * not it turns it around.
	 * 
	 * @return void
	 * @param
	 */
	public void sensorRight() {
		if (!isRight) {
			isRight = true;
			sensorMotor.rotate(-90);
		}

	}

	/**
	 * the method makes sure that the US sensor is looking forward and if it is
	 * not it turns it around.
	 * 
	 * @return void
	 * @param
	 */
	public void sensorForward() {
		if (isRight) {
			isRight = false;
			sensorMotor.rotate(90);
		}
	}

	/**
	 * this method is what increments the robot forward localizing at every
	 * point as it circles the parameter. it also makes sure the sensor looks 
	 */
	public void goUp() {
		sensorRight();

		int x = (int) ((odometer.getX() / Lab5.TILE_SIZE) + 0.5);
		int y = (int) ((odometer.getY() / Lab5.TILE_SIZE) + 0.5);

		double theta = odometer.nearestHeading();

		navigator.travelTo(x, y);
		navigator.turnTo(theta, true);

		if (y == UR[1] && theta == 0)
			navigator.turnTo(90, true);
		else if (x == UR[0] && theta == 90)
			navigator.turnTo(180, true);
		else if (y == LL[1] && theta == 180)
			navigator.turnTo(270, true);
		else if (x == LL[0] && theta == 270) {
			navigator.goToUpperRight(UR);
			foundSomething = true;
		}

		Odometer.rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, Lab5.TILE_SIZE), true);
		Odometer.leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, Lab5.TILE_SIZE), true);
		navigating = true;

		while (navigating) {
			if (USData.getFilteredData() < 30) {
				Odometer.rightMotor.stop(true);
				Odometer.leftMotor.stop(true);
				navigating = false;
				getBlock();

			} else if (!Odometer.rightMotor.isMoving() && !Odometer.leftMotor.isMoving()) {
				navigating = false;
			}
		}
	}

}
