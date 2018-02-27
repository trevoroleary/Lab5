package ca.mcgill.ecse211.search;

import ca.mcgill.ecse211.color.colorSensor;
import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.Navigation;
import ca.mcgill.ecse211.lab5.USLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import java.util.Timer;
import java.util.TimerTask;

public class Search {

	private EV3LargeRegulatedMotor sensorMotor;
	private final int[] LL;
	private final int[] UR;

	private colorSensor sensor;
	private Odometer odometer;
	private Navigation navigator;
	private USLocalizer USData;
	private boolean upTrue = true;
	private boolean isRight = false;
	private int sampleSize = 25;
	private float avgData;

	private float blockInFront = 50;
	private boolean navigating = false;
	private boolean foundSomething = false;

	public Search(int[] LL, int[] UR, colorSensor colorSensor, Odometer odometer, USLocalizer USData,
			Navigation navigator, EV3LargeRegulatedMotor sensorMotor) {
		this.sensorMotor = sensorMotor;
		this.LL = LL;
		this.UR = UR;
		this.sensor = colorSensor;
		this.odometer = odometer;
		this.navigator = navigator;
		this.USData = USData;

		/**
		 * This method initiates searching should only be called one the robot
		 * is in the LL of the search area.
		 * 
		 */

	}

	public void beginSearch() {

		while (!foundSomething) {
			if(colorSensor.isCorrect()) {
				foundSomething = true;
			} else {
			goUp();
			}
		}
		navigator.goToUpperRight(UR);
	}

	/**
	 * This method checks in front and 45deg on either side of the robot for
	 * blocks
	 * 
	 * @return returns an array of size 3. If all elements are 0 there is
	 *         nothing on either side of the robot or infront if i0 is 1 there
	 *         is something on the left, if i1 is 1 there is something in front,
	 *         if i2 is 1 there is something on the right
	 */

	public void checkSide() {
		int[] returnVals = new int[] { 0, 0, 0 };

		avgData = USData.deriData();
		LCD.drawString("US:" + avgData, 0, 6, false);

		if (avgData < 25) {
			returnVals[1] = 1;
			// TODO IDENTIFY BLOCK WITH FIND AHEAD
			// AVOID
			move(avgData - 5);
			navigator.turnTo(-90, false);
			move((Lab5.TRACK + 10) / 2);

			navigator.turnTo(90, false);
			move((Lab5.TRACK + 10) / 2);

			navigator.turnTo(90, false);
			move((Lab5.TRACK + 10) / 2);

			navigator.turnTo(-90, false);
			move((Lab5.TRACK + 10) / 2);
		}

		navigator.turn(45);
		avgData = USData.deriData();

		if (avgData < 25) {
			returnVals[2] = 1;

			move(avgData - 5);
			moveBack(avgData - 5);
		}

		else {

			navigator.turn(-90);
			avgData = USData.deriData();

			if (avgData < 25) {
				returnVals[0] = 1;
				// TODO IDENTIFY BLOCK WITH FIND AHEAD
				move(avgData - 5);
				moveBack(avgData - 5);

			}

			else {
				if (upTrue) {
					navigator.turnTo(0, false);
				} else {
					navigator.turnTo(180, false);
				}
				nextLine();

			}
		}

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
	 * This method increments forward one tile length If the robot it at the
	 * edge of the search area it calls nextSection instead
	 * 
	 * TODO while this method is moving forward, object avoidance needs to be
	 * implemented
	 */
	public void nextLine() {
		int x = (int) ((odometer.getX() / Lab5.TILE_SIZE) + 0.5);
		int y = (int) ((odometer.getY() / Lab5.TILE_SIZE) + 0.5);
		int theta = (int) (odometer.nearestHeading() + 0.5);

		if ((y == UR[1] && theta == 0) || (y == LL[1] && theta == 180)) {
			nextSection();
		} else {
			if (upTrue) {
				navigator.travelTo(x, y + 1);
				navigator.turnTo(0, true);
				checkSide();
			} else {
				navigator.travelTo(x, y - 1);
				navigator.turnTo(180, true);
				checkSide();
			}
		}
	}

	/**
	 * this method increments the robots x value so that it can start the next
	 * state of its searching in the search area This method also orients the
	 * robot to face inwards so an additional line can be searched
	 */
	public void nextSection() {
		int x = (int) ((odometer.getX() / Lab5.TILE_SIZE) + 0.5);
		int y = (int) ((odometer.getY() / Lab5.TILE_SIZE) + 0.5);
		navigator.travelTo(x, y);
		navigator.travelTo(x + 1, y);
		if (y == UR[1]) {
			upTrue = false;
			navigator.turnTo(180, true);
		} else {
			upTrue = true;
			navigator.turnTo(0, true);
		}
	}

	public void move(double distance) {


		Odometer.rightMotor.setSpeed(100);
		Odometer.leftMotor.setSpeed(100);
		Odometer.leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), true);
		Odometer.rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), false);

	}

	public void moveBack(double distance) {
		// AVOID

		Odometer.rightMotor.setSpeed(100);
		Odometer.leftMotor.setSpeed(100);
		Odometer.leftMotor.rotate(-convertDistance(Lab5.WHEEL_RAD, distance), true);
		Odometer.rightMotor.rotate(-convertDistance(Lab5.WHEEL_RAD, distance), false);

	}

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

	public void sensorRight() {
		if (!isRight){
		isRight = true;
		sensorMotor.rotate(-90);
		}

	}

	public void sensorForward() {
		if(isRight){
		isRight = false;
		sensorMotor.rotate(90);
		}
	}

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
			LCD.drawString("US:" + avgData, 0, 6, false);
			//if (USData.deriData() > blockInFront) {
			if(USData.getFilteredData() < 30) {
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
