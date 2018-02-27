package ca.mcgill.ecse211.lab5;


/**
 * THsi class is the Ultra sonic localization, it roughly aligns the robot up for the next stages in localization
 */

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;
//import ca.mcgill.ecse211.lab4;

public class USLocalizer {
	private static final int MOTOR_ROTATE = 70;
	// private double dTheta;

	public Navigation navigation;
	private double wallDistance = 30;
	private double wallError = 3;
	private final double TILE_SIZE = 30.48;
	private float[] deriDataArr = new float[10];
	// public static final double WHEEL_RAD = 2.12;
	// public static final double TRACK = 16.05;
	private float bAverage = 0;
	private float fAverage = 0;
	private Odometer odometer;
	private float[] usData;
	private int counter;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private boolean isRisingEdge;
	private SampleProvider usDistance;
	private boolean isFirst = true;

	// Navigation navigation = new Navigation(leftMotor, rightMotor);
	
	public USLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			SampleProvider usDistance, Navigation navigation) {

		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usDistance = usDistance;
		this.usData = new float[usDistance.sampleSize()];
		// this.usData = usData;
		this.navigation = navigation;

		leftMotor.setSpeed(MOTOR_ROTATE);
		rightMotor.setSpeed(MOTOR_ROTATE);

	}
/**
 *  calls the rising or falling edge depending on the start corner.
 * @param startCorner start corner is an int (0 - 3) that indicates which corner the robot starts it
 * 			this allows the localiztion to set the coordinate properly
 * @return void
 */
	public void localize(int startCorner) {

		if (getFilteredData() < wallDistance) {
			localizeRisingEdge(startCorner);
		} else {
			localizeFallingEdge(startCorner);
		}
	}
/**
 * this method uses the ultrasonic sensor to Localize and end up looking forward looking at 0 degrees
 * @param startCorner allows the set its location and theta properly at the starting corner (int from 0-3)
 */
	public void localizeRisingEdge(int startCorner) {

		double thetaA;
		double thetaB;
		double thetaRotation;

		while (getFilteredData() > wallDistance - wallError) {
			// rotate counter clockwise
			leftMotor.backward();
			rightMotor.forward();
		}

		while (getFilteredData() < wallDistance) {
			// rotate counter clockwise
			leftMotor.backward();
			rightMotor.forward();
		}

		// Without wall in front of robot, stop motors and latch angle
		thetaA = odometer.getTheta();

		leftMotor.stop(true);
		rightMotor.stop(false);

		while (getFilteredData() > wallDistance - wallError) {
			// rotate clockwise
			leftMotor.forward();
			rightMotor.backward();
		}

		while (getFilteredData() < wallDistance) {
			// rotate clockwise
			leftMotor.forward();
			rightMotor.backward();
		}

		thetaB = odometer.getTheta();

		leftMotor.stop(true);
		rightMotor.stop(false);

		thetaRotation = 225 - (thetaA + thetaB) / 2.0;
		navigation.turnTo(thetaRotation, false);

		if (startCorner == 0)
			odometer.setXYT(1, 1, 0);
		if (startCorner == 1)
			odometer.setXYT(7 * TILE_SIZE, 1, 270);
		if (startCorner == 2)
			odometer.setXYT(7 * TILE_SIZE, 7 * TILE_SIZE, 180);
		if (startCorner == 3)
			odometer.setXYT(1, 7 * TILE_SIZE , 90);
		Sound.beep();
	}
/**
 * this method uses the ultrasonic  to look at 0 degrees using the falling algorithm
 * @param startCorner gives this method the starting corner (int from 1-3)
 * @return void
 */
	public void localizeFallingEdge(int startCorner) {

		double thetaA;
		double thetaB;
		double thetaRotation;

		while (getFilteredData() < wallDistance + wallError) {
			leftMotor.forward();
			rightMotor.backward();
		}

		while (getFilteredData() > wallDistance) {
			leftMotor.forward();
			rightMotor.backward();
		}

		thetaA = odometer.getTheta();

		leftMotor.stop(true);
		rightMotor.stop(false);

		while (getFilteredData() < wallDistance + wallError) {
			leftMotor.backward();
			rightMotor.forward();
		}

		while (getFilteredData() > wallDistance) {
			leftMotor.backward();
			rightMotor.forward();
		}

		thetaB = odometer.getTheta();

		leftMotor.stop(true);
		rightMotor.stop(false);

		if (thetaA < thetaB) {
			thetaRotation = 225 - ((thetaA + thetaB) / 2.0);
			navigation.turnTo(thetaRotation, false);
		} else {
			thetaRotation = (thetaA + thetaB) / 2.0 + 180;
			navigation.turnTo(thetaRotation, false);
		}

		if (startCorner == 0)
			odometer.setXYT(0, 0, 0);
		if (startCorner == 1)
			odometer.setXYT(7 * TILE_SIZE, 0, 270);
		if (startCorner == 2)
			odometer.setXYT(7 * TILE_SIZE, 7 * TILE_SIZE, 180);
		if (startCorner == 3)
			odometer.setXYT(0, 7 * TILE_SIZE, 90);

	}

	public int getFilteredData() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}
	

	public float deriData() {

		if (isFirst) {
			isFirst = false;
			for (int i = 0; i < 10; i++) {

				usData[0] = getFilteredData();

				if ((int) usData[0] > 200) {
					usData[0] = 200;
				}

				deriDataArr[i] = usData[0];

				if (i < 5) {
					bAverage = +usData[0] / 5;
				} else {
					fAverage = +usData[0] / 5;
				}
			}

		} else {
			usData[0] = getFilteredData();

			if ((int) usData[0] > 200) {
				usData[0] = 200;
			}

			bAverage = +(deriDataArr[counter % 10] - deriDataArr[(counter + 5) % 10]) / 5;

			fAverage = -deriDataArr[(counter + 5) % 10] / 5;
			deriDataArr[counter % 10] = usData[0];
			fAverage = +deriDataArr[counter % 10] / 5;
		}
		counter++;

		return (bAverage - fAverage);

	}

}
