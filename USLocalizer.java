package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
//import ca.mcgill.ecse211.lab4;

public class USLocalizer {
	private static final int MOTOR_ROTATE = 70;
	// private double dTheta;

	public Navigation navigation;
	private double wallDistance = 30;
	private double wallError = 3;
	private final double TILE_SIZE = 30.48;

	// public static final double WHEEL_RAD = 2.12;
	// public static final double TRACK = 16.05;

	private Odometer odometer;
	private float[] usData;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private boolean isRisingEdge;
	private SampleProvider usDistance;
	private boolean turn = false;

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

	public void localize(int startCorner) {


		if (getFilteredData() < wallDistance) {
			localizeRisingEdge(startCorner);
		} else {
			localizeFallingEdge(startCorner);
		}
	}

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
		rightMotor.stop();

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
		rightMotor.stop();

		thetaRotation = 225 - (thetaA + thetaB) / 2.0;
		navigation.turnTo(thetaRotation, false);

		if(startCorner == 0)
			odometer.setXYT(0, 0, 0);
		if(startCorner == 1)
			odometer.setXYT(7*TILE_SIZE - 1, 0, 270);
		if(startCorner == 2)
			odometer.setXYT(7*TILE_SIZE - 1,7*TILE_SIZE - 1,180);
		if(startCorner == 3)
			odometer.setXYT(0, 7*TILE_SIZE - 1, 90);
		

	}

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
		rightMotor.stop();

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
		rightMotor.stop();

		if (thetaA < thetaB) {
			thetaRotation = 225 - ((thetaA + thetaB) / 2.0) ;
			navigation.turnTo(thetaRotation, false);
		} else {
			thetaRotation = (thetaA + thetaB) / 2.0 + 180;
			navigation.turnTo(thetaRotation, false);
		}

		if(startCorner == 0)
			odometer.setXYT(0, 0, 0);
		if(startCorner == 1)
			odometer.setXYT(7*TILE_SIZE, 0, 270);
		if(startCorner == 2)
			odometer.setXYT(7*TILE_SIZE,7*TILE_SIZE,180);
		if(startCorner == 3)
			odometer.setXYT(0, 7*TILE_SIZE, 90);

	}

	private int getFilteredData() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}
	
	public int getAvgData(int setSize) {
		int avgData = 0;
		
		for(int i = 0; i < setSize; i++) {
			
			usDistance.fetchSample(usData, 0);
			
				if(((int) (usData[0]*100) ) > 200) {
					avgData =+ 200/setSize;
				}
				else
					avgData =+ (int) (usData[0]*100)/setSize;
		}
		return avgData;
	}
	
}
