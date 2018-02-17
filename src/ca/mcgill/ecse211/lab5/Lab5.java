package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab5.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor; //MOTOR for sensor
import lejos.hardware.sensor.*;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;

public class Lab5 {

/**
 * This class is used as designated entry point to run program
 * 
 */
// Motor Objects, and Robot related parameters

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static SensorModes RSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	private static SampleProvider RColor = RSensor.getMode("Red");
	private static float[] RData = new float[RColor.sampleSize()];
	
	private static SensorModes LSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	private static SampleProvider LColor = LSensor.getMode("Red");
	private static float[] LData = new float[LColor.sampleSize()];
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S1");
	//private static final Port lightPort = LocalEV3.get().getPort("S1");

	private static boolean isRisingEdge = true;
	private static int startCorner;
	

	public static final double WHEEL_RAD = 2.12;//2.12
	public static final double TRACK = 10.2;//
	public static final double TILE_SIZE = 30.48;

	static SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
	static SampleProvider usDistance = ultrasonicSensor.getMode("Distance");
	static float[] usData = new float[usDistance.sampleSize()];

	/**
	 * This is the main method
	 * 
	 * @throws OdometerExceptions
	 */
	
	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

		Display odometrydisplay = new Display(lcd); // No need to change
		Navigation navigation = new Navigation(leftMotor, rightMotor);

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Rising | Falling", 0, 2);
			lcd.drawString("Edge	  | Edge", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT)
			isRisingEdge = true;
		else 
			isRisingEdge = false;

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("       0        ", 0, 0);
			lcd.drawString("       Up       ", 0, 1);
			lcd.drawString("3 Left   Right 1", 0, 2);
			lcd.drawString("      Down      ", 0, 3);
			lcd.drawString("       2        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (
				buttonChoice != Button.ID_LEFT && 
				buttonChoice != Button.ID_RIGHT && 
				buttonChoice != Button.ID_DOWN && 
				buttonChoice != Button.ID_UP
				);

		if (buttonChoice == Button.ID_RIGHT)
			startCorner = 1;
		if (buttonChoice == Button.ID_DOWN)
			startCorner = 2;
		if (buttonChoice == Button.ID_LEFT)
			startCorner = 3;
		if (buttonChoice == Button.ID_UP)
			startCorner = 0;
		
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		Thread ododisplayThread = new Thread(odometrydisplay);
		ododisplayThread.start();

		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, isRisingEdge, usDistance, navigation);
		LightLocalizer LightLocalizer = new LightLocalizer(odometer, leftMotor, rightMotor, RColor, LColor, RData, LData);

		/*
		// UltraSonic Localization
		USLocalizer.localize(startCorner);

		//while (Button.waitForAnyPress() != Button.ID_ENTER);

		LightLocalizer.localization(startCorner);
		
		
		
		LightLocalizer.travelTo(0, 1*TILE_SIZE);
		
		LightLocalizer.travelTo(1*TILE_SIZE, 1*TILE_SIZE);
		
		*/
		
		USLocalizer.localize(startCorner);
		LightLocalizer.correctY();

		LightLocalizer.travelTo(0, 0);
		
		LightLocalizer.turnTo(0);
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);

		System.exit(0);

	}
}
