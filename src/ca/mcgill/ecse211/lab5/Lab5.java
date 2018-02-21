package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.color.*;
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
	
	private static SensorModes RGBSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private static SampleProvider RGBColor = RGBSensor.getMode("RGB");
	private static float[] RGBData = new float[RGBColor.sampleSize()];
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S1");
	//private static final Port lightPort = LocalEV3.get().getPort("S1");

	private static int startCorner;
	public static Color targetColor;
	

	public static final double WHEEL_RAD = 2.12;//2.12
	public static final double TRACK = 10.2;//
	public static final double TILE_SIZE = 30.48;
	public static final int GRID_SIZE = 4;
	public static final int MOTOR_ROTATE = 80;
	public static final int MOTOR_STRAIGHT = 150;

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
		//Navigation navigation = new Navigation(odometer, leftMotor, rightMotor);

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
		
		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("       1        ", 0, 0);
			lcd.drawString("      Red       ", 0, 1);
			lcd.drawString("4 White   Blue 2", 0, 2);
			lcd.drawString("    Yellow      ", 0, 3);
			lcd.drawString("       3        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (
				buttonChoice != Button.ID_LEFT && 
				buttonChoice != Button.ID_RIGHT && 
				buttonChoice != Button.ID_DOWN && 
				buttonChoice != Button.ID_UP
				);
		
		if (buttonChoice == Button.ID_RIGHT){
			targetColor = Color.BLUE;
		}
		if (buttonChoice == Button.ID_DOWN){
			targetColor = Color.YELLOW;
		}
		if (buttonChoice == Button.ID_LEFT){
			targetColor = Color.WHITE;
		}
		if (buttonChoice == Button.ID_UP){
			targetColor = Color.RED;
			}
		
		
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		
		Thread ododisplayThread = new Thread(odometrydisplay);
		ododisplayThread.start();

		colorSensor colorSensor= new colorSensor(RGBData, RGBColor, targetColor);
		colorSensor.start();
		
		lightLocalizer lightLocalizer = new lightLocalizer(odometer, leftMotor, rightMotor, RColor, LColor, RData, LData);
		Navigation navigator = new Navigation(odometer, leftMotor, rightMotor, lightLocalizer);
		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, usDistance, navigator);

		
		USLocalizer.localize(startCorner);
		lightLocalizer.correctXY(startCorner);
		lightLocalizer.setXTOffset(startCorner);
		
		navigator.travelTo(0, 6);
		lightLocalizer.correctLocation();
		navigator.travelTo(6, 6);
		lightLocalizer.correctLocation();
		navigator.travelTo(6, 0);
		lightLocalizer.correctLocation();
		navigator.travelTo(0, 0);
		lightLocalizer.correctLocation();
		
		
		
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);

		System.exit(0);

	}
}
