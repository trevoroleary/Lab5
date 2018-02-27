package ca.mcgill.ecse211.color;

import ca.mcgill.ecse211.lab5.*;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class colorSensor extends Thread {

	public float[] RGBData;
	public static int targetColor;
	public static int sensorColor;
	public static int prevColor;
	public static boolean colorFoundBoolean = false;
	public float red;
	public float green;
	public float blue;
	public SampleProvider RGBColor;
	public static boolean correctColor = false;
	public static String foundColor = "";
	public static String colorResponse = "";

	private static final long COLOR_PERIOD = 25;

	// Red = 1
	// Blue = 2
	// Yellow = 3
	// White = 4

	private static float redR = 574;
	private static float redG = 83;
	private static float redB = 64;
	private static float blueR = 88;
	private static float blueG = 220;
	private static float blueB = 314;
	private static float yellowR = 1015;
	private static float yellowG = 721;
	private static float yellowB = 127;
	private static float whiteR = 995;
	private static float whiteG = 1000;
	private static float whiteB = 779;

	private static float redRSD = 15;
	private static float redGSD = 5;
	private static float redBSD = 5;
	private static float blueRSD = 10;
	private static float blueGSD = 15;
	private static float blueBSD = 20;
	private static float yellowRSD = 44;
	private static float yellowGSD = 34;
	private static float yellowBSD = 10;
	private static float whiteRSD = 25;
	private static float whiteGSD = 20;
	private static float whiteBSD = 25;

	private static double sdMultiplier = 4;

	public colorSensor(float[] RGBData, SampleProvider RGBColor, int targetColor) {
		this.RGBData = RGBData;
		this.RGBColor = RGBColor;
		this.targetColor = targetColor;
	}

	/**
	 * this method returns the integer value of what is read by the color
	 * sensor.
	 * 
	 * @param Red
	 * @param Green
	 * @param Blue
	 * @return int
	 */
	public static int colorEnumCalculator(float Red, float Green, float Blue) {

		if ((redR - sdMultiplier * redRSD) < Red && Red < redR + sdMultiplier * redRSD
				&& redG - sdMultiplier * redGSD < Green && Green < redG + sdMultiplier * redGSD
				&& redB - sdMultiplier * redBSD < Blue && Blue < redB + sdMultiplier * redBSD) {
			return 1;
		}
		if ((blueR - sdMultiplier * blueRSD) < Red && Red < blueR + sdMultiplier * blueRSD
				&& blueG - sdMultiplier * blueGSD < Green && Green < blueG + sdMultiplier * blueGSD
				&& blueB - sdMultiplier * blueBSD < Blue && Blue < blueB + sdMultiplier * blueBSD) {
			return 2;
		}
		if ((yellowR - sdMultiplier * yellowRSD) < Red && Red < yellowR + sdMultiplier * yellowRSD
				&& yellowG - sdMultiplier * yellowGSD < Green && Green < yellowG + sdMultiplier * yellowGSD
				&& yellowB - sdMultiplier * yellowBSD < Blue && Blue < yellowB + sdMultiplier * yellowBSD) {
			return 3;
		}
		if ((whiteR - sdMultiplier * whiteRSD) < Red && Red < whiteR + sdMultiplier * whiteRSD
				&& whiteG - sdMultiplier * whiteGSD < Green && Green < whiteG + sdMultiplier * whiteGSD
				&& whiteB - sdMultiplier * whiteBSD < Blue && Blue < whiteB + sdMultiplier * whiteBSD) {
			return 4;
		} else {
			return 0;
		}
	}

	/**
	 * this method returns a string that is relative to the color that the
	 * sensor reads.
	 * 
	 * @param color
	 * @param correctColor
	 * @return string
	 */
	public static String colorToString(int color, boolean correctColor) {
		if (correctColor && prevColor != color) {
			Sound.beep();
			prevColor = color;
			colorFoundBoolean = true;
		} else if (!correctColor && color != 0 && prevColor != color) {
			Sound.twoBeeps();
		}
		if (color == 1) {
			prevColor = color;
			return "RED";
		}
		if (color == 2) {
			prevColor = color;
			return "BLUE";
		}
		if (color == 3) {
			prevColor = color;
			return "YELLOW";
		}
		if (color == 4) {
			prevColor = color;
			return "WHITE";
		} else {
			return "";
		}
	}

	/**
	 * getter for the foundColor String
	 * 
	 * @return string
	 */
	public static String getResponse() {
		return foundColor;
	}

	/**
	 * getter for the colorFoundBoolean .
	 * 
	 * @return boolean
	 */
	public static boolean seeColor() {
		return colorFoundBoolean;
	}

	public void run() {
		long updateStart, updateEnd;
		while (true) {
			updateStart = System.currentTimeMillis();
			RGBColor.fetchSample(RGBData, 0);
			red = 10000 * RGBData[0];
			green = 10000 * RGBData[1];
			blue = 10000 * RGBData[2];
			sensorColor = colorEnumCalculator(red, green, blue);
			if (sensorColor == targetColor) {
				correctColor = true;
			} else {
				correctColor = false;
			}
			foundColor = colorToString(sensorColor, correctColor);
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < COLOR_PERIOD) {
				try {
					Thread.sleep(COLOR_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
				}
			}
		}
	}

}
