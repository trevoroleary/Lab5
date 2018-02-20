package ca.mcgill.ecse211.color;

import ca.mcgill.ecse211.lab5.*;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class colorSensor extends Thread {

	float[][] allData = new float[400][3];
	int j = 0;
	public float[] RGBData;
	public Color targetColor;
	public Color sensorColor;
	public static Color prevColor;
	public float red;
	public float green;
	public float blue;
	public SampleProvider RGBColor;
	private boolean correctColor = false;
	public static String foundColor="";
	public static String colorResponse="";

	private static final long COLOR_PERIOD = 25;
	
	private static float redR = 172;
	private static float redG = 25;
	private static float redB = 25;
	private static float blueR = 24;
	private static float blueG = 59;
	private static float blueB = 88;
	private static float yellowR = 265;
	private static float yellowG = 181;
	private static float yellowB = 34;
	private static float whiteR = 274;
	private static float whiteG = 270;
	private static float whiteB = 255;
	
	private static float redRSD = 44; 
	private static float redGSD = 5;
	private static float redBSD = 5;
	private static float blueRSD = 5;
	private static float blueGSD = 20;
	private static float blueBSD = 30;
	private static float yellowRSD = 78;
	private static float yellowGSD = 54;
	private static float yellowBSD = 5;
	private static float whiteRSD = 78;
	private static float whiteGSD = 83;
	private static float whiteBSD = 88;
	

	public colorSensor(float[] RGBData, SampleProvider RGBColor, Color targetColor ) {
		this.RGBData=RGBData;
		this.RGBColor=RGBColor;
		this.targetColor = targetColor;
	}

	public static Color colorEnumCalculator(float Red, float Green, float Blue) {

		if ((redR - 1.5 * redRSD) < Red && Red < redR + 1.5 * redRSD && redG - 1.5 * redGSD < Green
				&& Green < redG + 1.5 * redGSD && redB - 1.5 * redBSD < Blue && Blue < redB + 1.5 * redBSD) {
			return Color.RED;
		}if ((blueR - 1.5 * blueRSD) < Red && Red < blueR + 1.5 * blueRSD && blueG - 1.5 * blueGSD < Green
				&& Green < blueG + 1.5 * blueGSD && blueB - 1.5 * blueBSD < Blue && Blue < blueB + 1.5 * blueBSD) {
			return Color.BLUE;
		}if ((yellowR - 1.5 * yellowRSD) < Red && Red < yellowR + 1.5 * yellowRSD && yellowG - 1.5 * yellowGSD < Green
				&& Green < yellowG + 1.5 * yellowGSD && yellowB - 1.5 * yellowBSD < Blue && Blue < yellowB + 1.5 * yellowBSD) {
			return Color.YELLOW;
		}if ((whiteR - 1.5 * whiteRSD) < Red && Red < whiteR + 1.5 * whiteRSD && whiteG - 1.5 * whiteGSD < Green
				&& Green < whiteG + 1.5 * whiteGSD && whiteB - 1.5 * whiteBSD < Blue && Blue < whiteB + 1.5 * whiteBSD) {
			return Color.WHITE;
		} else {
			return Color.NONE;
		}
	}

	public static String colorToString(Color color, boolean correctColor) {
		if (correctColor && prevColor!=color) {
			Sound.twoBeeps();
		} else if (!correctColor && !color.equals(Color.NONE) && prevColor!=color) {
			Sound.beep();
		}
		if (color.equals(Color.RED)) {
			prevColor=color;
			return "RED";
		}
		if (color.equals(Color.BLUE)) {
			prevColor=color;
			return "BLUE";
		}
		if (color.equals(Color.YELLOW)) {
			prevColor=color;
			return "YELLOW";
		}
		if (color.equals(Color.WHITE)) {
			prevColor=color;
			return "WHITE";
		} else {
			return "";
		}
	}

	public String colorResult(Color color) {
		if (color.equals(Color.RED)) {
			return "Object Detected Red";
		} else if (color.equals(Color.BLUE)) {
			return "Object Detected Blue";
		} else if (color.equals(Color.YELLOW)) {
			return "Object Detected Yellow";
		} else if (color.equals(Color.WHITE)) {
			return "Object Detected White";
		} else {
			return "No Object Detected";
		}

	}
	
	public static String getResponse(){
		return foundColor;
	}
	
	/*public void printData() {
		System.out.println("-----ALL RED DATA-----");
		for(int i = 0; i<200; i++) {
			System.out.println(allData[i][0]);
		}
		System.out.println("-----ALL GREEN DATA-----");
		for(int i = 0; i<200; i++) {
			System.out.println(allData[i][1]);
		}
		System.out.println("-----ALL BLUE DATA-----");
		for(int i = 0; i<200; i++) {
			System.out.println(allData[i][2]);
		}
	}
	*/

	public void run() {
		long updateStart, updateEnd;
		while(true) {
			updateStart=System.currentTimeMillis();
		RGBColor.fetchSample(RGBData, 0);
		red =10000* RGBData[0];
		green =10000* RGBData[1];
		blue =10000* RGBData[2];
		/*if(j<400) {	
			allData[j][0] = red;
			allData[j][1] = green;
			allData[j][2] = blue;
		}
		j++;
		if(j == 400) {
			printData();
		}
		*/
		sensorColor = colorEnumCalculator(red, green, blue);
		if (sensorColor.equals(targetColor)) {
			correctColor = true;
		} else {
			correctColor = false;
		}
		foundColor = colorToString(sensorColor, correctColor);
		colorResponse = colorResult(sensorColor);
		updateEnd=System.currentTimeMillis();
		if (updateEnd-updateStart < COLOR_PERIOD) {
			try {
				Thread.sleep(COLOR_PERIOD-(updateEnd-updateStart));
			} catch (InterruptedException e) {
			}
			}
		}
	}

}

