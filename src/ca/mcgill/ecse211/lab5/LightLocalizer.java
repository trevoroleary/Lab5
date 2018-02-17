package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;
//import lab3.Lab3;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class LightLocalizer extends Thread  {
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3ColorSensor colorSensorL, colorSensorR;
	
	//initialize parameters
	public static final int MOTOR_ROTATE = 60;
	private static final int MOTOR_STRAIGHT = 150;
	private final double SENSOR_OFFSET = 5.0;		//distance between center of the track and light sensor
	public static final double WHEEL_RAD = 2.05;
	public static final double TRACK = 10.2;	//17.115
	public SampleProvider RColor, LColor;
	public float[] RData, LData;
	
	private Odometer odometer;

	//destination position
	private double x;
	private double y;
	
	//difference between current position and destination position
	private double dX;
	private double dY;
	private double dTheta;
	
	//angle toward destination
	private double currentX;
	private double currentY;
	private double Theta;
	
	//current position
	private double currentTheta;
	
	//distance between current position and destination position
	private double distance;
	
	private int linecounter = 0;

			
	//constructor
	public LightLocalizer(Odometer odo,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, SampleProvider RColor, SampleProvider LColor, float[] RData, float[] LData ) {
				
		this.RColor = RColor;
		this.RData = RData;
		
		this.LColor = LColor;
		this.LData = LData;
		
		this.odometer = odo;
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		
	    try {
			this.odometer = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
	
	/*
	public void localization(){

		double heading = nearestHeading(odometer.getTheta());
		
		odometer.setTheta(0);	
		
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 45), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 45),false);
		//robot turn right for 45 degree
		
	    
		int colorDetected = colourSensor.getColorID(); 
	
		//create an array to store the theta value
		double[] getTheta = new double[4];
		
		//keep the robot rotate counterclockwise
		leftMotor.setSpeed(MOTOR_ROTATE);
		rightMotor.setSpeed(MOTOR_ROTATE);
	    leftMotor.backward();
	    rightMotor.forward();
		
		//get theta value when encounter black lines each time
	    //thus first and third number in array is theta_y; index 0 and 2
	    //second and fourth number in array is theta_x;index 1 and 3
		while (linecounter < 4){
			 colorDetected = colourSensor.getColorID(); // black=13, yellow=6
			  if (colorDetected >= 12){
				  Sound.beep();
				  LCD.drawString("linecounter=" + linecounter, 0, 4);
				  getTheta[linecounter] = odometer.getTheta();
				  linecounter = linecounter +1;
			  }
		}
		
		turnTo(0);
		
		//calculate the theta_x and theta_y
		double ThetaX = getTheta[3] - getTheta[1];
		double ThetaY = getTheta[2] - getTheta[0];
		
		//calculate the distance from robot to black lines
		double x = -SENSOR_OFFSET*Math.cos(Math.toRadians(ThetaY/2));
		double y = -SENSOR_OFFSET*Math.cos(Math.toRadians(ThetaX/2));
		
		//correction
		odometer.setX(x);
		odometer.setY(y);
		
		//travel to origin
		travelTo(0,0);
		turnTo(0);
		
		odometer.setTheta(heading);
		}
	
	public void localization(int startCorner) {
		//robot turn right for 45 degree
		turnTo(45);
		
		//move straight
	    leftMotor.rotate(convertDistance(WHEEL_RAD, 10), true);
	    rightMotor.rotate(convertDistance(WHEEL_RAD, 10), false);
	    
		int colorDetected = colourSensor.getColorID(); 
	
		//create an array to store the theta value
		double[] getTheta = new double[4];
		
		//keep the robot rotate counterclockwise
		leftMotor.setSpeed(MOTOR_ROTATE);
		rightMotor.setSpeed(MOTOR_ROTATE);
	    leftMotor.backward();
	    rightMotor.forward();
		
		//get theta value when encounter black lines each time
	    //thus first and third number in array is theta_y; index 0 and 2
	    //second and fourth number in array is theta_x;index 1 and 3
		while (linecounter < 4){
			 colorDetected = colourSensor.getColorID(); // black=13, yellow=6
			  if (colorDetected >= 12){
				  Sound.beep();
				  LCD.drawString("linecounter=" + linecounter, 0, 4);
				  getTheta[linecounter] = odometer.getTheta();
				  linecounter = linecounter +1;
			  }
		}
		
		turnTo(0);
		
		//calculate the theta_x and theta_y
		double ThetaX = getTheta[3] - getTheta[1];
		double ThetaY = getTheta[2] - getTheta[0];
		
		//calculate the distance from robot to black lines
		double x = -SENSOR_OFFSET*Math.cos(Math.toRadians(ThetaY/2));
		double y = -SENSOR_OFFSET*Math.cos(Math.toRadians(ThetaX/2));
		
		//correction
		odometer.setX(x);
		odometer.setY(y);
		
		//travel to origin
		travelTo(0,0);
		turnTo(0);
		
		if(startCorner == 1)
			odometer.setTheta(270);
		if(startCorner == 2)
			odometer.setTheta(180);
		if(startCorner == 3)
			odometer.setTheta(90);
		
	}
	*/
	
	public void correctY() {

		boolean Navigating = true;
				
		leftMotor.setSpeed(50);
		rightMotor.setSpeed(50);
		
		leftMotor.forward();
		rightMotor.forward();
		
	  while(Navigating) {
		  
		RColor.fetchSample(RData, 0); // acquire data
		LColor.fetchSample(LData, 0);
		
	    
	   	if(RData[0] < 0.4) {
	   		leftMotor.stop(true);
	   		rightMotor.stop(false);
	   		fixTheta(0);
	   		odometer.setY(-SENSOR_OFFSET);
	   		Navigating = false;
    	}
	   	if(LData[0] < 0.4) {
	   		leftMotor.stop(true);
	   		rightMotor.stop(false);
	   		fixTheta(1);
	   		odometer.setY(-SENSOR_OFFSET);
	   		Navigating = false;
	   	}

	  }
	  correctX();
	}
	
	public void correctX() {
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		leftMotor.setSpeed(MOTOR_STRAIGHT);
		rightMotor.setSpeed(MOTOR_STRAIGHT);
		
		leftMotor.rotate(-180, true);
		rightMotor.rotate(-180, false);
		
		leftMotor.stop(true);
		rightMotor.stop(false);

		leftMotor.setSpeed(MOTOR_ROTATE);
		rightMotor.setSpeed(MOTOR_ROTATE);
		
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
		    
		
		boolean Navigating = true;
				
		leftMotor.setSpeed(50);
		rightMotor.setSpeed(50);
		
		leftMotor.forward();
		rightMotor.forward();
		
	  while(Navigating) {
		  
		RColor.fetchSample(RData, 0); // acquire data
		LColor.fetchSample(LData, 0);
		
	    
	   	if(RData[0] < 0.4) {
	   		leftMotor.stop(true);
	   		rightMotor.stop(false);
	   		fixTheta(0);
	   		odometer.setX(-SENSOR_OFFSET);
	   		Navigating = false;
    	}
	   	if(LData[0] < 0.4) {
	   		leftMotor.stop(true);
	   		rightMotor.stop(false);
	   		fixTheta(1);
	   		odometer.setX(-SENSOR_OFFSET);
	   		Navigating = false;
	   	}

	  }
	}
	
	public void fixTheta(int wheel){
		boolean Navigating = false;
		
		if(wheel == 0) {
			leftMotor.setSpeed(50);
			leftMotor.forward();
			Navigating = true;
			
			while(Navigating) {
				LColor.fetchSample(LData, 0);
				if(LData[0] < 0.4) {
					leftMotor.stop(true);
					Navigating = false;
				}
			}
		}
		if(wheel == 1) {
			rightMotor.setSpeed(50);
			rightMotor.forward();
			Navigating = true;
			
			while(Navigating) {
				RColor.fetchSample(RData, 0);
				if(RData[0] < 0.4) {
					rightMotor.stop(true);
					Navigating = false;
				}
			}
		}
		
		odometer.setTheta( nearestHeading( odometer.getTheta() ) );
	}
	
	//travel to the destination 
	public void travelTo(double x, double y){
		
		this.x = x;
		this.y = y;
		
		//get current position
		currentX = odometer.getX();
		currentY = odometer.getY();
		
		//compute the difference
		dX = x - currentX;
		dY = y - currentY;
		
		distance = Math.sqrt(Math.pow(dX, 2.0)+ Math.pow(dY, 2.0));
		Theta = (Math.atan2(dX, dY))*180/Math.PI;					//convert from radius to degree
		
		//rotate toward destination
		turnTo(Theta);
				
		//if there isn't block, just move straight
		rightMotor.setSpeed(MOTOR_STRAIGHT); 
		leftMotor.setSpeed(MOTOR_STRAIGHT);
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);

		
		}
	
	
	//turn at desired angle
	public void turnTo(double Theta) { 
		currentTheta = odometer.getTheta();		//currentTheta is in degree
		dTheta = Theta - currentTheta;
		
		//avoid maximal angle turn
		if(dTheta > 180){
			dTheta =  360 - dTheta;
		}
		else if(dTheta < -180){
			dTheta = 360 + dTheta;
		}

		//turn minimal Theta degree 
    	leftMotor.setSpeed(MOTOR_ROTATE);
    	rightMotor.setSpeed(MOTOR_ROTATE);
    	leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dTheta), true);
    	rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dTheta), false);
		
    	try{
    		Thread.sleep(150);
    	} catch(InterruptedException e){} 
    	
	}
	
	
	  /**
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return
	   */
	  private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }

	  private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
		  }

	  private static double nearestHeading(double heading) {
		  
			if((0 - heading) > -45)
				heading = 0.0;
			else if((90 - heading) > -45)
				heading = 90.0;
			else if((180 - heading) > - 45)
				heading = 180.0;
			else if((270 - heading) > - 45)
				heading = 270.0;
			else
				heading = 0.0;
	
			return heading;
	  }
}

