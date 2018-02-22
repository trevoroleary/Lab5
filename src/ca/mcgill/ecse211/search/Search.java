package ca.mcgill.ecse211.search;

import ca.mcgill.ecse211.color.Color;
import ca.mcgill.ecse211.color.colorSensor;
import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.robotics.SampleProvider;

public class Search {

	private final int[] LL;
	private final int[] UR;
	private colorSensor sensor;
	private Odometer odometer;
	private SampleProvider usDistance;
	private Navigation navigator;
	private float[] usSample;
	private float usData;
	private boolean upTrue = true;
	
	
	private float blockInFront = 10;
	
	public Search(int[] LL, int[] UR, colorSensor colorSensor, Odometer odometer, SampleProvider usDistance, Navigation navigator) {
		
		this.LL = LL;
		this.UR = UR;
		this.sensor = colorSensor;
		this.odometer = odometer;
		this.usDistance = usDistance;
		this.navigator = navigator;
		this.usSample = new float[1];
		this.usData = 0;
		
	}
	
	public void beginSearch() {
		checkSide();
		checkSide();
		checkSide();
		checkSide();
	}
	
	public int[] checkSide() {
		int[] returnVals = new int[] {0,0,0}; // 1 at 0 = nothing.. 1 at 1 = something right.... 1 at 2 = something left
		
		//Looks to its right for object
		navigator.turn(45);

		//TODO USDATA GET SAMPLE AND RETURN IF OBJECT
		
		if(false) {
			usData = 0;
			returnVals[1] = 1;
			//TODO IDENTIFY BLOCK WITH FIND AHEAD
			return returnVals;
		}
		
		//Looks to its left for an object
		else {
			navigator.turn(-90);
			usData = 0;
			
			//TODO USDATA GET SAMPLE AND RETURN IF OBJECT
			
			if(false) {
				usData = 0;
				returnVals[2] = 1;
				return returnVals;
			}
			else {
				if(upTrue) 
					navigator.turnTo(0,false);
				else
					navigator.turnTo(180, false);
			    usData = 0;
			    nextLine();
			    return returnVals;
			}

		}
		
	}
	
	public void nextLine() {
		int x = (int) ((odometer.getX()/Lab5.TILE_SIZE) + 0.5);
		int y = (int) ((odometer.getY()/Lab5.TILE_SIZE) + 0.5);
		int theta = (int) (odometer.nearestHeading() + 0.5);
		if((y == UR[1] && theta == 0) || (y == LL[1] && theta == 180)) {
			nextSection();
		}
		else { 
			if(upTrue) {
				navigator.travelTo(x, y + 1);
				navigator.turnTo(0, true);
				checkSide();
				
			}
			else {
				navigator.travelTo(x, y - 1);
				navigator.turnTo(180,true);
				checkSide();
			}
		}
	}
	
	public void nextSection() {
		int x = (int) ((odometer.getX()/Lab5.TILE_SIZE) + 0.5);
		int y = (int) ((odometer.getY()/Lab5.TILE_SIZE) + 0.5);
		navigator.travelTo(x, y);
		navigator.travelTo(x + 1, y);
		if(y == UR[1]) { 
			upTrue = false;
			navigator.turnTo(180, true);
		}
		else {
			upTrue = true;
			navigator.turnTo(0, true);
		}
		}
	}

