package ca.mcgill.ecse211.color;

import lejos.hardware.Sound;


public class colorEnum {

	Color color;
	static int redR = 192;
	static int redG = 44;
	static int redB = 8;
	static int blueR = 38;
	static int blueG = 87;
	static int blueB = 61;
	static int yellowR = 238 ;
	static int yellowG = 163;
	static int yellowB = 15;
	static int whiteR = 306;
	static int whiteG = 324;
	static int whiteB = 110;
	static int redSD = 15 ; 
	static int greenSD = 15;
	static int blueSD = 15;
	
	public colorEnum( Color color){
		this.color = color; 
	}
	
	public static Color colorEnumCalculator (float Red, float Green, float Blue){
		
		if ( (redR-2*redSD)<Red && Red<redR+2*redSD && redG-2*greenSD<Green && 
				Green<redG+2*greenSD && redB-2*blueSD<Blue && Blue<redB+2*blueSD ){
			return Color.RED; 
		}
		else if ( (blueR-2*redSD)<Red && Red<blueR+2*redSD && blueG-2*greenSD<Green && 
				Green<blueG+2*greenSD && blueB-2*blueSD<Blue && Blue<blueB+2*blueSD ){
			return Color.BLUE; 
		}
		else if ( (yellowR-2*redSD)<Red && Red<yellowR+2*redSD && yellowG-2*greenSD<Green && 
				Green<yellowG+2*greenSD && yellowB-2*blueSD<Blue && Blue<yellowB+2*blueSD ){
			return Color.YELLOW; 
		}
		else if ( (whiteR-2*redSD)<Red && Red<whiteR+2*redSD && whiteG-2*greenSD<Green && 
				Green<whiteG+2*greenSD && whiteB-2*blueSD<Blue && Blue<whiteB+2*blueSD ){
			return Color.WHITE; 
		}
		else {
			return Color.NONE;
		}
	}
	
	public static String colorToString (Color color, boolean correctColor){
		if (correctColor){
			Sound.twoBeeps();
		}
		else if (!correctColor && !color.equals(Color.NONE)){
			Sound.beep();
		}
		if (color.equals(Color.RED)){
			return "RED";
		}
		if (color.equals(Color.BLUE)){
			return "BLUE";
		}
		if (color.equals(Color.YELLOW)){
			return "YELLOW";
		}
		if (color.equals(Color.WHITE)){
			return "WHITE";
		}
		else {
		     return null;
		}
	}
	
	
	public String colorResult(Color color) {
		if (color.equals(Color.RED)){ 
			return "Object Detected Red";
		}
		else if (color.equals(Color.BLUE)){ 
			return "Object Detected Blue";
		}
		else if (color.equals(Color.YELLOW)){ 
			return "Object Detected Yellow";
		}
		else if (color.equals(Color.WHITE)){ 
			return "Object Detected White";
		}
		else{ 
			return "No Object Detected";
		}
		
	}
}
