package control;

import model.Localizer;
import model.Robot;
import view.RobotLocalizationViewer;

public class Main {
	/*
	 * build your own if you like, this is just an example of how to start the viewer
	 * ...
	 */
	
	final static int ROWS = 8;
	final static int COLS = 8;
	final static int HEAD = 4;
	
	public static void main( String[] args) {
				
		Robot marvin = new Robot(ROWS, COLS, HEAD);
		
		/*
		 * generate you own localiser / estimator wrapper here to plug it into the 
		 * graphics class.
		 */

		EstimatorInterface localizer = new Localizer(ROWS, COLS, HEAD, marvin);

		RobotLocalizationViewer viewer = new RobotLocalizationViewer( localizer);

		/*
		 * this thread controls the continuous update. If it is not started, 
		 * you can only click through your localisation stepwise
		 */
		new LocalizationDriver( 500, viewer).start();
	}
}	