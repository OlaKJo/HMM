package control;

import model.Localizer;
import model.Robot;
import view.RobotLocalizationViewer;

public class Main {
	/*
	 * build your own if you like, this is just an example of how to start the
	 * viewer ...
	 */

	final static int ROWS = 8;
	final static int COLS = 8;
	final static int HEAD = 4;

	public static void main(String[] args) {
		int rows = ROWS;
		int cols = COLS;

		if (args.length >= 1) {
			try {
				int arg = Integer.parseInt(args[0]);
				if (arg >= 5 && arg < 20) {
					rows = Integer.parseInt(args[0]);
					cols = rows;
				}
			} catch (Exception e) {
				System.out.println("There was an error with the input arguments. " + e.toString());
			}
		}
		Robot marvin = new Robot(rows, cols, HEAD);

		/*
		 * generate you own localiser / estimator wrapper here to plug it into the
		 * graphics class.
		 */

		EstimatorInterface localizer = new Localizer(rows, cols, HEAD, marvin);

		RobotLocalizationViewer viewer = new RobotLocalizationViewer(localizer);

		/*
		 * this thread controls the continuous update. If it is not started, you can
		 * only click through your localisation stepwise
		 */
		new LocalizationDriver(500, viewer).start();
	}
}