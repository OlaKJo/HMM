package control;

import view.*;

public class LocalizationDriver extends Thread {
	
	private RobotLocalizationViewer viewer;
	long timer;
	
	public LocalizationDriver( long stepTime, RobotLocalizationViewer v) {
		this.viewer = v;
		this.timer = stepTime;
	}
	
	public void run() {
		while( !isInterrupted()) {
			
			
			try{
				viewer.updateContinuously();
				sleep( timer);
			} catch( InterruptedException e) {
				System.out.println( "oops");
			}

		}
	}
	
}