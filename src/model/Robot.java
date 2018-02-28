package model;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Robot {
	
	Tuple truePos;
	Tuple sensePos;
	Heading heading;
	Random rand = new Random();
	
	private List<Tuple> closestN;
	private List<Tuple> farthestN;
	
	int rows, cols, head;
	
	/**
	 * Class which represent the Robot in the environment
	 * Has methods to move the robot according to the given rules and generate sensor values according to the given probabilities
	 * Takes the dimensions of the world as parameters, as well as the number of headings.
	 * @param rows
	 * @param cols
	 * @param head
	 */
	public Robot(int rows, int cols, int head) {
		this.rows = rows;
		this.cols = cols;
		this.head = head;
		
		closestN = new ArrayList<Tuple>();
		farthestN = new ArrayList<Tuple>();
		
		init();
	}
	
	/**
	 * initialize the tuples which represent the true position and the sensor position
	 * Set start heading to a random heading
	 */
	private void init() {
		truePos = new Tuple(0, 0); 
		truePos.setX(rand.nextInt(rows));
		truePos.setY(rand.nextInt(cols));
		sensePos = new Tuple(0, 0);
				
		heading = Heading.values()[rand.nextInt(head)];
	}
	
	/**
	 * updated the lists of neighboring cells, moves the robot, and updates the sensor value
	 * returns the sensed position to the Localizer
	 * @return
	 */
	public Tuple update() {
		move();
		updateNeighbors();
		updateSensedPos();
		return sensePos;
	}
	
	/**
	 * calculates a new heading and updated the true position using this heading.
	 * @return
	 */
	public Tuple move(){
		heading = calculateNextHeading();
	
		switch (heading) {
			case North:
			// moving right
				truePos.addX(-1);
				break;
			case East:
			//moving down
				truePos.addY(1);
				break;
			case South:
			//moving left
				truePos.addX(1);
				break;
			case West:
			//moving up
				truePos.addY(-1);
				break;	
		}
		return truePos;
	}
	
	/**
	 * @return
	 */
	public Tuple getTruePos() {
		return truePos;
	}
	
	/**
	 * @return
	 */
	public Tuple getSensedPos() {
		return sensePos;
	}
	
	/**
	 * randomizes a new heading according to the task description
	 * @return
	 */
	private Heading calculateNextHeading() {
		Heading nextHeading = heading;
		
		float randDir = rand.nextFloat();
		if (randDir < 0.3) {
				nextHeading = Heading.values()[rand.nextInt(4)];		
		}
		if(checkWallInCurrHeading(nextHeading)) {
			return calculateNextHeading();
		}
		return nextHeading;	
	}

	/**
	 * returns true if the robot will enter enter a wall if taking 1 step in the current heading
	 * @param heading
	 * @return
	 */
	private boolean checkWallInCurrHeading(Heading heading) {
		switch (heading) {
		case East:
		// moving right
			if (truePos.getY() + 1 > cols - 1)
				return true;
			break;
		case South:
		//moving down
			if (truePos.getX() + 1 > rows - 1)
				return true;
			break;
		case West:
		//moving left
			if (truePos.getY() - 1 < 0)
				return true;
			break;
		case North:
		//moving up
			if (truePos.getX() - 1 < 0)
				return true;
			break;
		}
		return false;
	}
	
	/**
	 * update closestN and farthestN with current neighboring positions
	 */
	private void updateNeighbors() {
		closestN.clear();
		farthestN.clear();
		for (int i = -2; i <= 2; i++) {
			for (int j = -2; j <= 2; j++) {
				if (i == 0 && j == 0)
					continue;
				int xInd = truePos.getX() + i;
				int yInd = truePos.getY() + j;
				
				if (xInd >= 0 && yInd >= 0 && xInd < rows && yInd < cols) {
					Tuple n = new Tuple(xInd, yInd);
					if (Math.hypot(truePos.getX()-xInd, truePos.getY()-yInd) < 2) {
						closestN.add(n);
					} else {
						farthestN.add(n);
					}
				}
			}
		}
	}
	
	/**
	 * updated the sensed position with probabilitites given in the task description.
	 */
	private void updateSensedPos() {
		float sensorRand = rand.nextFloat();
		
		if (sensorRand < 0.1) {
			sensePos.setPos(truePos);
		} else if (sensorRand < (0.05*closestN.size() + 0.1)) {
			int index = rand.nextInt(closestN.size());
			sensePos.setPos(closestN.get(index));
		} else if (sensorRand < (0.025*farthestN.size() + 0.05*closestN.size() + 0.1)) {
			int index = rand.nextInt(farthestN.size());
			sensePos.setPos(farthestN.get(index));
		} else {
			sensePos.setPos(-1,-1);
		}
	}
}
