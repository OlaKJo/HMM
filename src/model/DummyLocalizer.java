package model;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import control.EstimatorInterface;

public class DummyLocalizer implements EstimatorInterface {
		
	private int rows, cols, head;
	private int[] truePos, sensePos;
	private Random rand;
	private List<Tuple> closestN;
	private List<Tuple> farthestN;
	private int heading;

	public DummyLocalizer( int rows, int cols, int head) {
		this.rows = rows;
		this.cols = cols;
		this.head = head;
		truePos = new int[2]; 
		sensePos = new int[2];
		closestN = new ArrayList<Tuple>();
		farthestN = new ArrayList<Tuple>();
		rand = new Random();
		truePos[0] = rand.nextInt(rows);
		//truePos[1] = rand.nextInt(cols);
		truePos[1] = 0;
		heading = 1;
	}	
	
	public int getNumRows() {
		return rows;
	}
	
	public int getNumCols() {
		return cols;
	}
	
	public int getNumHead() {
		return head;
	}
	
	public double getTProb( int x, int y, int h, int nX, int nY, int nH) {
		return 0.0;
	}

	public double getOrXY( int rX, int rY, int x, int y, int h) {
		return 0.1;
	}


	public int[] getCurrentTruePosition() {
		return truePos;
	}

	public int[] getCurrentReading() {
		return sensePos;
	}


	public double getCurrentProb( int x, int y) {
		double ret = 0.0;
		return ret;
	}
	
	public void update() {
		move();
		updateNeighbors();
		updateSensedPos();
	}
	
	private void updateNeighbors() {
		closestN.clear();
		farthestN.clear();
		for (int i = -2; i <= 2; i++) {
			for (int j = -2; j <= 2; j++) {
				if (i == 0 && j == 0)
					continue;
				int xInd = truePos[0] + i;
				int yInd = truePos[1] + j;
				
				if (xInd >= 0 && yInd >= 0 && xInd < rows && yInd < cols) {
					Tuple n = new Tuple(xInd, yInd);
					if (Math.hypot(truePos[0]-xInd, truePos[1]-yInd) < 2) {
						closestN.add(n);
					} else {
						farthestN.add(n);
					}
				}
			}
		}
		System.out.println("Closest Neighbors");
		for(Tuple t : closestN) {
			System.out.println(t);
		}
		System.out.println("Farthest Neighbors");
		for(Tuple t : farthestN) {
			System.out.println(t);
		}
	}

	private void updateSensedPos() {
		float sensorRand = rand.nextFloat();
		
		if (sensorRand < 0.1) {
			sensePos[0] = truePos[0];
			sensePos[1] = truePos[1];
		} else if (sensorRand < (0.05*closestN.size() + 0.1)) {
			int index = rand.nextInt(closestN.size());
			sensePos[0] = closestN.get(index).getX();
			sensePos[1] = closestN.get(index).getY();
		} else if (sensorRand < (0.025*farthestN.size() + 0.05*closestN.size() + 0.1)) {
			int index = rand.nextInt(farthestN.size());
			sensePos[0] = farthestN.get(index).getX();
			sensePos[1] = farthestN.get(index).getY();
		} else {
			sensePos[0] = -1;
			sensePos[1] = -1;
		}
	}

	private void move(){
		//moving in dimension X
		if (heading == 0 || heading == 1) {
			if (truePos[0] == (rows - 1) || truePos[0] == 0) {
				heading++;
				heading %= 2;
			}
		}
		if (heading == 0) {
			truePos[0]++;
		} else {
			truePos[0]--;
		}
	}
}