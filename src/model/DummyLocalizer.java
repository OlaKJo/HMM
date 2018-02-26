package model;

import java.util.ArrayList;
import java.util.Arrays;
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
	private double[][] alpha;

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
		alpha = new double[rows][cols];
		double val = 1.0/(rows*cols);
		double[] tempArr = new double[cols];
		Arrays.fill(tempArr, val);
		Arrays.fill(alpha, tempArr);
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
		return alpha[x][y];
	}
	
	public void update() {
		move();
		updateNeighbors();
		updateSensedPos();
		updateProb();
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
	
	private void updateProb(){
		double[][] emissMat = getEmissProb(sensePos[0], sensePos[1]);
		System.out.println(getTransProb(2,2,2,3,1));
		double[][] newAlpha = new double[rows][cols];
		double[][] S = new double[rows][cols];
		double[][] E = new double[rows][cols];
		//loops x_t
		for(int m = 0; m < rows; m++) {
			for(int n = 0; n < cols; n++) {
				double sum = 0;
				
				//loops x_tminus1
				for(int i = 0; i < rows; i++) {
					for(int j = 0; j < cols; j++) {
						sum += getTransProb(i,j,m,n, heading)*alpha[i][j];
					}
				}
				S[m][n] = sum;
			}
		}
		
		E = getEmissProb(sensePos[0], sensePos[1]);
		
		double runningSum = 0;
		for(int m = 0; m < rows; m++) {
			for(int n = 0; n < cols; n++) {
				newAlpha[m][n] = S[m][n]*E[m][n];
				runningSum += newAlpha[m][n];
			}
		}
		
		for(int m = 0; m < rows; m++) {
			for(int n = 0; n < cols; n++) {
				newAlpha[m][n] = newAlpha[m][n]/runningSum;
			}
		}
		
		alpha = newAlpha;
	}
	
	private double getTransProb(int x_tminus1, int y_tminus1, int x_t, int y_t, int heading){
//		double[][] transMat = new double[alpha.length][alpha[0].length];
//		
//		for (int i = 0; i < alpha.length; i++) {
//			for (int j = 0; j < alpha[0].length; j++) {
//				transMat[i][j] = 0;
//			}
//		}
//		
//		float pot = 1;
//		List<Tuple> PathsFromX_tminus1 = getXYNeighbors(x_tminus1, y_tminus1);
//		
//		Tuple nextInHeading = nextForwardPos(x_tminus1, y_tminus1, heading);
//		
//		if(PathsFromX_tminus1.contains(nextInHeading)) {
//			transMat[nextInHeading.getX()][nextInHeading.getY()] = 0.7;
//			pot -= 0.7;
//			PathsFromX_tminus1.remove(nextInHeading);
//		}
//		
//		float remPot = pot/PathsFromX_tminus1.size();
//		
//		for (Tuple x : PathsFromX_tminus1) {
//			transMat[x.getX()][x.getY()] = remPot;
//		}
//		
//		return transMat;
		
		List<Tuple> PathsFromX_tminus1 = getXYNeighbors(x_tminus1, y_tminus1);
		Tuple xt = new Tuple(x_t, y_t);
		Tuple nextInHeading = nextForwardPos(x_tminus1, y_tminus1, heading);
		
		double pot = 1;
		
		if(PathsFromX_tminus1.contains(nextInHeading)) {
			pot -= 0.7;
			PathsFromX_tminus1.remove(nextInHeading);
		}
		
		if(xt.equals(nextInHeading)) {
			return 0.7;
		}
		
		return pot/PathsFromX_tminus1.size();
		
	}
	
	private Tuple nextForwardPos(int x, int y, int heading) {
		switch (heading) {
		case 0:
		// moving right
			y++;
			break;
		case 1:
		//moving down
			x++;
			break;
		case 2:
		//moving left
			y--;
			break;
		case 3:
		//moving up
			x--;
			break;
		}
		return new Tuple(x,y);
	}

	private List<Tuple> getXYNeighbors(int x, int y) {
		List<Tuple> retList = new ArrayList<Tuple>();
		for(int i = -1; i <= 1; i++) {
			for(int j = -1; j <= 1; j++) {
				if ((Math.abs(i) + Math.abs(j)) != 1)
					continue;
				if(x+i >= 0 && x+i < rows && y+j >= 0 && y+j < cols)
					retList.add(new Tuple(x+i,y+j));
			}
		}
		return retList;
	}

	private double[][] getEmissProb(int senseX, int senseY){
		double[][] emissMat = new double[alpha.length][alpha[0].length];
		for (int i = 0; i < alpha.length; i++) {
			for (int j = 0; j < alpha[0].length; j++) {
				if(senseX == i && senseY == j){
					 emissMat[i][j] = 0.1;
					 continue;
				 }
				if(senseX != -1 && Math.hypot(senseX-i, senseY-j) > Math.sqrt(8)) {
					emissMat[i][j] = 0;
					continue;
				}
				 List<List<Tuple>> neighbors = getXNeighbors(i, j);
				 List<Tuple> closest = neighbors.get(0);
				 List<Tuple> farthest = neighbors.get(1);
				 
				 if(senseX!=-1){
					 System.out.println("");
				 }
				 Tuple sensed = new Tuple(senseX, senseY);
				 if(closest.contains(sensed)){
					 emissMat[i][j] = 0.05;
				 }else if(farthest.contains(sensed)){
					 emissMat[i][j] = 0.025;
				 }else{
					 emissMat[i][j] = 1  - (0.1 + 0.05*closest.size() + 0.025*farthest.size());
				 }
			}
		}
		
		return emissMat;
	}

	private List<List<Tuple>> getXNeighbors(int x, int y) {
		List<Tuple> xClosestN = new ArrayList<Tuple>();
		List<Tuple> xFarthestN = new ArrayList<Tuple>();
		for (int i = -2; i <= 2; i++) {
			for (int j = -2; j <= 2; j++) {
				if (i == 0 && j == 0)
					continue;
				int xInd = x + i;
				int yInd = y + j;
				
				if (xInd >= 0 && yInd >= 0 && xInd < rows && yInd < cols) {
					Tuple n = new Tuple(xInd, yInd);
					if (Math.hypot(x-xInd, y-yInd) < 2) {
						xClosestN.add(n);
					} else {
						xFarthestN.add(n);
					}
				}
			}
		}
		List<List<Tuple>> retList = new ArrayList<List<Tuple>>();
		retList.add(xClosestN);
		retList.add(xFarthestN);
		return retList;
	}
	private void move(){
			heading = calculateNextHeading();
		
		switch (heading) {
			case 0:
			// moving right
				truePos[1]++;
				break;
			case 1:
			//moving down
				truePos[0]++;
				break;
			case 2:
			//moving left
				truePos[1]--;
				break;
			case 3:
			//moving up
				truePos[0]--;
				break;
		}
	}

	private boolean checkWallInCurrHeading(int heading) {
		
		switch (heading) {
		case 0:
		// moving right
			if (truePos[1] + 1 > cols - 1)
				return true;
			break;
		case 1:
		//moving down
			if (truePos[0] + 1 > rows - 1)
				return true;
			break;
		case 2:
		//moving left
			if (truePos[1] - 1 < 0)
				return true;
			break;
		case 3:
		//moving up
			if (truePos[0] - 1 < 0)
				return true;
			break;
		}
		return false;
		
	}

	private int calculateNextHeading() {
		int nextHeading = heading;
		
		float randDir = rand.nextFloat();
		if (randDir < 0.3) {
				nextHeading = rand.nextInt(4);		
		}
		if(checkWallInCurrHeading(nextHeading)) {
			return calculateNextHeading();
		}
		return nextHeading;	
		
//		if (wall) {
//			while (heading == nextHeading) {
//				nextHeading = rand.nextInt(head);
//			}
//		} else {
//			float randDir = rand.nextFloat();
//			if(randDir < 0.3) {
//				while (nextHeading == heading) {
//					nextHeading = rand.nextInt(head);
//				}
//			}
//		}
//		
//		return nextHeading;
		
	}

//	private boolean nextToWall() {
//		 return (truePos[0] == 0 || truePos[0] == (rows - 1) || truePos[1] == 0 || truePos[1] == cols);
//	}
}