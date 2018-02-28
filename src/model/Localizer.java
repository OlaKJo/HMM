package model;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.DiagonalMatrix;
import org.apache.commons.math3.linear.RealMatrix;

import control.EstimatorInterface;

public class Localizer implements EstimatorInterface {
		
	private int rows, cols, head;
	//private int[] truePos, sensePos;
	private Tuple truePos, sensePos;
	private Random rand;
	private List<Tuple> closestN;
	private List<Tuple> farthestN;
	private Heading heading;
	private double[] alpha;
	private double[][] T;
	private double[][] bigO;

	public Localizer( int rows, int cols, int head) {
		this.rows = rows;
		this.cols = cols;
		this.head = head;
		truePos = new Tuple(0, 0); 
		sensePos = new Tuple(0, 0);
		closestN = new ArrayList<Tuple>();
		farthestN = new ArrayList<Tuple>();
		rand = new Random();
		truePos.setX(rand.nextInt(rows));
		truePos.setY(rand.nextInt(cols));
		heading = Heading.North;
		alpha = new double[rows*cols*head];
		double val = 1.0/(rows*cols);
		Arrays.fill(alpha, val);
		T = new double[rows*cols*head][rows*cols*head];
		bigO = new double[rows*cols+1][rows*cols*head];
		generateT();
		generateBigO();
	}	
	
	private void generateBigO() {
		for (int x = 0; x < (rows); x++) {
			for (int y = 0; y < cols; y++) {
				bigO [rows*x + y] = getEmissProb(x, y);
			}
		}
		bigO[bigO.length-1] = getEmissProb(-1, -1);
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
	
	public double getTProb( int xt_minus1, int yt_minus1, int ht_minus1, int xt, int yt, int ht) {
		return T[4*cols*xt_minus1+4*yt_minus1+ht_minus1][4*cols*xt+4*yt+ht];

	}

	public double getOrXY( int rX, int rY, int x, int y, int h) {
		//Implement this!
		return 0.1;
	}


	public int[] getCurrentTruePosition() {
		return truePos.getPos();
	}

	public int[] getCurrentReading() {
		return sensePos.getPos();
	}


	public double getCurrentProb( int x, int y) {
		double sum = 0;
		for(int i = 0; i < 4; i++) {
			sum += alpha[x*rows*4 + y*4 + i];
		}
		return sum;
	}
	
	public void update() {
		move();
		updateNeighbors();
		updateSensedPos();
		alpha = updateProb(alpha, sensePos.getX(), sensePos.getY());
	}
	
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
	
	private double[] updateProb(double[] localAlpha, int sX, int sY){
		RealMatrix newAlpha = new BlockRealMatrix(localAlpha.length, 1);

		RealMatrix realAlpha = new BlockRealMatrix(new double[][] {localAlpha});
		realAlpha = realAlpha.transpose();
		
		RealMatrix realT = new BlockRealMatrix(T);
		RealMatrix realO;
		if(sX == -1) {
			realO = new DiagonalMatrix(bigO[bigO.length - 1]);
		} else {
			realO = new DiagonalMatrix(bigO[rows*sX + sY]);
		}
		
		newAlpha = realO.multiply(realT.transpose()).multiply(realAlpha);
		
		
		realAlpha = newAlpha.transpose().scalarMultiply(1/newAlpha.getNorm());
		localAlpha = realAlpha.getData()[0];
		
		return localAlpha;
		
	}
	
	private double getTransProb(int x_tminus1, int y_tminus1, Heading heading_tminus1, int x_t, int y_t, Heading heading_t){
		
		List<Tuple> PathsFromX_tminus1 = getXYNeighbors(x_tminus1, y_tminus1);
		Tuple xt = new Tuple(x_t, y_t);
		Tuple xtminus1 = new Tuple(x_tminus1, y_tminus1);
		Tuple nextInHeading = nextForwardPos(x_tminus1, y_tminus1, heading_tminus1);
		
		double pot = 1;
		
		if(PathsFromX_tminus1.contains(nextInHeading)) {
			pot -= 0.7;
			PathsFromX_tminus1.remove(nextInHeading);
		}
		
		if(xt.equals(nextInHeading) && heading_tminus1 == heading_t) {
			return 0.7;
		}
		
		if(PathsFromX_tminus1.contains(xt) && headingFromTo(xtminus1, xt).equals(heading_t))
			return pot/PathsFromX_tminus1.size();
		
		return 0;
		
	}
	
	private Heading headingFromTo(Tuple xtminus1, Tuple xt) {
		int xDiff = xt.getX() - xtminus1.getX();
		int yDiff = xt.getY() - xtminus1.getY();
		
		if(xDiff > 0) {
			return Heading.South;
		} else if(xDiff < 0) {
			return Heading.North;
		} else if(yDiff > 0) {
			return Heading.East;
		} else{
			return Heading.West;
		}
	}

	private void generateT() {
		int j = 0;
		int i = 0;
		
		for (int xRow = 0; xRow < rows; xRow++) {
			for (int yRow = 0; yRow < cols; yRow++) {
				for(int dirRow = 0; dirRow < 4; dirRow++) {
					
					
					for (int xCol = 0; xCol < rows; xCol++) {
						for (int yCol = 0; yCol < cols; yCol++) {
							for(int dirCol = 0; dirCol < 4; dirCol++) {
								
								T[i][j] = getTransProb(xRow, yRow, Heading.values()[dirRow], xCol, yCol, Heading.values()[dirCol]);
								j++;
							}
						}
					}
					i++;
					j = 0;
				}
			}
		}
	}
	
	private Tuple nextForwardPos(int x, int y, Heading heading) {
		switch (heading) {
		case East:
		// moving right
			y++;
			break;
		case South:
		//moving down
			x++;
			break;
		case West:
		//moving left
			y--;
			break;
		case North:
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

	private double[] getEmissProb(int senseX, int senseY){
		double[] emissVec = new double[alpha.length];
		
		for (int x = 0; x < rows; x++) {
			for (int y = 0; y < cols; y++) {
				if(senseX == x && senseY == y){
					for (int h = 0; h < head; h++) {
						emissVec[x*cols*4 + 4*y + h] = 0.1;
					}
					 continue;
				 }
				if(senseX != -1 && Math.hypot(senseX-x, senseY-y) > Math.sqrt(8)) {
					for (int h = 0; h < head; h++) {
						emissVec[x*cols*4 + 4*y + h] = 0;
					}
					continue;
				}
				 List<List<Tuple>> neighbors = getXNeighbors(x, y);
				 List<Tuple> closest = neighbors.get(0);
				 List<Tuple> farthest = neighbors.get(1);
				
				 Tuple sensed = new Tuple(senseX, senseY);
				 if(closest.contains(sensed)){
					 for (int h = 0; h < head; h++) {
							emissVec[x*cols*4 + 4*y + h] = 0.05;
						}
				 }else if(farthest.contains(sensed)){
					 for (int h = 0; h < head; h++) {
							emissVec[x*cols*4 + 4*y + h] = 0.025;
						}
				 }else{
					 double remainder =  1  - (0.1 + 0.05*closest.size() + 0.025*farthest.size());
					 for (int h = 0; h < head; h++) {
							emissVec[x*cols*4 + 4*y + h] = remainder;
						}
					
				 }
			}
		}
		
		return emissVec;
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
	}

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