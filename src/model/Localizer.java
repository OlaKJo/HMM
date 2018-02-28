package model;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.DiagonalMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import control.EstimatorInterface;


public class Localizer implements EstimatorInterface {
		
	private int rows, cols, head;
	private Tuple sensePos;
	
	private double[] alpha;
	private double[][] T;
	private double[][] bigO;
	private Robot marvin;

	/**
	 * Class which uses HMM and forward filtering to estimate the position of a Robot object in a world of size rows x cols.
	 * @param rows
	 * @param cols
	 * @param head
	 * @param m
	 */
	public Localizer( int rows, int cols, int head, Robot m) {
		this.rows = rows;
		this.cols = cols;
		this.head = head;
		marvin = m;
		
		initialize();
		generateT();
		generateBigO();
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
		return bigO[rows*rX + rY][head*rows*x + 4*y + h];		
	}


	public int[] getCurrentTruePosition() {
		Tuple t =  marvin.getTruePos();
		return new int[] {t.getX(), t.getY()};
	}

	public int[] getCurrentReading() {
		Tuple t =  marvin.getSensedPos();
		return new int[] {t.getX(), t.getY()};
	}


	public double getCurrentProb( int x, int y) {
		double sum = 0;
		for(int i = 0; i < 4; i++) {
			sum += alpha[x*rows*4 + y*4 + i];
		}
		return sum;
	}
	
	public void update() {
		sensePos = marvin.update();
		alpha = updateAlphaProb(alpha, sensePos.getX(), sensePos.getY());
	}
	
	/**
	 * initialize the alpha matrix to an even probability dist for the first step
	 * set the sizes of the transition matrix and the collection of diagonal matrices in bigO
	 */
	private void initialize() {
		alpha = new double[rows*cols*head];
		double val = 1.0/(rows*cols);
		Arrays.fill(alpha, val);
		
		T = new double[rows*cols*head][rows*cols*head];
		bigO = new double[rows*cols+1][rows*cols*head];		
	}


	/**
	 * Calculate all the emission probibilities, the diagonal matrices, and save the values as rows in bigO
	 */
	private void generateBigO() {
		for (int x = 0; x < (rows); x++) {
			for (int y = 0; y < cols; y++) {
				bigO [rows*x + y] = getEmissProb(x, y);
			}
		}
		bigO[bigO.length-1] = getEmissProb(-1, -1);
	}

	/**
	 * Implementation of Forward filtering HMM using 
	 * ft = norm_factor*O*T_transpose*ftminus1
	 * ft = newAlpha
	 * ftminus1 = realAlpha
	 * O = realO
	 * T_transpose = realT
	 * 
	 * @param localAlpha
	 * @param sX
	 * @param sY
	 * @return
	 */
	private double[] updateAlphaProb(double[] localAlpha, int sX, int sY){
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
	
	/**
	 * return the probability of moving from (x_tminus1, y_tminus1) in headin_tminus1, and ending up in (x_t, y_t) in heading_t
	 * @param x_tminus1
	 * @param y_tminus1
	 * @param heading_tminus1
	 * @param x_t
	 * @param y_t
	 * @param heading_t
	 * @return
	 */
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
	
	/**
	 * helper method which calculates the heading from one tuple to the second
	 * @param xtminus1
	 * @param xt
	 * @return
	 */
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

	/**
	 * Generates the T matrix, only needs to be run once
	 */
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
	
	/**
	 * @param x
	 * @param y
	 * @param heading
	 * @return
	 */
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

	/**
	 * returns the positions that are reachable in the next step from a given position 
	 * @param x
	 * @param y
	 * @return
	 */
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

	/**
	 * calculates the emission probability for all positions given the input sensory position
	 * returns as an array as follows: {{0,0}N, {0,0}E, {0,0}S, {0,0}W, {0,1}N, ... , {rows-1, cols-1}W}
	 * @param senseX
	 * @param senseY
	 * @return
	 */
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
				 List<List<Tuple>> neighbors = getNeighbors(x, y);
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

	/**
	 * Returns a list of the closest AND farthest neighbors of (x,y) using the distance between tuples
	 * @param x
	 * @param y
	 * @return
	 */
	private List<List<Tuple>> getNeighbors(int x, int y) {
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
}