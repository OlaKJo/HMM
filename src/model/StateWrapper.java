package model;

public class StateWrapper {
	
	public Heading heading;
	
	public State north;
	public State south;
	public State east;
	public State west;
	
	private Tuple pos;
	private float emissProb;
	
	@Override public boolean equals(Object obj) {
		if(obj instanceof StateWrapper) {
			return( pos == ((StateWrapper) obj).pos );
		}
		return false;
	}
	
	public void setEmissProb(float prob) {
		emissProb = prob;
	}
	
	public float getEmissProb() {
		return emissProb;
	}

}
