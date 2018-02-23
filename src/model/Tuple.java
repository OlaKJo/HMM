package model;

public class Tuple {

	private int x;
	private int y;
	
	public Tuple(int x, int y){
		this.x = x;
		this.y = y;
	}
	
	public int getX(){
		return x;
	}
	
	public int getY(){
		return y;
	}
	
	@Override public boolean equals(Object obj) {
		if(obj instanceof Tuple) {
			return( x == ((Tuple) obj).x && y == ((Tuple) obj).y);
		}
		return false;
	}
	
	public String toString() {
		return "(" + x + ", " + y + ")";
	}
}
