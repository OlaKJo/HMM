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
	
	public void setX(int x) {
		this.x = x;
	}
	
	public void setY(int y) {
		this.y = y;
	}
	
	public void setPos(int x, int y) {
		setX(x);
		setY(y);
	}
	
	public void setPos(Tuple other ) {
		setX(other.getX());
		setY(other.getY());
	}
	
	public int[] getPos() {
		int[] ret = new int[2];
		ret[0] = x;
		ret[1] = y;
		return ret;
	}
	
	public void addX(int n) {
		x += n;
	}
	
	public void addY(int n) {
		y += n;
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
