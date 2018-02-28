package model;

public enum Heading {
	North (0),
	East (1),
    South (2),
    West (3);
    

    private final int value;
    
    Heading(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}