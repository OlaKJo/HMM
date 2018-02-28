package model;

public enum Heading {
    East (0),
    South (1),
    West (2),
    North (3);

    private final int value;
    
    Heading(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}