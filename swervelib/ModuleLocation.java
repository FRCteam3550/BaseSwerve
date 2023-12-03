package swervelib;

public enum ModuleLocation {
    FrontLeft(0, "Front left"),
    FrontRight(1, "Front right"),
    BackLeft(2, "Back left"),
    BackRight(3, "Back right");

    public final int index;
    public final String name;

    private ModuleLocation(int index, String name) {
        this.index = index;
        this.name = name;
    }
}
