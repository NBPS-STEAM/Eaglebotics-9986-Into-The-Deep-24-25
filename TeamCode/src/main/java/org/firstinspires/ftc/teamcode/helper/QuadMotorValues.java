package org.firstinspires.ftc.teamcode.helper;

/*
 * This is a class which represents a set of motor power values. It records an amount of power for four driving motors.
 *
 * <T> is a special thing called a type parameter. When a QuadMotorValues object is created, a type
 * is given to be 'T'. All variables and objects used below that are type 'T' will become this type.
 * It's an easy way to make a class work for multiple types of values; for example, in this project,
 * QuadMotorValues is used in some places with Double values and others with Integer values.
 *
 * The type parameter is given when you create an object from the class. The type of 'T' doesn't
 * change after the object is created.
 *
 * The only requirement of type parameters is that it cannot be a primitive type. You must use the
 * "bubbled" class version of the type.
 * i.e. 'T' can be an Integer, but not an int.
 * The difference is that Integer is handled by Java like a class, while int is a primitive.
 * That makes Integer slightly slower (an extremely small amount), but sometimes the behavior is
 * preferred (such as in this case, where primitives can't be used for technical reasons).
 */
public class QuadMotorValues<T> {

    // Private instance variables
    private T frontLeftValue;
    private T frontRightValue;
    private T backLeftValue;
    private T backRightValue;


    // Constructor method
    public QuadMotorValues(T frontLeftValue, T frontRightValue, T backLeftValue, T backRightValue) {
        this.frontLeftValue = frontLeftValue;
        this.frontRightValue = frontRightValue;
        this.backLeftValue = backLeftValue;
        this.backRightValue = backRightValue;
    }

    // Getter methods
    public T getFrontLeftValue() {
        return frontLeftValue;
    }

    public T getFrontRightValue() {
        return frontRightValue;
    }

    public T getBackLeftValue() {
        return backLeftValue;
    }

    public T getBackRightValue() {
        return backRightValue;
    }

    // Setter methods
    public void setFrontLeftValue(T frontLeftValue) {
        this.frontLeftValue = frontLeftValue;
    }

    public void setFrontRightValue(T frontRightValue) {
        this.frontRightValue = frontRightValue;
    }

    public void setBackLeftValue(T backLeftValue) {
        this.backLeftValue = backLeftValue;
    }

    public void setBackRightValue(T backRightValue) {
        this.backRightValue = backRightValue;
    }
}
