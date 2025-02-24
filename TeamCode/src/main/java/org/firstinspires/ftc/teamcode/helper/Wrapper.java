package org.firstinspires.ftc.teamcode.helper;

/**
 * If multiple variables are set to the same Wrapper, then the object inside it can be set to a new value and
 * get() will still return the same new value everywhere.
 * <p>Without a Wrapper, if you set multiple variables to the same value, then setting one of those variables
 * to a new value won't update the others.</p>
 * <p>Truly, Java is an amazing piece of engineering.</p>
 */
public class Wrapper<T> {
    private T obj;

    public Wrapper() {
        this(null);
    }

    public Wrapper(T obj) {
        set(obj);
    }

    public void set(T obj) {
        this.obj = obj;
    }

    public T get() {
        return obj;
    }
}
