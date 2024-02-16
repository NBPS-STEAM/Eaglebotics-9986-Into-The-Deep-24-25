package org.firstinspires.ftc.teamcode.helper;

/*
 * This is an enum. Ever wish you could have a true/false statement with more than just true or false?
 * This does that. It's like a boolean, but you can name as many states as you like.
 *
 * To use an enum, you can make a variable, and for the type, name the enum as if it were a class.
 * i.e. private SidePosition side = SidePosition.CENTER;
 * To check the enum, you can use if statements with '=='.
 * i.e. if (side == SidePosition.LEFT) {}
 *      else if (side == SidePosition.RIGHT) {}
 * If you're up to it, you can also look into 'switch' statements. Switch statements are meant to be
 * the "correct" way to check enums, but they're kind of weird to write.
 * If you're confused, there's more important things to learn than switch. You can get by for now with just if-else.
 *
 * It's common practice to name enums like classes, but name enum states in all caps.
 * This makes it clear that they're enum states and not properties.
 *
 * Technical note: behind the scenes, enums are actually just ints. When you run the code, each enum
 * is replaced with a numeric value. If you want to get even more technical, booleans are also just
 * small ints. 'false' is 0 and 'true' is 1.
 */
public enum SidePosition {
    UNDEFINED,
    LEFT,
    CENTER,
    RIGHT,
}
