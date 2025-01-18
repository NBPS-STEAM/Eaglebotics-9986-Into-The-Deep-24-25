package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * A class that is accepted as a REV Color/Range Sensor but doesn't do anything.
 * NullColorRangeSensors can be instantiated freely and don't correspond to any physical hardware.
 * These are useful as placeholders to take the place of missing sensors.
 * Color and light readings always come out to zero. Distance readings always come out to {@link DistanceUnit#infinity}.
 */
public class NullColorRangeSensor implements ColorRangeSensor {
    @Override
    public int red() {
        return 0;
    }

    @Override
    public int green() {
        return 0;
    }

    @Override
    public int blue() {
        return 0;
    }

    @Override
    public int alpha() {
        return 0;
    }

    @Override
    public int argb() {
        return 0;
    }

    @Override
    public double getLightDetected() {
        return 0;
    }

    @Override
    public double getRawLightDetected() {
        return 0;
    }

    @Override
    public double getRawLightDetectedMax() {
        return 0;
    }

    @Override
    public void enableLed(boolean b) {

    }

    @Override
    public String status() {
        return "Null Color Range Sensor Object: No Real Hardware";
    }

    private I2cAddr phI2cAddr = null;
    @Override
    public void setI2cAddress(I2cAddr i2cAddr) {
        phI2cAddr = i2cAddr;
    }

    @Override
    public I2cAddr getI2cAddress() {
        return phI2cAddr;
    }

    @Override
    public double getDistance(DistanceUnit distanceUnit) {
        return DistanceUnit.infinity;
    }

    @Override
    public NormalizedRGBA getNormalizedColors() {
        return new NormalizedRGBA();
    }

    private float phGain = 0;
    @Override
    public float getGain() {
        return phGain;
    }

    @Override
    public void setGain(float v) {
        phGain = v;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "Null Color/Range Sensor Object";
    }

    @Override
    public String getConnectionInfo() {
        return "None";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
