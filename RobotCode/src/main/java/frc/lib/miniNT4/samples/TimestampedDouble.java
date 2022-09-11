package frc.lib.miniNT4.samples;

import java.io.IOException;

import org.msgpack.core.MessageBufferPacker;

public class TimestampedDouble extends TimestampedValue {
    double value;

    public TimestampedDouble(double value, long time){
        this.value = value;
        this.timestamp_us = time;
    }

    @Override
    public String toNiceString() {
        return "{ Time=" + Long.toString(this.timestamp_us) + "us Value=" + Double.toString(this.value) +"}";
    }

    @Override
    public void packValue(MessageBufferPacker packer) throws IOException {
        packer.packDouble(value);
    }

    @Override
    public Double getVal() {
        return value;
    }
    
}
