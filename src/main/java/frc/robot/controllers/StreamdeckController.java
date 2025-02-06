package frc.robot.controllers;

import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class StreamdeckController {
    private AtomicReference<String[]> buttonLabels;

    private NetworkTable panel;
    private NetworkTable states;
    private NetworkTableEntry labels;
    private NetworkTableEntry requestUpdate;

    private int ntConnListener;
    private int ntLabelsListener;

    public Boolean isConnected;

    public StreamdeckController() {
        buttonLabels = new AtomicReference<String[]>();
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        panel = inst.getTable("Streamdeck-Control").getSubTable("panel");
        requestUpdate = panel.getEntry("robotRequestingUpdate");
        labels = panel.getEntry("labels");
        states = panel.getSubTable("states");
        ntConnListener = inst.addConnectionListener(true, this::connectionListener);
        ntLabelsListener = inst.addListener(labels, EnumSet.of(NetworkTableEvent.Kind.kValueAll), this::labelListener);
    }

    private void connectionListener(NetworkTableEvent event) {
        if(event.is(NetworkTableEvent.Kind.kConnected))
            this.isConnected = true;
            this.requestUpdate.setBoolean(true);
        if(event.is(NetworkTableEvent.Kind.kDisconnected))
            this.isConnected = false;
    }

    private void labelListener(NetworkTableEvent event) {
        if(event.valueData.value.isStringArray())
           buttonLabels.set(event.valueData.value.getStringArray());
    }

    public String[] getLabels() {
        return buttonLabels.get();
    }

    public BooleanSupplier getButton(String label) {
        return () -> states.getEntry(label).getBoolean(false);
    }

    public void close() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.removeListener(ntConnListener);
        inst.removeListener(ntLabelsListener);
    }
}
