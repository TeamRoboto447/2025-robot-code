package frc.robot.controllers;

import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StreamdeckController {
    private AtomicReference<String[]> buttonLabels;

    private NetworkTable panel;
    private NetworkTable states;
    private NetworkTable controlSchemeSubtable;
    private NetworkTableEntry labels;
    private NetworkTableEntry requestUpdate;
    private ControlScheme currentScheme;

    private int ntConnListener;
    private int ntLabelsListener;

    public Boolean isConnected;

    public enum ControlScheme {
        LEGACY,
        MANUAL,
        SEMIAUTO,
        FULLYAUTO
    }

    public StreamdeckController(ControlScheme defaultControlScheme) {
        buttonLabels = new AtomicReference<String[]>();
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        panel = inst.getTable("Streamdeck-Control").getSubTable("panel");
        controlSchemeSubtable = panel.getSubTable("controlScheme");
        requestUpdate = panel.getEntry("robotRequestingUpdate");
        labels = panel.getEntry("labels");
        states = panel.getSubTable("states");
        ntConnListener = inst.addConnectionListener(true, this::connectionListener);
        ntLabelsListener = inst.addListener(labels, EnumSet.of(NetworkTableEvent.Kind.kValueAll), this::labelListener);
        Trigger manual = new Trigger(() -> controlSchemeSubtable.getEntry("manual").getBoolean(false));
        Trigger semiauto = new Trigger(() -> controlSchemeSubtable.getEntry("semi-auto").getBoolean(false));
        Trigger fullyauto = new Trigger(() -> controlSchemeSubtable.getEntry("fully-auto").getBoolean(false));
        Trigger legacy = new Trigger(
                () -> !manual.getAsBoolean() && !semiauto.getAsBoolean() && !fullyauto.getAsBoolean());
        manual.onTrue(Commands.run(() -> this.currentScheme = ControlScheme.MANUAL));
        semiauto.onTrue(Commands.run(() -> this.currentScheme = ControlScheme.SEMIAUTO));
        fullyauto.onTrue(Commands.run(() -> this.currentScheme = ControlScheme.FULLYAUTO));
        legacy.onTrue(Commands.run(() -> this.currentScheme = ControlScheme.LEGACY));

        switch(defaultControlScheme) {
            case FULLYAUTO:
                controlSchemeSubtable.getEntry("fully-auto").setBoolean(true);
                break;
            case LEGACY:
            controlSchemeSubtable.getEntry("semi-auto").setBoolean(true);
                break;
            case MANUAL:
            controlSchemeSubtable.getEntry("manual").setBoolean(true);
                break;
            case SEMIAUTO:
                break;
            default:
                break;
            
        }
        
        if(controlSchemeSubtable.getEntry("manual").getBoolean(false))
            this.currentScheme = ControlScheme.MANUAL;
        if(controlSchemeSubtable.getEntry("semi-auto").getBoolean(false))
            this.currentScheme = ControlScheme.SEMIAUTO;
        if(controlSchemeSubtable.getEntry("fully-auto").getBoolean(false))
            this.currentScheme = ControlScheme.FULLYAUTO;
    }

    private void connectionListener(NetworkTableEvent event) {
        if (event.is(NetworkTableEvent.Kind.kConnected))
            this.isConnected = true;
        this.requestUpdate.setBoolean(true);
        if (event.is(NetworkTableEvent.Kind.kDisconnected))
            this.isConnected = false;
    }

    private void labelListener(NetworkTableEvent event) {
        if (event.valueData.value.isStringArray())
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

    public ControlScheme getCurrentScheme() {
        return this.currentScheme;
    }
}
