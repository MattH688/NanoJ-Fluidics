package nanoj.pumpControl.java.sequentialProtocol.tabs;

import gnu.io.NRSerialPort;
import nanoj.pumpControl.java.pumps.Pump;
import nanoj.pumpControl.java.pumps.PumpManager;
import nanoj.pumpControl.java.sequentialProtocol.GUI;

import javax.swing.*;
import javax.swing.table.DefaultTableModel;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Vector;
import java.util.prefs.Preferences;

public class PumpConnections extends JPanel {
    private Preferences prefs = Preferences.userRoot().node(this.getClass().getName());
    private GUI gui;
    public String name = "Pump Connections";

    private static final String VIRTUAL_PUMP = "Virtual Pump";
    private static final String VIRTUAL_PORT = "Virtual port";
    private static final String PORT = "com";
    private static final String PUMP = "pump";

    public JComboBox availablePumpsList;
    public DefaultTableModel connectedPumpsTableModel;
    
    JLabel pumpListLabel;
    JLabel connectLabel;
    JComboBox portsList;
    public JButton connectButton;
    public JButton disconnectButton;
    JLabel connectedPumpsLabel;
    private JTable connectedPumpsTable;
    JScrollPane connectedPumpsListPane;

    public PumpConnections(GUI gui) {
        super();
        this.gui = gui;

        pumpListLabel = new JLabel("Pump type");
        connectLabel = new JLabel("Serial port");
        portsList = new JComboBox();
        connectButton = new JButton("Connect");
        disconnectButton = new JButton("Disconnect");
        connectedPumpsLabel = new JLabel("List of currently connected pumps");
        connectedPumpsTableModel = new DefaultTableModel(
                new String[][]{{PumpManager.NO_PUMP_CONNECTED,"",""}},
                new String[]{"Pump","Sub-Pump","COM port"}
        );
        connectedPumpsTable = new JTable(connectedPumpsTableModel);
        connectedPumpsListPane = new JScrollPane(connectedPumpsTable);

        availablePumpsList = new JComboBox(gui.pumpManager.getAvailablePumpsList());
        availablePumpsList.setSelectedItem(prefs.get(PUMP, VIRTUAL_PUMP));
        portsList = new JComboBox(new Vector(NRSerialPort.getAvailableSerialPorts()));
        portsList.addItem(VIRTUAL_PORT);
        portsList.setSelectedItem(prefs.get(PORT, VIRTUAL_PORT));

        PumpConnectionsLayout layout = new PumpConnectionsLayout(this);
        setLayout(layout);

        connectButton.addActionListener(new PumpConnections.Connect());
        disconnectButton.addActionListener(new PumpConnections.Disconnect());

    }

    public void rememberSettings() {
        prefs.put(PORT, (String) portsList.getSelectedItem());
        prefs.put(PUMP, (String) availablePumpsList.getSelectedItem());
    }

    private class Connect implements ActionListener {

        @Override
        public void actionPerformed(ActionEvent e) {
            String port = (String) portsList.getSelectedItem();
            prefs.put(PORT, port);
            try {
                String isItConnected = gui.pumpManager.connect((String) availablePumpsList.getSelectedItem(), port);
                if (isItConnected.equals(PumpManager.PORT_ALREADY_CONNECTED)) {
                    gui.log.message("Tried to connect to " +
                            availablePumpsList.getSelectedItem() + " on port " +
                            port + ", but that port is already in use!");
                    return;
                } else if (isItConnected.equals(Pump.FAILED_TO_CONNECT)) {
                    gui.log.message("Tried to connect to " +
                            availablePumpsList.getSelectedItem() + " on port " +
                            port + ", but there was an error!");
                    return;
                }
                if (connectedPumpsTableModel.getRowCount() > 0) {
                    for (int i = connectedPumpsTableModel.getRowCount() - 1; i > -1; i--) {
                        connectedPumpsTableModel.removeRow(i);
                    }
                }
                String[][] results = gui.pumpManager.getConnectedPumpsList();
                for (String[] result: results){
                    connectedPumpsTableModel.addRow(result);
                }
                gui.updatePumpSelection();
                gui.log.message("Connected to " + gui.pumpManager.getAvailablePumpsList()[availablePumpsList.getSelectedIndex()] +
                        " on port " + port);
            } catch (Exception e1) {
                gui.log.message("Error, can not connect, check core log.");
                e1.printStackTrace();
            }
        }
    }

    private class Disconnect implements ActionListener {

        @Override
        public void actionPerformed(ActionEvent e) {
            boolean success = false;
            if (connectedPumpsTable.getSelectedRow() < 0) {
                return;
            }
            String pump = null;
            String port = null;
            try {
                int selection = connectedPumpsTable.getSelectedRow();
                success = gui.pumpManager.disconnect(selection);
                pump = (String) connectedPumpsTableModel.getValueAt(selection,0);
                port = (String) connectedPumpsTableModel.getValueAt(selection,2);
            } catch (Exception e1) {
                gui.log.message("Error, failed to disconnect properly.");
                e1.printStackTrace();
            }
            if (success) {
                if (connectedPumpsTableModel.getRowCount() > 0) {
                    for (int i = connectedPumpsTableModel.getRowCount() - 1; i > -1; i--) {
                        connectedPumpsTableModel.removeRow(i);
                    }
                }
                String[][] results = gui.pumpManager.getConnectedPumpsList();
                for (String[] result: results){
                    connectedPumpsTableModel.addRow(result);
                }
                gui.log.message("Disconnected from " + pump + " on port " + port + ".");
            }
            gui.updatePumpSelection();
        }
    }

}
