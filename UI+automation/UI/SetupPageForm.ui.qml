import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    implicitWidth: 1200
    implicitHeight: 700

    property alias groupFlowField: groupFlowField
    property alias applyGroupButton: applyGroupButton
    property alias loadPresetButton: loadPresetButton
    property alias savePresetButton: savePresetButton
    property alias readyToRunButton: readyToRunButton
    property alias calibrationButton: calibrationButton

    property alias pump1: pc1
    property alias pump2: pc2
    property alias pump3: pc3
    property alias pump4: pc4
    property alias pump5: pc5
    property alias pump6: pc6
    property alias pump7: pc7
    property alias pump8: pc8
    property alias pump9: pc9

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 12

        Frame {
            Layout.fillWidth: true
            padding: 8

            RowLayout {
                anchors.fill: parent
                spacing: 8

                Label { text: "Flow"; font.pixelSize: 12 }

                TextField {
                    id: groupFlowField
                    text: "0.00"
                    validator: DoubleValidator { bottom: 0; decimals: 2 }
                    Layout.preferredWidth: 90
                    font.pixelSize: 12
                    inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
                }

                Label { text: "µL/min"; font.pixelSize: 12; color: "#555" }

                ToolButton { id: applyGroupButton; text: "Apply to Selected"; font.pixelSize: 12 }
                ToolButton { id: readyToRunButton; text: "Ready to Run"; font.pixelSize: 12 }

                Item { Layout.fillWidth: true }

                ToolButton { id: calibrationButton; text: "Calibration"; font.pixelSize: 12 }
                ToolButton { id: loadPresetButton; text: "Load Preset"; font.pixelSize: 12 }
                ToolButton { id: savePresetButton; text: "Save Preset"; font.pixelSize: 12 }
            }
        }

        GridLayout {
            columns: 3
            rowSpacing: 12
            columnSpacing: 12
            Layout.fillWidth: true
            Layout.fillHeight: true

            PumpCardForm { id: pc1; pumpId: 1; titleLabel.text: "Pump 1" }
            PumpCardForm { id: pc2; pumpId: 2; titleLabel.text: "Pump 2" }
            PumpCardForm { id: pc3; pumpId: 3; titleLabel.text: "Pump 3" }
            PumpCardForm { id: pc4; pumpId: 4; titleLabel.text: "Pump 4" }
            PumpCardForm { id: pc5; pumpId: 5; titleLabel.text: "Pump 5" }
            PumpCardForm { id: pc6; pumpId: 6; titleLabel.text: "Pump 6" }
            PumpCardForm { id: pc7; pumpId: 7; titleLabel.text: "Pump 7" }
            PumpCardForm { id: pc8; pumpId: 8; titleLabel.text: "Pump 8" }
            PumpCardForm { id: pc9; pumpId: 9; titleLabel.text: "Pump 9" }
        }
    }
}









