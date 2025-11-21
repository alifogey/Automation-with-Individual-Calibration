import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    implicitWidth: 360
    implicitHeight: 220

    property bool used: false
    property alias titleLabel: titleLabel
    property alias baseFlowLabel: baseFlowLabel

    property alias modeCombo: modeCombo
    property alias shapeCombo: shapeCombo
    property alias periodField: periodField
    property alias dutyField: dutyField
    property alias totalMinutesField: totalMinutesField

    property alias stepEnabledCheck: stepEnabledCheck
    property alias stepMinutesField: stepMinutesField
    property alias stepFlowField: stepFlowField

    // NEW: per-pump pulsatile range
    property alias minFlowField: minFlowField
    property alias maxFlowField: maxFlowField

    Rectangle {
        anchors.fill: parent
        radius: 8
        color: "#ffffff"
        border.color: "#cfd8dc"
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8

        RowLayout {
            spacing: 8
            Label {
                id: titleLabel
                text: "Pump"
                font.bold: true
            }
            Item { Layout.fillWidth: true }
            Label {
                text: "Base flow:"
                font.pixelSize: 11
                color: "#555"
            }
            Label {
                id: baseFlowLabel
                text: "0.00 µL/min"
                font.pixelSize: 11
                font.bold: true
            }
        }

        RowLayout {
            spacing: 8

            Label { text: "Mode:"; font.pixelSize: 12 }

            ComboBox {
                id: modeCombo
                model: [ "Constant", "Pulsatile" ]
                Layout.preferredWidth: 110
                font.pixelSize: 12
            }

            Label {
                text: "Shape:"
                visible: modeCombo.currentText === "Pulsatile"
                font.pixelSize: 12
            }

            ComboBox {
                id: shapeCombo
                model: [ "Square", "Sinusoidal" ]
                Layout.preferredWidth: 110
                font.pixelSize: 12
                visible: modeCombo.currentText === "Pulsatile"
            }
        }

        // Period / duty for pulsatile
        RowLayout {
            spacing: 8

            Label {
                text: "Period (s):"
                visible: modeCombo.currentText === "Pulsatile"
                font.pixelSize: 12
            }

            TextField {
                id: periodField
                text: "2.0"
                validator: DoubleValidator { bottom: 0; decimals: 2 }
                Layout.preferredWidth: 70
                font.pixelSize: 12
                inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
                visible: modeCombo.currentText === "Pulsatile"
            }

            Label {
                text: "Duty (%):"
                visible: modeCombo.currentText === "Pulsatile"
                         && shapeCombo.currentText === "Square"
                font.pixelSize: 12
            }

            TextField {
                id: dutyField
                text: "50"
                validator: DoubleValidator { bottom: 1; top: 99; decimals: 1 }
                Layout.preferredWidth: 60
                font.pixelSize: 12
                inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
                visible: modeCombo.currentText === "Pulsatile"
                         && shapeCombo.currentText === "Square"
            }
        }

        // NEW: min / max range for pulsatile
        RowLayout {
            spacing: 8
            visible: modeCombo.currentText === "Pulsatile"

            Label { text: "Min (µL/min):"; font.pixelSize: 12 }

            TextField {
                id: minFlowField
                text: "0.00"
                validator: DoubleValidator { bottom: 0; decimals: 2 }
                Layout.preferredWidth: 70
                font.pixelSize: 12
                inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
            }

            Label { text: "Max (µL/min):"; font.pixelSize: 12 }

            TextField {
                id: maxFlowField
                text: "0.00"
                validator: DoubleValidator { bottom: 0; decimals: 2 }
                Layout.preferredWidth: 70
                font.pixelSize: 12
                inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
            }
        }

        RowLayout {
            spacing: 8

            Label { text: "Total run time (min):"; font.pixelSize: 12 }

            TextField {
                id: totalMinutesField
                text: "5.0"
                validator: DoubleValidator { bottom: 0; decimals: 2 }
                Layout.preferredWidth: 70
                font.pixelSize: 12
                inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
            }
        }

        RowLayout {
            spacing: 8

            CheckBox {
                id: stepEnabledCheck
                text: "Change flow after time (Constant mode)"
                font.pixelSize: 11
            }
        }

        RowLayout {
            spacing: 8
            enabled: stepEnabledCheck.checked

            Label { text: "After (min):"; font.pixelSize: 12 }

            TextField {
                id: stepMinutesField
                text: "2.0"
                validator: DoubleValidator { bottom: 0; decimals: 2 }
                Layout.preferredWidth: 70
                font.pixelSize: 12
                inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
            }

            Label { text: "new flow (µL/min):"; font.pixelSize: 12 }

            TextField {
                id: stepFlowField
                text: "0.00"
                validator: DoubleValidator { bottom: 0; decimals: 2 }
                Layout.preferredWidth: 80
                font.pixelSize: 12
                inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
            }
        }
    }
}




