import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    implicitWidth: 320
    implicitHeight: 140

    // which UI pump this card represents
    property int pumpId: 0

    // priming state used to toggle button label/color
    property bool priming: false

    // Expose controls to Main.qml
    property alias titleLabel: titleLabel
    property alias enableCheck: enableCheck
    property alias flowField: flowField
    property alias primeButton: primeButton

    Rectangle {
        anchors.fill: parent
        radius: 8
        color: "#ffffff"
        border.color: "#cfd8dc"
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 10

        RowLayout {
            spacing: 8
            CheckBox { id: enableCheck }
            Label {
                id: titleLabel
                text: "Pump 1"
                font.bold: true
            }
            Item { Layout.fillWidth: true }
        }

        RowLayout {
            spacing: 10

            Label {
                text: "Flow:"
                Layout.alignment: Qt.AlignVCenter
            }

            TextField {
                id: flowField
                placeholderText: "0.00"
                text: "0.00"
                validator: DoubleValidator { bottom: 0; decimals: 2 }
                horizontalAlignment: Text.AlignHCenter
                Layout.preferredWidth: 80
                inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
            }

            Label {
                text: "µL/min"
                color: "#555"
                Layout.alignment: Qt.AlignVCenter
            }

            Button {
                id: primeButton
                text: root.priming ? "Priming…" : "Prime"
                Layout.preferredWidth: 80

                background: Rectangle {
                    radius: 4
                    color: root.priming ? "#ffcdd2" : "#e0e0e0"
                }
            }

            Item { Layout.fillWidth: true }
        }
    }
}



