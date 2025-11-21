import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    implicitWidth: 260
    implicitHeight: 120

    property alias selectCheck: selectCheck
    property alias titleLabel: titleLabel
    property alias setFlowValue: setFlowValue
    property alias ppsLabel: ppsLabel
    property alias infoLabel: infoLabel

    Rectangle {
        anchors.fill: parent
        radius: 8
        color: "#ffffff"
        border.color: "#cfd8dc"
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 4

        RowLayout {
            spacing: 4
            CheckBox { id: selectCheck }
            Label {
                id: titleLabel
                text: "Pump"
                font.bold: true
                font.pixelSize: 12
            }
            Item { Layout.fillWidth: true }
        }

        RowLayout {
            spacing: 4
            Label { text: "Set:"; font.pixelSize: 11 }

            Label {
                id: setFlowValue
                text: "0.00"
                font.bold: true
                font.pixelSize: 11
            }

            Label {
                text: "µL/min"
                font.pixelSize: 11
                color: "#555"
            }

            Label {
                text: "•"
                font.pixelSize: 11
                color: "#999"
            }

            Label {
                text: "pps:"
                font.pixelSize: 11
            }

            Label {
                id: ppsLabel
                text: "0"
                font.bold: true
                font.pixelSize: 11
            }

            Item { Layout.fillWidth: true }
        }

        Label {
            id: infoLabel
            text: ""      // e.g. "Constant • 5.0 min • Change to 22.00 µL/min at 1.0 min • Flow changed..."
            font.pixelSize: 11
            color: "#666"
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            horizontalAlignment: Text.AlignLeft
            // optional: limit lines so it never gets crazy tall
            // maximumLineCount: 3
            // elide: Text.ElideRight
        }
    }
}






