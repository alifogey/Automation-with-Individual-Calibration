import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    implicitWidth: 800
    implicitHeight: 480

    /* Exposed to Main.qml */
    property alias runTimeLabel: runTimeLabel
    property alias startButton: startButton
    property alias pauseButton: pauseButton
    property alias stopButton: stopButton
    property alias pauseSelectedButton: pauseSelectedButton
    property alias resumeSelectedButton: resumeSelectedButton

    property alias r1: r1
    property alias r2: r2
    property alias r3: r3
    property alias r4: r4
    property alias r5: r5
    property alias r6: r6
    property alias r7: r7
    property alias r8: r8
    property alias r9: r9

    // ⬇️ keep only this alias, badges are gone
    property alias statusLabel: statusLabel

    property real scaleFactor: (height < 600 ? 0.85 : 1.0)

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8 * scaleFactor
        spacing: 6 * scaleFactor

        // Big timer
        Label {
            id: runTimeLabel
            text: "00:00:00"
            font.pointSize: 26 * scaleFactor
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            Layout.alignment: Qt.AlignHCenter
        }

        // Small status line (step change, done, etc.)
        Label {
            id: statusLabel
            text: ""
            font.pixelSize: 11 * scaleFactor
            color: "#555"
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.WordWrap
        }

        // Controls row
        RowLayout {
            spacing: 8 * scaleFactor
            Layout.alignment: Qt.AlignHCenter

            Button {
                id: startButton
                text: "Start"
                font.pixelSize: 12 * scaleFactor
                Layout.preferredWidth: 80 * scaleFactor
            }
            Button {
                id: pauseButton
                text: "Pause All"
                font.pixelSize: 12 * scaleFactor
                Layout.preferredWidth: 90 * scaleFactor
            }
            Button {
                id: stopButton
                text: "Stop All"
                font.pixelSize: 12 * scaleFactor
                Layout.preferredWidth: 90 * scaleFactor
            }

            Item { Layout.preferredWidth: 16 * scaleFactor }

            Button {
                id: pauseSelectedButton
                text: "Pause Selected"
                font.pixelSize: 11 * scaleFactor
                Layout.preferredWidth: 110 * scaleFactor
            }
            Button {
                id: resumeSelectedButton
                text: "Resume Selected"
                font.pixelSize: 11 * scaleFactor
                Layout.preferredWidth: 120 * scaleFactor
            }
        }

        // Pump grid
        GridLayout {
            columns: 3
            rowSpacing: 8 * scaleFactor
            columnSpacing: 8 * scaleFactor
            Layout.fillWidth: true
            Layout.fillHeight: true

            RunPumpCardForm { id: r1; visible: false; scale: scaleFactor }
            RunPumpCardForm { id: r2; visible: false; scale: scaleFactor }
            RunPumpCardForm { id: r3; visible: false; scale: scaleFactor }
            RunPumpCardForm { id: r4; visible: false; scale: scaleFactor }
            RunPumpCardForm { id: r5; visible: false; scale: scaleFactor }
            RunPumpCardForm { id: r6; visible: false; scale: scaleFactor }
            RunPumpCardForm { id: r7; visible: false; scale: scaleFactor }
            RunPumpCardForm { id: r8; visible: false; scale: scaleFactor }
            RunPumpCardForm { id: r9; visible: false; scale: scaleFactor }
        }
    }
}






