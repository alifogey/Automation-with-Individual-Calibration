import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    implicitWidth: 800
    implicitHeight: 480

    property alias startAutomationButton: startAutomationButton
    property alias skipAutomationCheck: skipAutomationCheck

    property alias a1: a1
    property alias a2: a2
    property alias a3: a3
    property alias a4: a4
    property alias a5: a5
    property alias a6: a6
    property alias a7: a7
    property alias a8: a8
    property alias a9: a9

    // Which pump index (0..8) is currently selected for editing
    property int selectedIndex: 0

    // Tiny scaling for Pi screen
    property real scaleFactor: (height < 600 ? 0.85 : 1.0)

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8 * scaleFactor
        spacing: 8 * scaleFactor

        // Top bar: title, skip automation, start button
        Frame {
            Layout.fillWidth: true
            padding: 6 * scaleFactor

            RowLayout {
                anchors.fill: parent
                spacing: 8 * scaleFactor

                Label {
                    text: "Automation"
                    font.pixelSize: 14 * scaleFactor
                    font.bold: true
                }

                Item { Layout.fillWidth: true }

                CheckBox {
                    id: skipAutomationCheck
                    text: "Skip automation (manual mode)"
                    font.pixelSize: 11 * scaleFactor
                }

                Button {
                    id: startAutomationButton
                    text: "Start"
                    font.pixelSize: 12 * scaleFactor
                    Layout.preferredWidth: 90 * scaleFactor
                }
            }
        }

        // Middle area: list of pumps on left, selected pump card on right
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 8 * scaleFactor

            // LEFT COLUMN: pump selector
            Frame {
                Layout.preferredWidth: 150 * scaleFactor
                Layout.fillHeight: true
                padding: 6 * scaleFactor

                ColumnLayout {
                    anchors.fill: parent
                    spacing: 4 * scaleFactor

                    Label {
                        text: "Pumps"
                        font.pixelSize: 12 * scaleFactor
                        font.bold: true
                    }

                    // Each button only visible if that pump is 'used'
                    Button {
                        text: a1.titleLabel.text
                        visible: a1.used
                        checkable: true
                        checked: root.selectedIndex === 0
                        font.pixelSize: 11 * scaleFactor
                        onClicked: root.selectedIndex = 0
                    }
                    Button {
                        text: a2.titleLabel.text
                        visible: a2.used
                        checkable: true
                        checked: root.selectedIndex === 1
                        font.pixelSize: 11 * scaleFactor
                        onClicked: root.selectedIndex = 1
                    }
                    Button {
                        text: a3.titleLabel.text
                        visible: a3.used
                        checkable: true
                        checked: root.selectedIndex === 2
                        font.pixelSize: 11 * scaleFactor
                        onClicked: root.selectedIndex = 2
                    }
                    Button {
                        text: a4.titleLabel.text
                        visible: a4.used
                        checkable: true
                        checked: root.selectedIndex === 3
                        font.pixelSize: 11 * scaleFactor
                        onClicked: root.selectedIndex = 3
                    }
                    Button {
                        text: a5.titleLabel.text
                        visible: a5.used
                        checkable: true
                        checked: root.selectedIndex === 4
                        font.pixelSize: 11 * scaleFactor
                        onClicked: root.selectedIndex = 4
                    }
                    Button {
                        text: a6.titleLabel.text
                        visible: a6.used
                        checkable: true
                        checked: root.selectedIndex === 5
                        font.pixelSize: 11 * scaleFactor
                        onClicked: root.selectedIndex = 5
                    }
                    Button {
                        text: a7.titleLabel.text
                        visible: a7.used
                        checkable: true
                        checked: root.selectedIndex === 6
                        font.pixelSize: 11 * scaleFactor
                        onClicked: root.selectedIndex = 6
                    }
                    Button {
                        text: a8.titleLabel.text
                        visible: a8.used
                        checkable: true
                        checked: root.selectedIndex === 7
                        font.pixelSize: 11 * scaleFactor
                        onClicked: root.selectedIndex = 7
                    }
                    Button {
                        text: a9.titleLabel.text
                        visible: a9.used
                        checkable: true
                        checked: root.selectedIndex === 8
                        font.pixelSize: 11 * scaleFactor
                        onClicked: root.selectedIndex = 8
                    }

                    Item { Layout.fillHeight: true }
                }
            }

            // RIGHT: show ONLY the selected pump card, full-height
            Frame {
                Layout.fillWidth: true
                Layout.fillHeight: true
                padding: 4 * scaleFactor

                Item {
                    id: pumpArea
                    anchors.fill: parent

                    AutomationPumpCardForm {
                        id: a1
                        anchors.fill: parent
                        visible: used && root.selectedIndex === 0
                        scale: scaleFactor
                    }
                    AutomationPumpCardForm {
                        id: a2
                        anchors.fill: parent
                        visible: used && root.selectedIndex === 1
                        scale: scaleFactor
                    }
                    AutomationPumpCardForm {
                        id: a3
                        anchors.fill: parent
                        visible: used && root.selectedIndex === 2
                        scale: scaleFactor
                    }
                    AutomationPumpCardForm {
                        id: a4
                        anchors.fill: parent
                        visible: used && root.selectedIndex === 3
                        scale: scaleFactor
                    }
                    AutomationPumpCardForm {
                        id: a5
                        anchors.fill: parent
                        visible: used && root.selectedIndex === 4
                        scale: scaleFactor
                    }
                    AutomationPumpCardForm {
                        id: a6
                        anchors.fill: parent
                        visible: used && root.selectedIndex === 5
                        scale: scaleFactor
                    }
                    AutomationPumpCardForm {
                        id: a7
                        anchors.fill: parent
                        visible: used && root.selectedIndex === 6
                        scale: scaleFactor
                    }
                    AutomationPumpCardForm {
                        id: a8
                        anchors.fill: parent
                        visible: used && root.selectedIndex === 7
                        scale: scaleFactor
                    }
                    AutomationPumpCardForm {
                        id: a9
                        anchors.fill: parent
                        visible: used && root.selectedIndex === 8
                        scale: scaleFactor
                    }
                }
            }
        }
    }
}





