import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQml 2.15
import Qt.labs.settings 1.1
import QtQuick.VirtualKeyboard 2.4

ApplicationWindow {
    id: app
    visible: true
    width: 1280
    height: 800
    title: "Microfluidic Pump Controller"

    // Pumps chosen for automation (by pump index 1..9)
    property var automationPumpIds: []
    // Pumps currently paused (Run page)
    property var pausedPumpIds: []
    // Run-tab timer seconds
    property int elapsedSec: 0

    // Automation timing / step info for Run tab
    property double automationTotalMinutes: 0.0
    property bool automationFinished: false
    property bool automationHasStep: false
    property double automationStepMinutes: 0.0
    property bool automationStepTriggered: false

    /* ===================== Small helpers ===================== */

    function pad(n) {
        return (n < 10 ? "0" : "") + n;
    }

    function getPumpFlow(pumpCard) {
        if (!pumpCard || !pumpCard.flowField)
            return 0.0;
        var v = parseFloat(pumpCard.flowField.text);
        return isNaN(v) ? 0.0 : v;
    }

    function setPumpFlow(pumpCard, value) {
        if (pumpCard && pumpCard.flowField)
            pumpCard.flowField.text = value.toFixed(2);
    }

    function isPumpSelected(pumpCard) {
        if (!pumpCard)
            return false;
        if (pumpCard.enableCheck)
            return pumpCard.enableCheck.checked;
        if (pumpCard.selectCheckBox)
            return pumpCard.selectCheckBox.checked;
        if (pumpCard.checkBox)
            return pumpCard.checkBox.checked;
        return true;
    }

    function applyGroupFlowToSelected() {
        var v = parseFloat(setup.groupFlowField.text);
        if (isNaN(v) || v < 0)
            return;

        var pumps = [
            setup.pump1, setup.pump2, setup.pump3,
            setup.pump4, setup.pump5, setup.pump6,
            setup.pump7, setup.pump8, setup.pump9
        ];

        for (var i = 0; i < pumps.length; ++i) {
            if (isPumpSelected(pumps[i]))
                setPumpFlow(pumps[i], v);
        }

        console.log("Apply to Selected ->", v, "µL/min");
    }

    function markAutomationCompleteOnRunCards() {
        var runCards = [run.r1, run.r2, run.r3, run.r4,
                        run.r5, run.r6, run.r7, run.r8, run.r9];

        for (var i = 0; i < runCards.length; ++i) {
            var c = runCards[i];
            if (!c || !c.visible)
                continue;

            c.opacity = 0.5;
            if (c.infoLabel && c.infoLabel.text.indexOf("Run complete") === -1)
                c.infoLabel.text += " \u2022 Run complete";
        }
    }

    // Run page helpers
    function pumpIdFromRunCard(c) {
        if (!c)
            return -1;
        if (c.objectName && c.objectName.indexOf("pumpId:") === 0)
            return parseInt(c.objectName.substring(7));
        if (c.titleLabel.text.indexOf("Pump ") === 0)
            return parseInt(c.titleLabel.text.substring(5));
        return -1;
    }

    function selectedRunPumpIds() {
        var ids = [];
        var runCards = [run.r1, run.r2, run.r3, run.r4, run.r5, run.r6, run.r7, run.r8, run.r9];
        for (var i = 0; i < runCards.length; ++i) {
            var c = runCards[i];
            if (!c || !c.visible || !c.selectCheck.checked)
                continue;
            var pid = pumpIdFromRunCard(c);
            if (pid > 0 && ids.indexOf(pid) === -1)
                ids.push(pid);
        }
        return ids;
    }

    /* ===================== Preset storage (Setup flows) ===================== */

    Settings {
        id: presetSettings
        fileName: "presets.ini"
        category: "Presets"
        property string presetStore: "{}"
    }

    property var presetMap: (function () {
        try { return JSON.parse(presetSettings.presetStore); }
        catch(e) { return {}; }
    })()

    function persistPresets() {
        presetSettings.presetStore = JSON.stringify(presetMap);
    }

    function readCurrentConfig() {
        var cards = [setup.pump1, setup.pump2, setup.pump3,
                     setup.pump4, setup.pump5, setup.pump6,
                     setup.pump7, setup.pump8, setup.pump9];
        var flows = [];
        var enabled = [];
        for (var i = 0; i < cards.length; ++i) {
            flows.push(cards[i].flowField.text);
            enabled.push(cards[i].enableCheck ? cards[i].enableCheck.checked : true);
        }
        return { flows: flows, enabled: enabled };
    }

    function applyConfig(cfg) {
        if (!cfg || !cfg.flows || cfg.flows.length !== 9)
            return;
        var cards = [setup.pump1, setup.pump2, setup.pump3,
                     setup.pump4, setup.pump5, setup.pump6,
                     setup.pump7, setup.pump8, setup.pump9];
        for (var i = 0; i < 9; ++i) {
            cards[i].flowField.text = cfg.flows[i];
            if (cfg.enabled && cfg.enabled.length === 9 && cards[i].enableCheck)
                cards[i].enableCheck.checked = cfg.enabled[i];
        }
    }

    ListModel { id: presetNamesModel }

    Dialog {
        id: savePresetDialog
        modal: true
        title: "Save Preset"
        standardButtons: Dialog.Ok | Dialog.Cancel

        contentItem: ColumnLayout {
            anchors.margins: 12
            spacing: 8
            Label { text: "Preset name:" }
            TextField { id: presetNameField; Layout.preferredWidth: 260 }
        }

        onAccepted: {
            var name = presetNameField.text.trim();
            if (!name.length)
                return;
            presetMap[name] = readCurrentConfig();
            persistPresets();
            console.log("Saved preset:", name);
        }
    }

    Dialog {
        id: loadPresetDialog
        modal: true
        title: "Load Preset"
        standardButtons: Dialog.Ok | Dialog.Cancel
        property int selectedIndex: -1

        contentItem: ColumnLayout {
            anchors.margins: 12
            spacing: 8

            ListView {
                id: presetList
                model: presetNamesModel
                Layout.preferredWidth: 260
                Layout.preferredHeight: 200
                delegate: ItemDelegate {
                    width: ListView.view.width
                    text: name
                    onClicked: {
                        loadPresetDialog.selectedIndex = index;
                        presetList.currentIndex = index;
                    }
                }
            }
        }

        function refresh() {
            presetNamesModel.clear();
            var keys = Object.keys(presetMap).sort();
            for (var i = 0; i < keys.length; ++i)
                presetNamesModel.append({ "name": keys[i] });
            selectedIndex = presetNamesModel.count ? 0 : -1;
            presetList.currentIndex = selectedIndex;
        }

        onOpened: refresh()

        onAccepted: {
            if (selectedIndex < 0 || selectedIndex >= presetNamesModel.count)
                return;
            var name = presetNamesModel.get(selectedIndex).name;
            console.log("Load preset:", name);
            applyConfig(presetMap[name]);
        }
    }

    function handleSavePreset() { presetNameField.text = ""; savePresetDialog.open(); }
    function handleLoadPreset() { loadPresetDialog.open(); }

    /* ===================== Calibration (per-pump µL/min per pps) ===================== */

    Settings {
        id: calibrationSettings
        fileName: "calibration.ini"
        category: "Calibration"
        property string perPumpJson: ""   // JSON: [c1, c2, ..., c9]
    }

    // runtime array of 9 factors
    property var pumpCalFactors: (function () {
        try {
            var arr = JSON.parse(calibrationSettings.perPumpJson);
            if (arr && arr.length === 9)
                return arr;
        } catch(e) {}
        var def = [];
        for (var i = 0; i < 9; ++i) def.push(1.0);
        return def;
    })()

    function savePumpCalibration() {
        calibrationSettings.perPumpJson = JSON.stringify(pumpCalFactors);
    }

    function calibrationForPumpId(pid) {
        if (!pumpCalFactors || pid <= 0)
            return 1.0;
        if (pid - 1 >= pumpCalFactors.length)
            return 1.0;
        var f = pumpCalFactors[pid - 1];
        if (!f || f <= 0)
            return 1.0;
        return f;
    }

    function refreshRunPpsFromCalibration() {
        var runCards = [run.r1, run.r2, run.r3, run.r4,
                        run.r5, run.r6, run.r7, run.r8, run.r9];

        for (var i = 0; i < runCards.length; ++i) {
            var c = runCards[i];
            if (!c || !c.visible)
                continue;

            var pid = pumpIdFromRunCard(c);
            if (pid <= 0)
                continue;

            var flowVal = parseFloat(c.setFlowValue.text);
            if (isNaN(flowVal) || flowVal <= 0)
                continue;

            var factor = calibrationForPumpId(pid);
            var pps = factor > 0 ? (flowVal / factor) : 0.0;
            c.ppsLabel.text = pps.toFixed(0);
        }
    }

    Dialog {
        id: calibrationDialog
        modal: true
        title: "Per-pump Calibration (µL/min per pps)"
        standardButtons: Dialog.Ok | Dialog.Cancel

        contentItem: ColumnLayout {
            anchors.margins: 12
            spacing: 8

            Label {
                text: "Enter calibration factors for each pump (µL/min per 1 pps).\n" +
                      "Existing values are kept if you leave a field blank."
                wrapMode: Text.WordWrap
            }

            GridLayout {
                columns: 3
                rowSpacing: 4
                columnSpacing: 10

                Label { text: "Pump 1"; font.pixelSize: 11 }
                TextField {
                    id: cal1; Layout.preferredWidth: 80; font.pixelSize: 11
                    validator: DoubleValidator { bottom: 0; decimals: 5 }
                    inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
                }
                Item {}

                Label { text: "Pump 2"; font.pixelSize: 11 }
                TextField {
                    id: cal2; Layout.preferredWidth: 80; font.pixelSize: 11
                    validator: DoubleValidator { bottom: 0; decimals: 5 }
                    inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
                }
                Item {}

                Label { text: "Pump 3"; font.pixelSize: 11 }
                TextField {
                    id: cal3; Layout.preferredWidth: 80; font.pixelSize: 11
                    validator: DoubleValidator { bottom: 0; decimals: 5 }
                    inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
                }
                Item {}

                Label { text: "Pump 4"; font.pixelSize: 11 }
                TextField {
                    id: cal4; Layout.preferredWidth: 80; font.pixelSize: 11
                    validator: DoubleValidator { bottom: 0; decimals: 5 }
                    inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
                }
                Item {}

                Label { text: "Pump 5"; font.pixelSize: 11 }
                TextField {
                    id: cal5; Layout.preferredWidth: 80; font.pixelSize: 11
                    validator: DoubleValidator { bottom: 0; decimals: 5 }
                    inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
                }
                Item {}

                Label { text: "Pump 6"; font.pixelSize: 11 }
                TextField {
                    id: cal6; Layout.preferredWidth: 80; font.pixelSize: 11
                    validator: DoubleValidator { bottom: 0; decimals: 5 }
                    inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
                }
                Item {}

                Label { text: "Pump 7"; font.pixelSize: 11 }
                TextField {
                    id: cal7; Layout.preferredWidth: 80; font.pixelSize: 11
                    validator: DoubleValidator { bottom: 0; decimals: 5 }
                    inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
                }
                Item {}

                Label { text: "Pump 8"; font.pixelSize: 11 }
                TextField {
                    id: cal8; Layout.preferredWidth: 80; font.pixelSize: 11
                    validator: DoubleValidator { bottom: 0; decimals: 5 }
                    inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
                }
                Item {}

                Label { text: "Pump 9"; font.pixelSize: 11 }
                TextField {
                    id: cal9; Layout.preferredWidth: 80; font.pixelSize: 11
                    validator: DoubleValidator { bottom: 0; decimals: 5 }
                    inputMethodHints: Qt.ImhFormattedNumbersOnly | Qt.ImhPreferNumbers
                }
                Item {}
            }
        }

        onOpened: {
            // populate fields with current values
            var arr = pumpCalFactors;
            cal1.text = "" + arr[0];
            cal2.text = "" + arr[1];
            cal3.text = "" + arr[2];
            cal4.text = "" + arr[3];
            cal5.text = "" + arr[4];
            cal6.text = "" + arr[5];
            cal7.text = "" + arr[6];
            cal8.text = "" + arr[7];
            cal9.text = "" + arr[8];
        }

        onAccepted: {
            // read back; keep old value if field is empty or invalid
            var old = pumpCalFactors;
            var next = [];

            function upd(fieldText, idx) {
                var t = fieldText.trim();
                if (t.length === 0) {
                    next.push(old[idx]);   // keep old
                    return;
                }
                var v = parseFloat(t);
                if (isNaN(v) || v <= 0)
                    next.push(old[idx]);
                else
                    next.push(v);
            }

            upd(cal1.text, 0);
            upd(cal2.text, 1);
            upd(cal3.text, 2);
            upd(cal4.text, 3);
            upd(cal5.text, 4);
            upd(cal6.text, 5);
            upd(cal7.text, 6);
            upd(cal8.text, 7);
            upd(cal9.text, 8);

            pumpCalFactors = next;
            savePumpCalibration();
            refreshRunPpsFromCalibration();
        }
    }

    function openCalibrationDialog() {
        calibrationDialog.open();
    }

    /* ===================== Automation helpers ===================== */

    // copy Setup -> Automation
    function populateAutomationFromSetup() {
        var setupCards = [setup.pump1, setup.pump2, setup.pump3,
                          setup.pump4, setup.pump5, setup.pump6,
                          setup.pump7, setup.pump8, setup.pump9];

        var autoCards = [automation.a1, automation.a2, automation.a3,
                         automation.a4, automation.a5, automation.a6,
                         automation.a7, automation.a8, automation.a9];

        // Clear automation cards
        for (var j = 0; j < autoCards.length; ++j) {
            var ac0 = autoCards[j];
            ac0.used = false;
        }

        automationPumpIds = [];
        var firstUsedIndex = -1;

        for (var i = 0; i < setupCards.length; ++i) {
            var sc = setupCards[i];
            if (!sc || !sc.flowField)
                continue;

            var f = parseFloat(sc.flowField.text);
            if (!isNaN(f) && f > 0) {
                var acard = autoCards[i];     // same index as pump
                acard.used = true;
                acard.titleLabel.text = sc.titleLabel.text;
                acard.baseFlowLabel.text = sc.flowField.text + " µL/min";

                // Defaults
                acard.modeCombo.currentIndex = 0;          // Constant
                acard.shapeCombo.currentIndex = 0;         // Square
                acard.periodField.text = "2.0";
                acard.dutyField.text = "50";
                acard.totalMinutesField.text = "5.0";
                acard.stepEnabledCheck.checked = false;
                acard.stepMinutesField.text = "0.0";
                acard.stepFlowField.text = sc.flowField.text;

                automationPumpIds.push(i + 1);

                if (firstUsedIndex === -1)
                    firstUsedIndex = i;
            }
        }

        if (firstUsedIndex !== -1)
            automation.selectedIndex = firstUsedIndex;

        console.log("Automation pumps:", automationPumpIds);
    }

    // Fill Run page from Automation configuration
    function populateRunFromAutomation() {
        var runCards = [run.r1, run.r2, run.r3, run.r4, run.r5, run.r6, run.r7, run.r8, run.r9];
        var autoCards = [automation.a1, automation.a2, automation.a3,
                         automation.a4, automation.a5, automation.a6,
                         automation.a7, automation.a8, automation.a9];

        // Clear run cards
        for (var k = 0; k < runCards.length; ++k) {
            var rc0 = runCards[k];
            if (!rc0)
                continue;
            rc0.visible = false;
            rc0.selectCheck.checked = false;
            rc0.setFlowValue.text = "0.00";
            rc0.ppsLabel.text = "0";
            if (rc0.infoLabel)
                rc0.infoLabel.text = "";
            rc0.opacity = 1.0;
            rc0.objectName = "";
        }

        var slot = 0;
        var isManual = automation.skipAutomationCheck && automation.skipAutomationCheck.checked;

        for (var i = 0; i < autoCards.length && slot < runCards.length; ++i) {
            var ac = autoCards[i];
            if (!ac || !ac.used)
                continue;

            var rc = runCards[slot++];
            rc.visible = true;
            rc.titleLabel.text = ac.titleLabel.text;

            // Parse base flow from "X.XX µL/min"
            var baseFlowText = ac.baseFlowLabel.text.split(" ")[0];
            var f = parseFloat(baseFlowText);
            if (isNaN(f))
                f = 0.0;
            rc.setFlowValue.text = f.toFixed(2);

            // Pump ID:  matches automationPumpIds
            var pumpId = (i < automationPumpIds.length) ? automationPumpIds[i] : (i + 1);
            rc.objectName = "pumpId:" + pumpId;

            // Compute pps using calibration
            var factor = calibrationForPumpId(pumpId);
            var pps = factor > 0 ? (f / factor) : 0.0;
            rc.ppsLabel.text = pps.toFixed(0);

            // Info string
            var mode = ac.modeCombo.currentText;
            var summary = "";

            if (isManual) {
                if (mode === "Constant") {
                    summary = "Manual mode \u2013 Constant \u2022 No time limit";
                } else {
                    var shapeM = ac.shapeCombo.currentText;
                    summary = "Manual mode \u2013 " + shapeM + " \u2022 No time limit";
                }

            } else {
                var minutes = parseFloat(ac.totalMinutesField.text);
                if (isNaN(minutes) || minutes <= 0)
                    minutes = 0.0;

                if (mode === "Constant") {
                    summary = "Constant \u2022 " + minutes.toFixed(1) + " min";

                    if (ac.stepEnabledCheck.checked) {
                        var stepT = parseFloat(ac.stepMinutesField.text);
                        var stepF = parseFloat(ac.stepFlowField.text);
                        if (!isNaN(stepT) && !isNaN(stepF) && stepT > 0) {
                            summary += " \u2022 Change to " +
                                       stepF.toFixed(2) + " µL/min at " +
                                       stepT.toFixed(1) + " min";
                        }
                    }

                } else {
                    var shape2 = ac.shapeCombo.currentText;
                    var period2 = parseFloat(ac.periodField.text);
                    if (isNaN(period2) || period2 <= 0)
                        period2 = 2.0;

                    summary = shape2 + " pulsatile \u2022 " +
                              minutes.toFixed(1) + " min \u2022 " +
                              period2.toFixed(1) + " s period";

                    if (shape2 === "Square") {
                        var duty = parseFloat(ac.dutyField.text);
                        if (!isNaN(duty))
                            summary += " \u2022 " + duty.toFixed(1) + "% duty";
                    }
                }
            }

            rc.infoLabel.text = summary;
            rc.opacity = 1.0;
        }
    }

    /* ===================== Tabs & Pages ===================== */

    header: TabBar {
        id: tabs
        TabButton { text: "Set up" }
        TabButton { text: "Automation" }
        TabButton { text: "Run" }
    }

    StackLayout {
        id: stack
        anchors.fill: parent
        currentIndex: tabs.currentIndex

        // ---- Setup page ----
        SetupPageForm {
            id: setup
            Layout.fillWidth: true
            Layout.fillHeight: true
        }

        // ---- Automation page ----
        AutomationPageForm {
            id: automation
            Layout.fillWidth: true
            Layout.fillHeight: true
        }

        // ---- Run page ----
        RunPageForm {
            id: run
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }

    /* ===================== Run timer ===================== */

    Timer {
        id: runTimer
        interval: 1000
        repeat: true
        running: false
        onTriggered: {
            // --- update big timer ---
            elapsedSec += 1;
            var h = Math.floor(elapsedSec / 3600);
            var m = Math.floor((elapsedSec % 3600) / 60);
            var s = elapsedSec % 60;
            run.runTimeLabel.text = pad(h) + ":" + pad(m) + ":" + pad(s);

            // --- per-pump step changes (constant mode) ---
            var runCards = [run.r1, run.r2, run.r3, run.r4,
                            run.r5, run.r6, run.r7, run.r8, run.r9];
            var marker = "Change to ";

            for (var i = 0; i < runCards.length; ++i) {
                var c = runCards[i];
                if (!c || !c.visible || !c.infoLabel)
                    continue;

                var text = c.infoLabel.text;

                var idxMarker = text.indexOf(marker);
                if (idxMarker === -1)
                    continue;

                if (text.indexOf("Flow changed to ") !== -1)
                    continue;

                var idxAt  = text.indexOf(" at ", idxMarker + marker.length);
                var idxMin = text.indexOf(" min", idxAt);
                if (idxAt === -1 || idxMin === -1)
                    continue;

                var flowStr = text.substring(idxMarker + marker.length, idxAt).trim();
                var timeStr = text.substring(idxAt + 4, idxMin).trim();

                var stepMinutes = parseFloat(timeStr);
                if (isNaN(stepMinutes) || stepMinutes <= 0)
                    continue;

                if (elapsedSec < stepMinutes * 60)
                    continue;

                var flowVal = parseFloat(flowStr);
                if (!isNaN(flowVal)) {
                    if (c.setFlowValue)
                        c.setFlowValue.text = flowVal.toFixed(2);

                    if (c.ppsLabel) {
                        var pid2_forPps = pumpIdFromRunCard(c);
                        var factor2 = calibrationForPumpId(pid2_forPps);
                        var pps2 = factor2 > 0 ? (flowVal / factor2) : 0.0;
                        c.ppsLabel.text = pps2.toFixed(0);
                    }

                    if (typeof backend !== "undefined" && backend.set_flow) {
                        var pid2 = pumpIdFromRunCard(c);
                        if (pid2 > 0)
                            backend.set_flow(pid2, flowVal);
                    }
                }

                var tag = "Flow changed to " + flowStr + " at " + stepMinutes.toFixed(1) + " min";
                c.infoLabel.text = text + " \u2022 " + tag;

                run.statusLabel.text = "Flow changed at " + stepMinutes.toFixed(1) + " min.";
            }

            // --- Automation finished indicator (timed runs) ---
            if (!automationFinished &&
                automationTotalMinutes > 0 &&
                elapsedSec >= automationTotalMinutes * 60) {

                automationFinished = true;
                runTimer.stop();

                if (typeof backend !== "undefined" && backend.stopAll)
                    backend.stopAll();

                run.statusLabel.text = "Automation complete.";
                markAutomationCompleteOnRunCards();
            }
        }
    }

    // On-screen keyboard for touch use
    InputPanel {
        id: kb
        z: 9999
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        visible: Qt.inputMethod.visible
    }

    onActiveFocusItemChanged: {
        if (activeFocusItem && activeFocusItem.inputMethodHints !== undefined)
            Qt.inputMethod.show();
    }

    /* ===================== Wiring on startup ===================== */

    Component.onCompleted: {
        console.log("Main loaded");

        if (typeof backend !== "undefined" && backend.refreshPorts)
            backend.refreshPorts();

        // Setup buttons
        setup.applyGroupButton.clicked.connect(app.applyGroupFlowToSelected);
        setup.readyToRunButton.clicked.connect(function () {
            app.populateAutomationFromSetup();
            elapsedSec = 0;
            run.runTimeLabel.text = "00:00:00";
            tabs.currentIndex = 1;   // go to Automation tab
        });
        setup.savePresetButton.clicked.connect(app.handleSavePreset);
        setup.loadPresetButton.clicked.connect(app.handleLoadPreset);
        if (setup.calibrationButton)
            setup.calibrationButton.clicked.connect(app.openCalibrationDialog);

        // Prime buttons: toggle full-speed priming
        var setupCards = [setup.pump1, setup.pump2, setup.pump3,
                          setup.pump4, setup.pump5, setup.pump6,
                          setup.pump7, setup.pump8, setup.pump9];

        for (var i = 0; i < setupCards.length; ++i) {
            var card = setupCards[i];
            if (!card || !card.primeButton)
                continue;

            card.priming = false;

            card.primeButton.clicked.connect((function (c) {
                return function () {
                    var pid = c.pumpId;
                    if (pid <= 0)
                        return;

                    c.priming = !c.priming;
                    console.log("Prime toggle for pump", pid, "->", c.priming);

                    if (typeof backend === "undefined")
                        return;

                    if (c.priming) {
                        if (backend.prime)
                            backend.prime(pid);
                    } else {
                        if (backend.stop)
                            backend.stop(pid);
                    }
                }
            })(card));
        }

        // Run buttons
        run.startButton.clicked.connect(function () {
            var runCards = [run.r1, run.r2, run.r3, run.r4,
                            run.r5, run.r6, run.r7, run.r8, run.r9];

            if (typeof backend !== "undefined" && backend.set_flow) {
                for (var i = 0; i < runCards.length; ++i) {
                    var c = runCards[i];
                    if (!c || !c.visible)
                        continue;

                    var pid = pumpIdFromRunCard(c);
                    if (pid <= 0)
                        continue;

                    var f = parseFloat(c.setFlowValue.text);
                    if (isNaN(f) || f <= 0)
                        continue;

                    backend.set_flow(pid, f);   // µL/min directly
                }
            }

            if (!runTimer.running)
                runTimer.start();
        });

        run.pauseButton.clicked.connect(function () {
            runTimer.stop();
            if (typeof backend !== "undefined" && backend.pauseAll)
                backend.pauseAll();
        });

        run.stopButton.clicked.connect(function () {
            runTimer.stop();
            elapsedSec = 0;
            run.runTimeLabel.text = "00:00:00";
            pausedPumpIds = [];
            if (typeof backend !== "undefined" && backend.stopAll)
                backend.stopAll();

            var runCards = [run.r1, run.r2, run.r3, run.r4, run.r5, run.r6, run.r7, run.r8, run.r9];
            for (var j = 0; j < runCards.length; ++j) {
                var c2 = runCards[j];
                if (!c2) continue;
                c2.opacity = 1.0;
                if (c2.infoLabel)
                    c2.infoLabel.text = c2.infoLabel.text.replace(" (paused)", "");
            }

            automationFinished = true;
            automationStepTriggered = false;
            run.statusLabel.text = (automationTotalMinutes === 0.0)
                    ? "Manual run stopped."
                    : "Automation stopped.";
        });

        // Pause selected
        run.pauseSelectedButton.clicked.connect(function () {
            var ids = selectedRunPumpIds();
            console.log("Pause selected pumps:", ids);

            var runCards = [run.r1, run.r2, run.r3, run.r4,
                            run.r5, run.r6, run.r7, run.r8, run.r9];
            for (var j = 0; j < runCards.length; ++j) {
                var c3 = runCards[j];
                if (!c3 || !c3.visible || !c3.selectCheck.checked)
                    continue;
                var pid = pumpIdFromRunCard(c3);
                if (ids.indexOf(pid) !== -1) {
                    c3.opacity = 0.4;
                    if (c3.infoLabel && c3.infoLabel.text.indexOf("(paused)") === -1)
                        c3.infoLabel.text += " (paused)";
                    if (pausedPumpIds.indexOf(pid) === -1)
                        pausedPumpIds.push(pid);
                }
            }

            if (typeof backend !== "undefined" && backend.pausePumps)
                backend.pausePumps(ids);
        });

        // Resume selected
        run.resumeSelectedButton.clicked.connect(function () {
            var ids = selectedRunPumpIds();
            console.log("Resume selected pumps:", ids);

            var runCards = [run.r1, run.r2, run.r3, run.r4,
                            run.r5, run.r6, run.r7, run.r8, run.r9];
            for (var j = 0; j < runCards.length; ++j) {
                var c4 = runCards[j];
                if (!c4 || !c4.visible || !c4.selectCheck.checked)
                    continue;
                var pid = pumpIdFromRunCard(c4);
                if (ids.indexOf(pid) !== -1) {
                    c4.opacity = 1.0;
                    if (c4.infoLabel)
                        c4.infoLabel.text = c4.infoLabel.text.replace(" (paused)", "");
                    var idx = pausedPumpIds.indexOf(pid);
                    if (idx !== -1)
                        pausedPumpIds.splice(idx, 1);
                }
            }

            if (typeof backend !== "undefined" && backend.resumePumps)
                backend.resumePumps(ids);
        });

        // Automation start: populate Run + call backend
        automation.startAutomationButton.clicked.connect(function () {
            if (automationPumpIds.length === 0)
                return;

            console.log("Start Automation clicked (Automation page)");

            var manual = automation.skipAutomationCheck.checked;

            // 1) Fill Run tab from Automation config
            app.populateRunFromAutomation();

            // 2) Reset timer and go to Run tab
            elapsedSec = 0;
            run.runTimeLabel.text = "00:00:00";
            tabs.currentIndex = 2;

            // 3) Derive timing from first used automation card for summary/backend
            var acCards = [automation.a1, automation.a2, automation.a3,
                           automation.a4, automation.a5, automation.a6,
                           automation.a7, automation.a8, automation.a9];
            var ac0 = null;
            for (var j = 0; j < acCards.length; ++j) {
                if (acCards[j] && acCards[j].used) {
                    ac0 = acCards[j];
                    break;
                }
            }
            if (!ac0)
                return;

            var mode = ac0.modeCombo.currentText;
            var minutes = parseFloat(ac0.totalMinutesField.text);
            if (isNaN(minutes) || minutes <= 0)
                minutes = 5.0;

            var shape = "";
            var period = 0.0;
            var duty = 0.0;

            if (mode === "Pulsatile") {
                shape = ac0.shapeCombo.currentText;
                period = parseFloat(ac0.periodField.text);
                if (isNaN(period) || period <= 0)
                    period = 2.0;

                if (shape === "Square") {
                    duty = parseFloat(ac0.dutyField.text);
                    if (isNaN(duty) || duty <= 0 || duty >= 100)
                        duty = 50.0;
                    duty = duty / 100.0;   // fraction
                }
            }

            automationTotalMinutes = manual ? 0.0 : minutes;
            automationStepTriggered = false;
            automationFinished = false;

            if (!manual &&
                mode === "Constant" &&
                ac0.stepEnabledCheck.checked) {

                var stepM = parseFloat(ac0.stepMinutesField.text);
                if (!isNaN(stepM) && stepM > 0) {
                    automationHasStep = true;
                    automationStepMinutes = stepM;
                } else {
                    automationHasStep = false;
                    automationStepMinutes = 0.0;
                }
            } else {
                automationHasStep = false;
                automationStepMinutes = 0.0;
            }

            if (manual) {
                run.statusLabel.text = "Manual control: use Start / Pause / Stop.";
            } else {
                var msg = "Automation running: " + minutes.toFixed(1) + " min total.";
                if (automationHasStep)
                    msg += " Step change at " + automationStepMinutes.toFixed(1) + " min.";
                run.statusLabel.text = msg;
            }

            if (typeof backend !== "undefined"
                    && backend.startAutomation
                    && !manual) {

                if (mode === "Constant") {
                    backend.startAutomation(
                                automationPumpIds,
                                "Constant",
                                "",
                                minutes,
                                0.0,
                                0.0);
                } else {
                    backend.startAutomation(
                                automationPumpIds,
                                "Pulsatile",
                                shape,
                                minutes,
                                period,
                                duty);
                }
            }
        });
    }
}
























