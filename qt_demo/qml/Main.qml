import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15

ApplicationWindow {
    visible: true
    width: 720; height: 600
    title: "Libi Perception Viewer"
    color: "#0f1420"

    Column {
        anchors.centerIn: parent
        spacing: 16

        Rectangle {
            width: 640; height: 480
            color: "#000"; border.color: "#2a3550"; border.width: 1
            Image {
                anchors.fill: parent
                fillMode: Image.PreserveAspectFit
                cache: false
                source: client.frameCounter > 0
                        ? "image://perc/frame?c=" + client.frameCounter : ""
            }
            Text {
                anchors.centerIn: parent
                visible: client.frameCounter === 0
                text: client.connected ? "연결됨 — 프레임 대기" : "서버 연결 대기…"
                color: "#88aacc"; font.pixelSize: 18
            }
        }

        Row {
            spacing: 16
            anchors.horizontalCenter: parent.horizontalCenter
            Button { text: "등록"; onClicked: client.doRegister() }
            Button { text: "리셋"; onClicked: client.doReset() }
            Label {
                anchors.verticalCenter: parent.verticalCenter
                text: client.connected ? "● 연결됨" : "○ 끊김"
                color: client.connected ? "#4caf50" : "#f44336"
                font.pixelSize: 16
            }
        }
    }
}
