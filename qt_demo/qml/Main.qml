import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15

ApplicationWindow {
    visible: true
    width: 720; height: 600
    title: "Libi Perception Viewer"
    color: "#0f1420"

    function distColor(cm) {
        if (cm < 0) return "#5a6a85";      // no reading
        if (cm < 30) return "#f44336";     // danger
        if (cm < 60) return "#ff9800";     // warn
        return "#4caf50";                  // clear
    }

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
            spacing: 28
            anchors.horizontalCenter: parent.horizontalCenter

            Column {                                  // 등록/리셋 + 연결 상태
                spacing: 8
                anchors.verticalCenter: parent.verticalCenter
                Row {
                    spacing: 12
                    Button { text: "등록"; onClicked: client.doRegister() }
                    Button { text: "리셋"; onClicked: client.doReset() }
                }
                Label {
                    text: client.connected ? "● 연결됨" : "○ 끊김"
                    color: client.connected ? "#4caf50" : "#f44336"
                    font.pixelSize: 15
                }
            }

            Grid {                                    // LiDAR 전/후/좌/우 (cm)
                columns: 3
                columnSpacing: 4; rowSpacing: 2
                anchors.verticalCenter: parent.verticalCenter

                Item { width: 58; height: 30 }
                Column {
                    width: 58
                    Text { anchors.horizontalCenter: parent.horizontalCenter; text: "전"; color: "#88aacc"; font.pixelSize: 12 }
                    Text { anchors.horizontalCenter: parent.horizontalCenter
                           text: client.lidarFront >= 0 ? client.lidarFront + " cm" : "—"
                           color: distColor(client.lidarFront); font.pixelSize: 14; font.bold: true }
                }
                Item { width: 58; height: 30 }

                Column {
                    width: 58
                    Text { anchors.horizontalCenter: parent.horizontalCenter; text: "좌"; color: "#88aacc"; font.pixelSize: 12 }
                    Text { anchors.horizontalCenter: parent.horizontalCenter
                           text: client.lidarLeft >= 0 ? client.lidarLeft + " cm" : "—"
                           color: distColor(client.lidarLeft); font.pixelSize: 14; font.bold: true }
                }
                Item { width: 58; height: 30 }
                Column {
                    width: 58
                    Text { anchors.horizontalCenter: parent.horizontalCenter; text: "우"; color: "#88aacc"; font.pixelSize: 12 }
                    Text { anchors.horizontalCenter: parent.horizontalCenter
                           text: client.lidarRight >= 0 ? client.lidarRight + " cm" : "—"
                           color: distColor(client.lidarRight); font.pixelSize: 14; font.bold: true }
                }

                Item { width: 58; height: 30 }
                Column {
                    width: 58
                    Text { anchors.horizontalCenter: parent.horizontalCenter; text: "후"; color: "#88aacc"; font.pixelSize: 12 }
                    Text { anchors.horizontalCenter: parent.horizontalCenter
                           text: client.lidarBack >= 0 ? client.lidarBack + " cm" : "—"
                           color: distColor(client.lidarBack); font.pixelSize: 14; font.bold: true }
                }
                Item { width: 58; height: 30 }
            }
        }
    }
}
