#!/usr/bin/env bash

set -e
set -o pipefail

INSTANCE=$1

QGC_DIR=${CDINIT_SESSION_ROOT}/qgroundcontrol_${INSTANCE}
GCS_PORT=$((14550 + INSTANCE))

mkdir -p "${QGC_DIR}/QGroundControl"

(
cat << EOF
[General]
SettingsVersion=9
_deleteBingNoTileTilesDone=true
androidSaveToSDCard=false
appFontPointSize=14
audioMuted=true
firstRunPromptIdsShown="1,2"
savePath="${QGC_DIR}"

[AutoConnect]
autoConnectLibrePilot=false
autoConnectPixhawk=false
autoConnectRTKGPS=false
autoConnectSiKRadio=false
autoConnectUDP=false
autoConnectZeroConf=false

[FlyView]
instrumentQmlFile2=qrc:/qml/QGroundControl/FlightMap/Widgets/HorizontalCompassAttitude.qml

[LinkConfigurations]
Link0\\auto=true
Link0\\high_latency=false
Link0\\host0=127.0.0.1
Link0\\hostCount=1
Link0\\name=px4
Link0\\port=${GCS_PORT}
Link0\\port0=${GCS_PORT}
Link0\\type=1
count=1

[MAVLinkLogGroup]
Description=QGroundControl Session
LogURL=https://logs.px4.io/upload
PublicLog=true
RateKey=notset

[MainWindowState]
height=1166
visibility=4
width=1943
x=1
y=29

[QGCQml]
IsPIPVisible=true

[RadioCalibration]
TransmitterMode=2

[Units]
areaUnits=1
horizontalDistanceUnits=1
speedUnits=1
temperatureUnits=0
verticalDistanceUnits=1
EOF
) > "${QGC_DIR}/QGroundControl/QGroundControl.ini"


env XDG_CONFIG_HOME="${QGC_DIR}" QGroundControl-x86_64.AppImage
