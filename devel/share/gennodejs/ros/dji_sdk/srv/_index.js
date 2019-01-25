
"use strict";

let MissionWpSetSpeed = require('./MissionWpSetSpeed.js')
let MissionHpUpdateYawRate = require('./MissionHpUpdateYawRate.js')
let DroneArmControl = require('./DroneArmControl.js')
let Activation = require('./Activation.js')
let StereoVGASubscription = require('./StereoVGASubscription.js')
let MFIOConfig = require('./MFIOConfig.js')
let DroneTaskControl = require('./DroneTaskControl.js')
let MissionWpGetSpeed = require('./MissionWpGetSpeed.js')
let MissionWpUpload = require('./MissionWpUpload.js')
let SDKControlAuthority = require('./SDKControlAuthority.js')
let StereoDepthSubscription = require('./StereoDepthSubscription.js')
let CameraAction = require('./CameraAction.js')
let MissionWpAction = require('./MissionWpAction.js')
let QueryDroneVersion = require('./QueryDroneVersion.js')
let MissionHpGetInfo = require('./MissionHpGetInfo.js')
let MissionHpUpload = require('./MissionHpUpload.js')
let Stereo240pSubscription = require('./Stereo240pSubscription.js')
let MissionHpUpdateRadius = require('./MissionHpUpdateRadius.js')
let SetHardSync = require('./SetHardSync.js')
let SetLocalPosRef = require('./SetLocalPosRef.js')
let SetupCameraStream = require('./SetupCameraStream.js')
let MissionWpGetInfo = require('./MissionWpGetInfo.js')
let MissionStatus = require('./MissionStatus.js')
let MissionHpAction = require('./MissionHpAction.js')
let MFIOSetValue = require('./MFIOSetValue.js')
let SendMobileData = require('./SendMobileData.js')
let MissionHpResetYaw = require('./MissionHpResetYaw.js')

module.exports = {
  MissionWpSetSpeed: MissionWpSetSpeed,
  MissionHpUpdateYawRate: MissionHpUpdateYawRate,
  DroneArmControl: DroneArmControl,
  Activation: Activation,
  StereoVGASubscription: StereoVGASubscription,
  MFIOConfig: MFIOConfig,
  DroneTaskControl: DroneTaskControl,
  MissionWpGetSpeed: MissionWpGetSpeed,
  MissionWpUpload: MissionWpUpload,
  SDKControlAuthority: SDKControlAuthority,
  StereoDepthSubscription: StereoDepthSubscription,
  CameraAction: CameraAction,
  MissionWpAction: MissionWpAction,
  QueryDroneVersion: QueryDroneVersion,
  MissionHpGetInfo: MissionHpGetInfo,
  MissionHpUpload: MissionHpUpload,
  Stereo240pSubscription: Stereo240pSubscription,
  MissionHpUpdateRadius: MissionHpUpdateRadius,
  SetHardSync: SetHardSync,
  SetLocalPosRef: SetLocalPosRef,
  SetupCameraStream: SetupCameraStream,
  MissionWpGetInfo: MissionWpGetInfo,
  MissionStatus: MissionStatus,
  MissionHpAction: MissionHpAction,
  MFIOSetValue: MFIOSetValue,
  SendMobileData: SendMobileData,
  MissionHpResetYaw: MissionHpResetYaw,
};
