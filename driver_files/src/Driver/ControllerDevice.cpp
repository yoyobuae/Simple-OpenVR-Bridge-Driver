#include <algorithm>

#include "ControllerDevice.hpp"
#include "Key.hpp"

#include "input.h"

extern double wantedHeightOffset;
const double pi = std::acos(-1);

inline vr::HmdQuaternion_t HmdQuaternion_Init( double w, double x, double y, double z )
{
    vr::HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

inline vr::HmdQuaternion_t HmdQuaternion_Init_Angle( double angle, double x, double y, double z )
{
	double rad = 2 * pi * angle / 360;
	double rad_2 = rad / 2;
	return HmdQuaternion_Init(std::cos(rad_2),
				  -std::sin(rad_2) * x,
				  -std::sin(rad_2) * y,
				  -std::sin(rad_2) * z);
}

inline vr::HmdQuaternion_t HmdQuaternion_Product(vr::HmdQuaternion_t quat_a, vr::HmdQuaternion_t quat_b)
{
    vr::HmdQuaternion_t quat_res;
	quat_res.w = quat_a.w*quat_b.w - quat_a.x*quat_b.x - quat_a.y*quat_b.y - quat_a.z*quat_b.z;
	quat_res.x = quat_a.w*quat_b.x + quat_a.x*quat_b.w + quat_a.y*quat_b.z - quat_a.z*quat_b.y;
	quat_res.y = quat_a.w*quat_b.y - quat_a.x*quat_b.z + quat_a.y*quat_b.w + quat_a.z*quat_b.x;
	quat_res.z = quat_a.w*quat_b.z + quat_a.x*quat_b.y - quat_a.y*quat_b.x + quat_a.z*quat_b.w;
	return quat_res;
}

ExampleDriver::ControllerDevice::ControllerDevice(std::string serial, ControllerDevice::Handedness handedness):
    serial_(serial),
    handedness_(handedness)
{
}

std::string ExampleDriver::ControllerDevice::GetSerial()
{
    return this->serial_;
}

long long counter = 0;

void ExampleDriver::ControllerDevice::SetDirection(float x, float y, float rx, float ry, float a, float b)
{
    if(x == 0.0f && y == 0.0f)
        GetDriver()->GetInput()->UpdateBooleanComponent(this->joystick_touch_component_, false, 0);
    else
        GetDriver()->GetInput()->UpdateBooleanComponent(this->joystick_touch_component_, true, 0);

    GetDriver()->GetInput()->UpdateScalarComponent(this->joystick_x_component_, x, 0);
    GetDriver()->GetInput()->UpdateScalarComponent(this->joystick_y_component_, y, 0);

    if (rx == 0.0f && ry == 0.0f)
        GetDriver()->GetInput()->UpdateBooleanComponent(this->trackpad_touch_component_, false, 0);
    else
        GetDriver()->GetInput()->UpdateBooleanComponent(this->trackpad_touch_component_, true, 0);

    GetDriver()->GetInput()->UpdateScalarComponent(this->trackpad_x_component_, rx, 0);
    GetDriver()->GetInput()->UpdateScalarComponent(this->trackpad_y_component_, ry, 0);

    if (a > 0.5) {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, true, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, true, 0);
    }
    else {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, false, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, false, 0);
    }

    if (b > 0.5) {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_click_component_, true, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_touch_component_, true, 0);
    }
    else {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_click_component_, false, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_touch_component_, false, 0);
    }
}

void ExampleDriver::ControllerDevice::Update()
{
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;

    // Check if we need to keep vibrating
    if (this->did_vibrate_) {
        this->vibrate_anim_state_ += (GetDriver()->GetLastFrameTime().count()/1000.f);
        if (this->vibrate_anim_state_ > 1.0f) {
            this->did_vibrate_ = false;
            this->vibrate_anim_state_ = 0.0f;
        }
    }

    // Setup pose for this frame
    auto pose = this->last_pose_;

    // Update time delta (for working out velocity)
    std::chrono::milliseconds time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    double time_since_epoch_seconds = time_since_epoch.count() / 1000.0;
    double pose_time_delta_seconds = (time_since_epoch - _pose_timestamp).count() / 1000.0;

    // Update pose timestamp

    _pose_timestamp = time_since_epoch;

    // Copy the previous position data
    double previous_position[3] = { 0 };
    std::copy(std::begin(pose.vecPosition), std::end(pose.vecPosition), std::begin(previous_position));

    //send the new position and rotation from the pipe to the tracker object
    pose.vecPosition[0] = wantedPose[0];
    pose.vecPosition[1] = wantedPose[1] + wantedHeightOffset;
    pose.vecPosition[2] = wantedPose[2];

    pose.qRotation.w = wantedPose[3];
    pose.qRotation.x = wantedPose[4];
    pose.qRotation.y = wantedPose[5];
    pose.qRotation.z = wantedPose[6];


    if (pose_time_delta_seconds > 0)            //unless we get two pose updates at the same time, update velocity so steamvr can do some interpolation
    {
        pose.vecVelocity[0] = 0.8 * pose.vecVelocity[0] + 0.2 * (pose.vecPosition[0] - previous_position[0]) / pose_time_delta_seconds;
        pose.vecVelocity[1] = 0.8 * pose.vecVelocity[1] + 0.2 * (pose.vecPosition[1] - previous_position[1]) / pose_time_delta_seconds;
        pose.vecVelocity[2] = 0.8 * pose.vecVelocity[2] + 0.2 * (pose.vecPosition[2] - previous_position[2]) / pose_time_delta_seconds;
    }
    pose.poseTimeOffset = this->wantedTimeOffset;


    //pose.vecVelocity[0] = (pose.vecPosition[0] - previous_position[0]) / pose_time_delta_seconds;
    //pose.vecVelocity[1] = (pose.vecPosition[1] - previous_position[1]) / pose_time_delta_seconds;
    //pose.vecVelocity[2] = (pose.vecPosition[2] - previous_position[2]) / pose_time_delta_seconds;

    // Recalibrate controller orientation on button press
    if (this->handedness_ == Handedness::LEFT) {
        if (getJoyButton(BTN_TRIGGER_HAPPY2)) {
            pose.qDriverFromHeadRotation = HmdQuaternion_Init(-pose.qRotation.w,
                                                              pose.qRotation.x,
                                                              pose.qRotation.y,
                                                              pose.qRotation.z);
        }
    }
    else if (this->handedness_ == Handedness::RIGHT) {
        if (getJoyButton(BTN_TRIGGER_HAPPY3)) {
            pose.qDriverFromHeadRotation = HmdQuaternion_Init(-pose.qRotation.w,
                                                              pose.qRotation.x,
                                                              pose.qRotation.y,
                                                              pose.qRotation.z);
        }
    }

    // Post pose
    GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
    this->last_pose_ = pose;
}

void ExampleDriver::ControllerDevice::UpdatePos(double a, double b, double c, double time, double smoothing)
{
    this->wantedPose[0] = (1 - smoothing) * this->wantedPose[0] + smoothing * a;
    this->wantedPose[1] = (1 - smoothing) * this->wantedPose[1] + smoothing * b;
    this->wantedPose[2] = (1 - smoothing) * this->wantedPose[2] + smoothing * c;

    this->wantedTimeOffset = time;

}

void ExampleDriver::ControllerDevice::UpdateRot(double qw, double qx, double qy, double qz, double time, double smoothing)
{
    //lerp
    double dot = qx * this->wantedPose[4] + qy * this->wantedPose[5] + qz * this->wantedPose[6] + qw * this->wantedPose[3];

    if (dot < 0)
    {
        this->wantedPose[3] = smoothing * qw - (1 - smoothing) * this->wantedPose[3];
        this->wantedPose[4] = smoothing * qx - (1 - smoothing) * this->wantedPose[4];
        this->wantedPose[5] = smoothing * qy - (1 - smoothing) * this->wantedPose[5];
        this->wantedPose[6] = smoothing * qz - (1 - smoothing) * this->wantedPose[6];
    }
    else
    {
        this->wantedPose[3] = smoothing * qw + (1 - smoothing) * this->wantedPose[3];
        this->wantedPose[4] = smoothing * qx + (1 - smoothing) * this->wantedPose[4];
        this->wantedPose[5] = smoothing * qy + (1 - smoothing) * this->wantedPose[5];
        this->wantedPose[6] = smoothing * qz + (1 - smoothing) * this->wantedPose[6];
    }
    //normalize
    double mag = sqrt(this->wantedPose[3] * this->wantedPose[3] +
        this->wantedPose[4] * this->wantedPose[4] +
        this->wantedPose[5] * this->wantedPose[5] +
        this->wantedPose[6] * this->wantedPose[6]);

    this->wantedPose[3] /= mag;
    this->wantedPose[4] /= mag;
    this->wantedPose[5] /= mag;
    this->wantedPose[6] /= mag;

    this->wantedTimeOffset = time;

}

DeviceType ExampleDriver::ControllerDevice::GetDeviceType()
{
    return DeviceType::CONTROLLER;
}

ExampleDriver::ControllerDevice::Handedness ExampleDriver::ControllerDevice::GetHandedness()
{
    return this->handedness_;
}

vr::TrackedDeviceIndex_t ExampleDriver::ControllerDevice::GetDeviceIndex()
{
    return this->device_index_;
}

void ExampleDriver::ControllerDevice::RunFrame()
{
    if (this->handedness_ == Handedness::LEFT) {
        GetDriver()->GetInput()->UpdateBooleanComponent(application_button_click_component_, getJoyButton(BTN_Z), 0); //Application Menu
        GetDriver()->GetInput()->UpdateBooleanComponent(grip_button_click_component_, getJoyButton(BTN_TL2), 0); //Grip
        GetDriver()->GetInput()->UpdateBooleanComponent(system_button_click_component_, getJoyButton(BTN_SELECT), 0); //System
        GetDriver()->GetInput()->UpdateBooleanComponent(trackpad_button_click_component_, getJoyButton(BTN_THUMBL), 0); //Trackpad

        float x = static_cast<float>(getJoyAxis(ABS_X))/32768.0f;
        float y = static_cast<float>(getJoyAxis(ABS_Y))/-32768.0f;
        float norm = hypot(x, y);
        float arct = atan2(y, x);

        // Deadzone and touch calcs
        bool trackpad_touch = norm > 0.125f;
        float trackpad_norm = norm > 0.25f ? (norm - 0.25f) / 0.75f : 0.0f;
        float trackpad_x = trackpad_norm * cos(arct);
        float trackpad_y = trackpad_norm * sin(arct);

        GetDriver()->GetInput()->UpdateBooleanComponent(trackpad_touch_component_, trackpad_touch, 0); //Trackpad
        GetDriver()->GetInput()->UpdateScalarComponent(trackpad_x_component_, trackpad_x, 0); //Trackpad x
        GetDriver()->GetInput()->UpdateScalarComponent(trackpad_y_component_, trackpad_y, 0); //Trackpad y


        if (getJoyButton(BTN_TL)) { //Trigger
            GetDriver()->GetInput()->UpdateScalarComponent(trigger_value_component_, 1.0, 0);
        } else {
            GetDriver()->GetInput()->UpdateScalarComponent(trigger_value_component_, 0.0, 0);
        }

        if (getJoyButton(BTN_DPAD_UP)) { // Height up
            wantedHeightOffset += 0.02;
        }
        if (getJoyButton(BTN_DPAD_DOWN)) { // Height down
            wantedHeightOffset -= 0.02;
        }
        if (getJoyButton(BTN_DPAD_RIGHT)) { // Height reset
            wantedHeightOffset = 0.0;
        }
    }
    else if (this->handedness_ == Handedness::RIGHT) {
        GetDriver()->GetInput()->UpdateBooleanComponent(application_button_click_component_, getJoyButton(BTN_MODE), 0); //Application Menu
        GetDriver()->GetInput()->UpdateBooleanComponent(grip_button_click_component_, getJoyButton(BTN_TR2), 0); //Grip
        GetDriver()->GetInput()->UpdateBooleanComponent(system_button_click_component_, getJoyButton(BTN_START), 0); //System
        GetDriver()->GetInput()->UpdateBooleanComponent(trackpad_button_click_component_, getJoyButton(BTN_THUMBR), 0); //Trackpad

        float x = static_cast<float>(getJoyAxis(ABS_RX))/32768.0f;
        float y = static_cast<float>(getJoyAxis(ABS_RY))/-32768.0f;
        float norm = hypot(x, y);
        float arct = atan2(y, x);

        // Deadzone and touch calcs
        bool trackpad_touch = norm > 0.125f;
        float trackpad_norm = norm > 0.25f ? (norm - 0.25f) / 0.75f : 0.0f;
        float trackpad_x = trackpad_norm * cos(arct);
        float trackpad_y = trackpad_norm * sin(arct);

        GetDriver()->GetInput()->UpdateBooleanComponent(trackpad_touch_component_, trackpad_touch, 0); //Trackpad
        GetDriver()->GetInput()->UpdateScalarComponent(trackpad_x_component_, trackpad_x, 0); //Trackpad x
        GetDriver()->GetInput()->UpdateScalarComponent(trackpad_y_component_, trackpad_y, 0); //Trackpad y


        if (getJoyButton(BTN_TR)) { //Trigger
            GetDriver()->GetInput()->UpdateScalarComponent(trigger_value_component_, 1.0, 0);
        } else {
            GetDriver()->GetInput()->UpdateScalarComponent(trigger_value_component_, 0.0, 0);
        }

    }
}

vr::EVRInitError ExampleDriver::ControllerDevice::Activate(uint32_t unObjectId)
{
    this->device_index_ = unObjectId;

    GetDriver()->Log("Activating controller " + this->serial_);

    // Get the properties handle
    auto props = GetDriver()->GetProperties()->TrackedDeviceToPropertyContainer(this->device_index_);

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, "vive_controller");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{htc}/input/vive_controller_profile.json");

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "ViveMV");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ManufacturerName_String, "HTC");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_TrackingSystemName_String, "VR Controller");
    GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_SerialNumber_String, this->serial_.c_str());

    uint64_t supportedButtons = 0xFFFFFFFFFFFFFFFFULL;
    GetDriver()->GetProperties()->SetUint64Property(props, vr::Prop_SupportedButtons_Uint64, supportedButtons);

    // Give SteamVR a hint at what hand this controller is for

    GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_Treadmill);

    // this file tells the UI what to show the user for binding this controller as well as what default bindings should
    // be for legacy or other apps
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{htc}/input/vive_controller_profile.json");

    //  Buttons handles
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/application_menu/click", &application_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/grip/click", &grip_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/click", &system_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trackpad/click", &trackpad_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trackpad/touch", &trackpad_touch_component_);

    // Analog handles
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trackpad/x", &trackpad_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trackpad/y", &trackpad_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trigger/value", &trigger_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

    GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_TrackPad);

    // create our haptic component
    GetDriver()->GetInput()->CreateHapticComponent(props, "/output/haptic", &haptic_component_);


    return vr::EVRInitError::VRInitError_None;
}

void ExampleDriver::ControllerDevice::Deactivate()
{
    this->device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void ExampleDriver::ControllerDevice::EnterStandby()
{
}

void* ExampleDriver::ControllerDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    return nullptr;
}

void ExampleDriver::ControllerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t ExampleDriver::ControllerDevice::GetPose()
{
    return last_pose_;
}
