#include "HMDDevice.hpp"
#include "Key.hpp"

#if !defined(_WIN32) && !defined(__WIN32__) && !defined(WIN32)
#include "string.h"
#define _stricmp strcasecmp
#endif

#include <openvr_driver.h>

double wantedHeightOffset = 0;
bool viewLockRequested = false;
bool viewLocked = false;

using namespace vr;

const double pi = std::acos(-1);

inline HmdQuaternion_t HmdQuaternion_Init( double w, double x, double y, double z )
{
	HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

inline HmdQuaternion_t HmdQuaternion_Init_Angle( double angle, double x, double y, double z )
{
	double rad = 2 * pi * angle / 360;
	double rad_2 = rad / 2;
	return HmdQuaternion_Init(std::cos(rad_2),
				  -std::sin(rad_2) * x,
				  -std::sin(rad_2) * y,
				  -std::sin(rad_2) * z);
}

inline HmdQuaternion_t HmdQuaternion_Product(HmdQuaternion_t quat_a, HmdQuaternion_t quat_b)
{
	HmdQuaternion_t quat_res;
	quat_res.w = quat_a.w*quat_b.w - quat_a.x*quat_b.x - quat_a.y*quat_b.y - quat_a.z*quat_b.z;
	quat_res.x = quat_a.w*quat_b.x + quat_a.x*quat_b.w + quat_a.y*quat_b.z - quat_a.z*quat_b.y;
	quat_res.y = quat_a.w*quat_b.y - quat_a.x*quat_b.z + quat_a.y*quat_b.w + quat_a.z*quat_b.x;
	quat_res.z = quat_a.w*quat_b.z + quat_a.x*quat_b.y - quat_a.y*quat_b.x + quat_a.z*quat_b.w;
	return quat_res;
}

ExampleDriver::HMDDevice::HMDDevice(std::string serial):serial_(serial)
{
}

std::string ExampleDriver::HMDDevice::GetSerial()
{
    return this->serial_;
}

void ExampleDriver::HMDDevice::Update()
{
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;

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

    if (viewLockRequested && !viewLocked) {
        viewLockPose[0] = wantedPose[0];
        viewLockPose[1] = wantedPose[1] + wantedHeightOffset;
        viewLockPose[2] = wantedPose[2];

        viewLockPose[3] = wantedPose[3];
        viewLockPose[4] = wantedPose[4];
        viewLockPose[5] = wantedPose[5];
        viewLockPose[6] = wantedPose[6];
        viewLocked = true;
    }
    if (viewLockRequested && viewLocked) {
        //send the new position and rotation from the pipe to the tracker object
        pose.vecPosition[0] = wantedPose[0];
        pose.vecPosition[1] = wantedPose[1] + wantedHeightOffset;
        pose.vecPosition[2] = wantedPose[2];

        pose.qRotation.w = viewLockPose[3];
        pose.qRotation.x = viewLockPose[4];
        pose.qRotation.y = viewLockPose[5];
        pose.qRotation.z = viewLockPose[6];
    } else {
        //send the new position and rotation from the pipe to the tracker object
        pose.vecPosition[0] = wantedPose[0];
        pose.vecPosition[1] = wantedPose[1] + wantedHeightOffset;
        pose.vecPosition[2] = wantedPose[2];

        pose.qRotation.w = wantedPose[3];
        pose.qRotation.x = wantedPose[4];
        pose.qRotation.y = wantedPose[5];
        pose.qRotation.z = wantedPose[6];
        viewLocked = false;
    }


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

    // Compensate for difference in orientation between tracker and forward HMD direction
    // Should probably be made adjustable
    pose.qDriverFromHeadRotation =
        HmdQuaternion_Product(HmdQuaternion_Init_Angle( -45, 0, 1, 0 ),
                              HmdQuaternion_Init_Angle( 30, 1, 0, 0 ));

    // Post pose
    GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
    this->last_pose_ = pose;
}

void ExampleDriver::HMDDevice::UpdatePos(double a, double b, double c, double time, double smoothing)
{
    this->wantedPose[0] = (1 - smoothing) * this->wantedPose[0] + smoothing * a;
    this->wantedPose[1] = (1 - smoothing) * this->wantedPose[1] + smoothing * b;
    this->wantedPose[2] = (1 - smoothing) * this->wantedPose[2] + smoothing * c;

    this->wantedTimeOffset = time;

}

void ExampleDriver::HMDDevice::UpdateRot(double qw, double qx, double qy, double qz, double time, double smoothing)
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

DeviceType ExampleDriver::HMDDevice::GetDeviceType()
{
    return DeviceType::HMD;
}

vr::TrackedDeviceIndex_t ExampleDriver::HMDDevice::GetDeviceIndex()
{
    return this->device_index_;
}

void ExampleDriver::HMDDevice::RunFrame()
{
}

vr::EVRInitError ExampleDriver::HMDDevice::Activate(uint32_t unObjectId)
{
    this->device_index_ = unObjectId;

    GetDriver()->Log("Activating HMD " + this->serial_);

    // Load settings values
    // Could probably make this cleaner with making a wrapper class
    try {
        int window_x = std::get<int>(GetDriver()->GetSettingsValue("window_x"));
        if (window_x > 0)
            this->window_x_ = window_x;
    }
    catch (const std::bad_variant_access&) {}; // Wrong type or doesnt exist

    try {
        int window_y = std::get<int>(GetDriver()->GetSettingsValue("window_y"));
        if (window_y > 0)
            this->window_x_ = window_y;
    }
    catch (const std::bad_variant_access&) {}; // Wrong type or doesnt exist

    try {
        int window_width = std::get<int>(GetDriver()->GetSettingsValue("window_width"));
        if (window_width > 0)
            this->window_width_ = window_width;
    }
    catch (const std::bad_variant_access&) {}; // Wrong type or doesnt exist

    try {
        int window_height = std::get<int>(GetDriver()->GetSettingsValue("window_height"));
        if (window_height > 0)
            this->window_height_ = window_height;
    }
    catch (const std::bad_variant_access&) {}; // Wrong type or doesnt exist

    // Get the properties handle
    auto props = GetDriver()->GetProperties()->TrackedDeviceToPropertyContainer(this->device_index_);

    // Set some universe ID (Must be 2 or higher)
    GetDriver()->GetProperties()->SetUint64Property(props, vr::Prop_CurrentUniverseId_Uint64, 2);

    // Set the IPD to zero since we'll be displaying on desktop
    GetDriver()->GetProperties()->SetFloatProperty(props, vr::Prop_UserIpdMeters_Float, 0.f);

    // Set the display FPS
    GetDriver()->GetProperties()->SetFloatProperty(props, vr::Prop_DisplayFrequency_Float, 60.f);

    // Set up a model "number" (not needed but good to have)
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "EXAMPLE_HMD_DEVICE");

    // Set up icon paths
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReady_String, "{example}/icons/hmd_ready.png");

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceOff_String, "{example}/icons/hmd_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearching_String, "{example}/icons/hmd_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{example}/icons/hmd_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{example}/icons/hmd_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceNotReady_String, "{example}/icons/hmd_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceStandby_String, "{example}/icons/hmd_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceAlertLow_String, "{example}/icons/hmd_not_ready.png");




    return vr::EVRInitError::VRInitError_None;
}

void ExampleDriver::HMDDevice::Deactivate()
{
    this->device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void ExampleDriver::HMDDevice::EnterStandby()
{
}

void* ExampleDriver::HMDDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    if (!_stricmp(pchComponentNameAndVersion, vr::IVRDisplayComponent_Version)) {
        return static_cast<vr::IVRDisplayComponent*>(this);
    }
    return nullptr;
}

void ExampleDriver::HMDDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t ExampleDriver::HMDDevice::GetPose()
{
    return this->last_pose_;
}

void ExampleDriver::HMDDevice::GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
{
    *pnX = this->window_x_;
    *pnY = this->window_y_;
    *pnWidth = this->window_width_;
    *pnHeight = this->window_height_;
}

bool ExampleDriver::HMDDevice::IsDisplayOnDesktop()
{
    return true;
}

bool ExampleDriver::HMDDevice::IsDisplayRealDisplay()
{
    return false;
}

void ExampleDriver::HMDDevice::GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight)
{
    *pnWidth = this->window_width_;
    *pnHeight = this->window_height_;
}

void ExampleDriver::HMDDevice::GetEyeOutputViewport(vr::EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
{
    *pnY = 0;
    *pnWidth = this->window_width_ / 2;
    *pnHeight = this->window_height_;

    if (eEye == vr::EVREye::Eye_Left) {
        *pnX = 0;
    }
    else {
        *pnX = this->window_width_ / 2;
    }
}

void ExampleDriver::HMDDevice::GetProjectionRaw(vr::EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom)
{
    if (eEye == vr::EVREye::Eye_Left) {
        *pfLeft = -1.706161137;
        *pfRight = 0.0;
    }
    else {
        *pfLeft = 0.0;
        *pfRight = 1.706161137;
    }
    *pfTop = -1;
    *pfBottom = 1;
}

vr::DistortionCoordinates_t ExampleDriver::HMDDevice::ComputeDistortion(vr::EVREye eEye, float fU, float fV)
{
    vr::DistortionCoordinates_t coordinates;
    coordinates.rfBlue[0] = fU;
    coordinates.rfBlue[1] = fV;
    coordinates.rfGreen[0] = fU;
    coordinates.rfGreen[1] = fV;
    coordinates.rfRed[0] = fU;
    coordinates.rfRed[1] = fV;
    return coordinates;
}
void ExampleDriver::HMDDevice::save_current_pose(double a, double b, double c, double qw, double qx, double qy, double qz, double time)
{
}

int ExampleDriver::HMDDevice::get_next_pose(double req_time, double pred[])
{
    return -1;
}

void ExampleDriver::HMDDevice::reinit(int msaved, double mtime, double msmooth)
{
}
