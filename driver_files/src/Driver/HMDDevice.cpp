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

static void normalizeQuat(double pose[])
{
    //normalize
    double mag = sqrt(pose[3] * pose[3] +
        pose[4] * pose[4] +
        pose[5] * pose[5] +
        pose[6] * pose[6]);

    pose[3] /= mag;
    pose[4] /= mag;
    pose[5] /= mag;
    pose[6] /= mag;
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

    double next_pose[7];
    if (get_next_pose(0, next_pose) != 0)
        return;

    normalizeQuat(next_pose);

    double wantedPose[7];
    wantedPose[0] = next_pose[0] * (1 - smoothing) + pose.vecPosition[0] * smoothing;
    wantedPose[1] = next_pose[1] * (1 - smoothing) + pose.vecPosition[1] * smoothing;
    wantedPose[2] = next_pose[2] * (1 - smoothing) + pose.vecPosition[2] * smoothing;

    wantedPose[3] = next_pose[3] * (1 - smoothing) + pose.qRotation.w * smoothing;
    wantedPose[4] = next_pose[4] * (1 - smoothing) + pose.qRotation.x * smoothing;
    wantedPose[5] = next_pose[5] * (1 - smoothing) + pose.qRotation.y * smoothing;
    wantedPose[6] = next_pose[6] * (1 - smoothing) + pose.qRotation.z * smoothing;

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

    //normalize
    double mag = sqrt(pose.qRotation.w * pose.qRotation.w +
        pose.qRotation.x * pose.qRotation.x +
        pose.qRotation.y * pose.qRotation.y +
        pose.qRotation.z * pose.qRotation.z);

    pose.qRotation.w /= mag;
    pose.qRotation.x /= mag;
    pose.qRotation.y /= mag;
    pose.qRotation.z /= mag;

    pose.poseTimeOffset = 0;

    // Compensate for difference in orientation between tracker and forward HMD direction
    // Should probably be made adjustable
    pose.qDriverFromHeadRotation =
        HmdQuaternion_Product(HmdQuaternion_Init_Angle( -42, 0, 1, 0 ),
                              HmdQuaternion_Init_Angle( 18, 1, 0, 0 ));
    pose.vecDriverFromHeadTranslation[0] = 0.007382;
    pose.vecDriverFromHeadTranslation[1] = -0.065965;
    pose.vecDriverFromHeadTranslation[2] = 0.057777;

    // Post pose
    GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
    this->last_pose_ = pose;
}

DeviceType ExampleDriver::HMDDevice::GetDeviceType()
{
    return DeviceType::HMD;
}

vr::TrackedDeviceIndex_t ExampleDriver::HMDDevice::GetDeviceIndex()
{
    return this->device_index_;
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

void ExampleDriver::HMDDevice::Log(std::string message)
{
    std::string message_endl = message + "\n";
    vr::VRDriverLog()->Log(message_endl.c_str());
}

void ExampleDriver::HMDDevice::save_current_pose(double a, double b, double c, double qw, double qx, double qy, double qz, double time_offset)
{
    double next_pose[7];
    int pose_valid = get_next_pose(time_offset, next_pose);

    double dot = qx * next_pose[4] + qy * next_pose[5] + qz * next_pose[6] + qw * next_pose[3];

    if (dot < 0)
    {
        qx = -qx;
        qy = -qy;
        qz = -qz;
        qw = -qw;
    } 

    //update times
    std::chrono::milliseconds time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    double time_since_epoch_seconds = time_since_epoch.count() / 1000.0;

    double curr_time = time_since_epoch_seconds;
    double time_since_update = curr_time - this->last_update;
    this->last_update = curr_time;

    for (int i = 0; i < max_saved; i++)
    {
        if (prev_positions[i][0] >= 0)
            prev_positions[i][0] += time_since_update;
        if (prev_positions[i][0] > max_time)
            prev_positions[i][0] = -1;
    }

    double time = time_offset;

    double dist = sqrt(pow(next_pose[0] - a, 2) + pow(next_pose[1] - b, 2) + pow(next_pose[2] - c, 2));
    if (pose_valid == 0 && dist > 0.5)
    {
        Log("Dropped a pose! its error was " + std::to_string(dist));
        Log("Height vs predicted height:" + std::to_string(b) + " " + std::to_string(next_pose[1]));
        return;
    }

    dist = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
    if (dist > 10)
    {
        Log("Dropped a pose! Was outside of playspace: " + std::to_string(dist));
        return;
    }

    if (time > max_time)
        return;

    if (prev_positions[max_saved - 1][0] < time && prev_positions[max_saved - 1][0] >= 0)
        return;

    int i = 0;
    while (prev_positions[i][0] < time&& prev_positions[i][0] >= 0)
        i++;

    for (int j = max_saved - 1; j > i; j--)
    {
        if (prev_positions[j - 1][0] >= 0)
        {
            for (int k = 0; k < 8; k++)
            {
                prev_positions[j][k] = prev_positions[j - 1][k];
            }
        }
        else
        {
            prev_positions[j][0] = -1;
        }
    }
    prev_positions[i][0] = time;
    prev_positions[i][1] = a;
    prev_positions[i][2] = b;
    prev_positions[i][3] = c;
    prev_positions[i][4] = qw;
    prev_positions[i][5] = qx;
    prev_positions[i][6] = qy;
    prev_positions[i][7] = qz;
    return;
}

int ExampleDriver::HMDDevice::get_next_pose(double time_offset, double pred[])
{
    int statuscode = 0;

    std::chrono::milliseconds time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    double time_since_epoch_seconds = time_since_epoch.count() / 1000.0;

    double req_time = time_since_epoch_seconds - time_offset;

    double new_time = last_update - req_time;

    if (new_time < -0.2)      //limit prediction to max 0.2 second into the future to prevent your feet from being yeeted into oblivion
    {
        new_time = -0.2;
        statuscode = 1;
    }

    int curr_saved = 0;

    double avg_time = 0;
    double avg_time2 = 0;
    for (int i = 0; i < max_saved; i++)
    {
        if (prev_positions[i][0] < 0)
            break;
        curr_saved++;
        avg_time += prev_positions[i][0];
        avg_time2 += (prev_positions[i][0] * prev_positions[i][0]);
    }

    if (curr_saved < 4)
    {
        statuscode = -1;
        return statuscode;
    }
    avg_time /= curr_saved;
    avg_time2 /= curr_saved;

    double st = 0;
    for (int j = 0; j < curr_saved; j++)
    {
        st += ((prev_positions[j][0] - avg_time) * (prev_positions[j][0] - avg_time));
    }
    st = sqrt(st * (1.0 / curr_saved));


    for (int i = 1; i < 8; i++)
    {
        double avg_val = 0;
        double avg_val2 = 0;
        double avg_tval = 0;
        for (int ii = 0; ii < curr_saved; ii++)
        {
            avg_val += prev_positions[ii][i];
            avg_tval += (prev_positions[ii][0] * prev_positions[ii][i]);
            avg_val2 += (prev_positions[ii][i] * prev_positions[ii][i]);
        }
        avg_val /= curr_saved;
        avg_tval /= curr_saved;
        avg_val2 /= curr_saved;

        double sv = 0;
        for (int j = 0; j < curr_saved; j++)
        {
            sv += ((prev_positions[j][i] - avg_val) * (prev_positions[j][i] - avg_val));
        }
        sv = sqrt(sv * (1.0 / curr_saved));

        double rxy = (avg_tval - (avg_val * avg_time)) / sqrt((avg_time2 - (avg_time * avg_time)) * (avg_val2 - (avg_val * avg_val)));
        double b = rxy * (sv / st);
        double a = avg_val - (b * avg_time);

        double y = a + b * new_time;
        if (abs(avg_val2 - (avg_val * avg_val)) < 0.00000001)               //bloody floating point rounding errors
            y = avg_val;

        pred[i - 1] = y;
    }
    return statuscode;
}

void ExampleDriver::HMDDevice::reinit(int msaved, double mtime, double msmooth)
{
    if (msaved < 5)     //prevent having too few values to calculate linear interpolation, and prevent crash on 0
        msaved = 5;

    if (msmooth < 0)
        msmooth = 0;
    else if (msmooth > 0.99)
        msmooth = 0.99;

    max_saved = msaved;
    std::vector<std::vector<double>> temp(msaved, std::vector<double>(8,-1));
    prev_positions = temp;
    max_time = mtime;
    smoothing = msmooth;
}
