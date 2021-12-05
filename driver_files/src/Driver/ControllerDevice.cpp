#include <algorithm>

#include "ControllerDevice.hpp"
#include "Key.hpp"

#include "input.h"

extern double wantedHeightOffset;
extern bool viewLockRequested;
extern bool viewLocked;

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

ExampleDriver::ControllerDevice::ControllerDevice(std::string serial, ControllerDevice::Handedness handedness):
    serial_(serial),
    handedness_(handedness)
{
}

std::string ExampleDriver::ControllerDevice::GetSerial()
{
    return this->serial_;
}

static const int NUM_BONES = 31;

static vr::VRBoneTransform_t left_open_hand_pose[NUM_BONES] = {
{ { 0.000000f,  0.000000f,  0.000000f,  1.000000f }, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
{ {-0.034038f,  0.006503f,  0.194722f,  1.000000f }, {-0.020867f, -0.093728f, -0.995377f, -0.001752f} },
{ {-0.012083f,  0.028070f,  0.025050f,  1.000000f }, { 0.464112f,  0.567418f,  0.272106f,  0.623374f} },
{ { 0.040406f,  0.000000f, -0.000000f,  1.000000f }, { 0.994838f,  0.082939f,  0.019454f,  0.055130f} },
{ { 0.032517f,  0.000000f,  0.000000f,  1.000000f }, { 0.974793f, -0.003213f,  0.021867f, -0.222015f} },
{ { 0.030464f, -0.000000f, -0.000000f,  1.000000f }, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
{ { 0.000632f,  0.026866f,  0.015002f,  1.000000f }, { 0.644251f,  0.421979f, -0.478202f,  0.422133f} },
{ { 0.074204f, -0.005002f,  0.000234f,  1.000000f }, { 0.995332f,  0.007007f, -0.039124f,  0.087949f} },
{ { 0.043930f, -0.000000f, -0.000000f,  1.000000f }, { 0.997891f,  0.045808f,  0.002142f, -0.045943f} },
{ { 0.028695f,  0.000000f,  0.000000f,  1.000000f }, { 0.999649f,  0.001850f, -0.022782f, -0.013409f} },
{ { 0.022821f,  0.000000f, -0.000000f,  1.000000f }, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
{ { 0.002177f,  0.007120f,  0.016319f,  1.000000f }, { 0.546723f,  0.541276f, -0.442520f,  0.460749f} },
{ { 0.070953f,  0.000779f,  0.000997f,  1.000000f }, { 0.980294f, -0.167261f, -0.078959f,  0.069368f} },
{ { 0.043108f,  0.000000f,  0.000000f,  1.000000f }, { 0.997947f,  0.018493f,  0.013192f,  0.059886f} },
{ { 0.033266f,  0.000000f,  0.000000f,  1.000000f }, { 0.997394f, -0.003328f, -0.028225f, -0.066315f} },
{ { 0.025892f, -0.000000f,  0.000000f,  1.000000f }, { 0.999195f, -0.000000f,  0.000000f,  0.040126f} },
{ { 0.000513f, -0.006545f,  0.016348f,  1.000000f }, { 0.516692f,  0.550143f, -0.495548f,  0.429888f} },
{ { 0.065876f,  0.001786f,  0.000693f,  1.000000f }, { 0.990420f, -0.058696f, -0.101820f,  0.072495f} },
{ { 0.040697f,  0.000000f,  0.000000f,  1.000000f }, { 0.999545f, -0.002240f,  0.000004f,  0.030081f} },
{ { 0.028747f, -0.000000f, -0.000000f,  1.000000f }, { 0.999102f, -0.000721f, -0.012693f,  0.040420f} },
{ { 0.022430f, -0.000000f,  0.000000f,  1.000000f }, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
{ {-0.002478f, -0.018981f,  0.015214f,  1.000000f }, { 0.526918f,  0.523940f, -0.584025f,  0.326740f} },
{ { 0.062878f,  0.002844f,  0.000332f,  1.000000f }, { 0.986609f, -0.059615f, -0.135163f,  0.069132f} },
{ { 0.030220f,  0.000000f,  0.000000f,  1.000000f }, { 0.994317f,  0.001896f, -0.000132f,  0.106446f} },
{ { 0.018187f,  0.000000f,  0.000000f,  1.000000f }, { 0.995931f, -0.002010f, -0.052079f, -0.073526f} },
{ { 0.018018f,  0.000000f, -0.000000f,  1.000000f }, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
{ {-0.006059f,  0.094495f,  0.134706f,  1.000000f }, { 0.603532f,  0.469441f,  0.453574f,  0.457870f} },
{ {-0.040416f,  0.053070f,  0.035695f,  1.000000f }, {-0.506844f,  0.464959f, -0.500872f, -0.525404f} },
{ {-0.039354f,  0.010390f,  0.032193f,  1.000000f }, {-0.432292f,  0.554868f, -0.507427f, -0.497757f} },
{ {-0.038340f, -0.025562f,  0.046489f,  1.000000f }, {-0.451063f,  0.610663f, -0.531357f, -0.375891f} },
{ {-0.031806f, -0.050073f,  0.076335f,  1.000000f }, {-0.293610f,  0.699277f, -0.542197f, -0.361701f} },
};

static vr::VRBoneTransform_t left_closed_hand_pose[NUM_BONES] = {
{ { 0.000000f,  0.000000f,  0.000000f,  1.000000f }, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
{ {-0.034038f,  0.006503f,  0.194722f,  1.000000f }, {-0.020867f, -0.093728f, -0.995377f, -0.001752f} },
{ {-0.016305f,  0.027529f,  0.017800f,  1.000000f }, { 0.225703f,  0.483332f,  0.126413f,  0.836342f} },
{ { 0.040406f,  0.000000f, -0.000000f,  1.000000f }, { 0.894335f, -0.013302f, -0.082902f,  0.439448f} },
{ { 0.032517f,  0.000000f,  0.000000f,  1.000000f }, { 0.842428f,  0.000655f,  0.001244f,  0.538807f} },
{ { 0.030464f, -0.000000f, -0.000000f,  1.000000f }, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
{ { 0.003802f,  0.021514f,  0.012803f,  1.000000f }, { 0.617314f,  0.395175f, -0.510874f,  0.449185f} },
{ { 0.074204f, -0.005002f,  0.000234f,  1.000000f }, { 0.737291f, -0.032006f, -0.115013f,  0.664944f} },
{ { 0.043287f, -0.000000f, -0.000000f,  1.000000f }, { 0.611381f,  0.003287f,  0.003823f,  0.791321f} },
{ { 0.028275f,  0.000000f,  0.000000f,  1.000000f }, { 0.745388f, -0.000684f, -0.000945f,  0.666629f} },
{ { 0.022821f,  0.000000f, -0.000000f,  1.000000f }, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
{ { 0.005787f,  0.006806f,  0.016534f,  1.000000f }, { 0.514203f,  0.522315f, -0.478348f,  0.483700f} },
{ { 0.070953f,  0.000779f,  0.000997f,  1.000000f }, { 0.723653f, -0.097901f,  0.048546f,  0.681458f} },
{ { 0.043108f,  0.000000f,  0.000000f,  1.000000f }, { 0.637464f, -0.002366f, -0.002831f,  0.770472f} },
{ { 0.033266f,  0.000000f,  0.000000f,  1.000000f }, { 0.658008f,  0.002610f,  0.003196f,  0.753000f} },
{ { 0.025892f, -0.000000f,  0.000000f,  1.000000f }, { 0.999195f, -0.000000f,  0.000000f,  0.040126f} },
{ { 0.004123f, -0.006858f,  0.016563f,  1.000000f }, { 0.489609f,  0.523374f, -0.520644f,  0.463997f} },
{ { 0.065876f,  0.001786f,  0.000693f,  1.000000f }, { 0.759970f, -0.055609f,  0.011571f,  0.647471f} },
{ { 0.040331f,  0.000000f,  0.000000f,  1.000000f }, { 0.664315f,  0.001595f,  0.001967f,  0.747449f} },
{ { 0.028489f, -0.000000f, -0.000000f,  1.000000f }, { 0.626957f, -0.002784f, -0.003234f,  0.779042f} },
{ { 0.022430f, -0.000000f,  0.000000f,  1.000000f }, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
{ { 0.001131f, -0.019295f,  0.015429f,  1.000000f }, { 0.479766f,  0.477833f, -0.630198f,  0.379934f} },
{ { 0.062878f,  0.002844f,  0.000332f,  1.000000f }, { 0.827001f,  0.034282f,  0.003440f,  0.561144f} },
{ { 0.029874f,  0.000000f,  0.000000f,  1.000000f }, { 0.702185f, -0.006716f, -0.009289f,  0.711903f} },
{ { 0.017979f,  0.000000f,  0.000000f,  1.000000f }, { 0.676853f,  0.007956f,  0.009917f,  0.736009f} },
{ { 0.018018f,  0.000000f, -0.000000f,  1.000000f }, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
{ { 0.019716f,  0.032725f,  0.120839f,  1.000000f }, { 0.555534f, -0.355282f,  0.420864f, -0.622921f} },
{ { 0.000171f,  0.040569f,  0.132329f,  1.000000f }, {-0.014669f,  0.018545f, -0.724419f, -0.688954f} },
{ { 0.000448f,  0.015845f,  0.135929f,  1.000000f }, {-0.076598f,  0.082078f, -0.723012f, -0.681651f} },
{ { 0.003949f, -0.005700f,  0.134274f,  1.000000f }, {-0.077167f,  0.042391f, -0.750489f, -0.654991f} },
{ { 0.003263f, -0.026301f,  0.126851f,  1.000000f }, { 0.056743f, -0.085538f, -0.781077f, -0.615940f} },
};

static vr::VRBoneTransform_t right_open_hand_pose[NUM_BONES] = {
{ {-0.000000f,  0.000000f,  0.000000f,  1.000000f }, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
{ { 0.034038f,  0.006503f,  0.194722f,  1.000000f }, {-0.020867f, -0.093728f,  0.995377f,  0.001752f} },
{ { 0.012083f,  0.028070f,  0.025050f,  1.000000f }, { 0.464112f,  0.567418f, -0.272106f, -0.623374f} },
{ {-0.040406f,  0.000000f, -0.000000f,  1.000000f }, { 0.994838f,  0.082939f, -0.019454f, -0.055130f} },
{ {-0.032517f,  0.000000f,  0.000000f,  1.000000f }, { 0.974793f, -0.003213f, -0.021867f,  0.222015f} },
{ {-0.030464f, -0.000000f, -0.000000f,  1.000000f }, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
{ {-0.000632f,  0.026866f,  0.015002f,  1.000000f }, { 0.644251f,  0.421979f,  0.478202f, -0.422133f} },
{ {-0.074204f, -0.005002f,  0.000234f,  1.000000f }, { 0.995332f,  0.007007f,  0.039124f, -0.087949f} },
{ {-0.043930f, -0.000000f, -0.000000f,  1.000000f }, { 0.997891f,  0.045808f, -0.002142f,  0.045943f} },
{ {-0.028695f,  0.000000f,  0.000000f,  1.000000f }, { 0.999649f,  0.001850f,  0.022782f,  0.013409f} },
{ {-0.022821f,  0.000000f, -0.000000f,  1.000000f }, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
{ {-0.002177f,  0.007120f,  0.016319f,  1.000000f }, { 0.546723f,  0.541276f,  0.442520f, -0.460749f} },
{ {-0.070953f,  0.000779f,  0.000997f,  1.000000f }, { 0.980294f, -0.167261f,  0.078959f, -0.069368f} },
{ {-0.043108f,  0.000000f,  0.000000f,  1.000000f }, { 0.997947f,  0.018493f, -0.013192f, -0.059886f} },
{ {-0.033266f,  0.000000f,  0.000000f,  1.000000f }, { 0.997394f, -0.003328f,  0.028225f,  0.066315f} },
{ {-0.025892f, -0.000000f,  0.000000f,  1.000000f }, { 0.999195f, -0.000000f, -0.000000f, -0.040126f} },
{ {-0.000513f, -0.006545f,  0.016348f,  1.000000f }, { 0.516692f,  0.550143f,  0.495548f, -0.429888f} },
{ {-0.065876f,  0.001786f,  0.000693f,  1.000000f }, { 0.990420f, -0.058696f,  0.101820f, -0.072495f} },
{ {-0.040697f,  0.000000f,  0.000000f,  1.000000f }, { 0.999545f, -0.002240f, -0.000004f, -0.030081f} },
{ {-0.028747f, -0.000000f, -0.000000f,  1.000000f }, { 0.999102f, -0.000721f,  0.012693f, -0.040420f} },
{ {-0.022430f, -0.000000f,  0.000000f,  1.000000f }, { 1.000000f,  0.000000f, -0.000000f, -0.000000f} },
{ { 0.002478f, -0.018981f,  0.015214f,  1.000000f }, { 0.526918f,  0.523940f,  0.584025f, -0.326740f} },
{ {-0.062878f,  0.002844f,  0.000332f,  1.000000f }, { 0.986609f, -0.059615f,  0.135163f, -0.069132f} },
{ {-0.030220f,  0.000000f,  0.000000f,  1.000000f }, { 0.994317f,  0.001896f,  0.000132f, -0.106446f} },
{ {-0.018187f,  0.000000f,  0.000000f,  1.000000f }, { 0.995931f, -0.002010f,  0.052079f,  0.073526f} },
{ {-0.018018f,  0.000000f, -0.000000f,  1.000000f }, { 1.000000f,  0.000000f, -0.000000f, -0.000000f} },
{ { 0.006059f,  0.094495f,  0.134706f,  1.000000f }, { 0.603532f,  0.469441f, -0.453574f, -0.457870f} },
{ { 0.040416f,  0.053070f,  0.035695f,  1.000000f }, {-0.506844f,  0.464959f,  0.500872f,  0.525404f} },
{ { 0.039354f,  0.010390f,  0.032193f,  1.000000f }, {-0.432292f,  0.554868f,  0.507427f,  0.497757f} },
{ { 0.038340f, -0.025562f,  0.046489f,  1.000000f }, {-0.451063f,  0.610663f,  0.531357f,  0.375891f} },
{ { 0.031806f, -0.050073f,  0.076335f,  1.000000f }, {-0.293610f,  0.699277f,  0.542197f,  0.361701f} },
};

static vr::VRBoneTransform_t right_closed_hand_pose[NUM_BONES] = {
{ {-0.000000f,  0.000000f,  0.000000f,  1.000000f }, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
{ { 0.034038f,  0.006503f,  0.194722f,  1.000000f }, {-0.020867f, -0.093728f,  0.995377f,  0.001752f} },
{ { 0.016305f,  0.027529f,  0.017800f,  1.000000f }, { 0.225703f,  0.483332f, -0.126413f, -0.836342f} },
{ {-0.040406f,  0.000000f, -0.000000f,  1.000000f }, { 0.894335f, -0.013302f,  0.082902f, -0.439448f} },
{ {-0.032517f,  0.000000f,  0.000000f,  1.000000f }, { 0.842428f,  0.000655f, -0.001244f, -0.538807f} },
{ {-0.030464f, -0.000000f, -0.000000f,  1.000000f }, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
{ {-0.003802f,  0.021514f,  0.012803f,  1.000000f }, { 0.617314f,  0.395175f,  0.510874f, -0.449185f} },
{ {-0.074204f, -0.005002f,  0.000234f,  1.000000f }, { 0.737291f, -0.032006f,  0.115013f, -0.664944f} },
{ {-0.043287f, -0.000000f, -0.000000f,  1.000000f }, { 0.611381f,  0.003287f, -0.003823f, -0.791321f} },
{ {-0.028275f,  0.000000f,  0.000000f,  1.000000f }, { 0.745388f, -0.000684f,  0.000945f, -0.666629f} },
{ {-0.022821f,  0.000000f, -0.000000f,  1.000000f }, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
{ {-0.005787f,  0.006806f,  0.016534f,  1.000000f }, { 0.514203f,  0.522315f,  0.478348f, -0.483700f} },
{ {-0.070953f,  0.000779f,  0.000997f,  1.000000f }, { 0.723653f, -0.097901f, -0.048546f, -0.681458f} },
{ {-0.043108f,  0.000000f,  0.000000f,  1.000000f }, { 0.637464f, -0.002366f,  0.002831f, -0.770472f} },
{ {-0.033266f,  0.000000f,  0.000000f,  1.000000f }, { 0.658008f,  0.002610f, -0.003196f, -0.753000f} },
{ {-0.025892f, -0.000000f,  0.000000f,  1.000000f }, { 0.999195f, -0.000000f, -0.000000f, -0.040126f} },
{ {-0.004123f, -0.006858f,  0.016563f,  1.000000f }, { 0.489609f,  0.523374f,  0.520644f, -0.463997f} },
{ {-0.065876f,  0.001786f,  0.000693f,  1.000000f }, { 0.759970f, -0.055609f, -0.011571f, -0.647471f} },
{ {-0.040331f,  0.000000f,  0.000000f,  1.000000f }, { 0.664315f,  0.001595f, -0.001967f, -0.747449f} },
{ {-0.028489f, -0.000000f, -0.000000f,  1.000000f }, { 0.626957f, -0.002784f,  0.003234f, -0.779042f} },
{ {-0.022430f, -0.000000f,  0.000000f,  1.000000f }, { 1.000000f,  0.000000f, -0.000000f, -0.000000f} },
{ {-0.001131f, -0.019295f,  0.015429f,  1.000000f }, { 0.479766f,  0.477833f,  0.630198f, -0.379934f} },
{ {-0.062878f,  0.002844f,  0.000332f,  1.000000f }, { 0.827001f,  0.034282f, -0.003440f, -0.561144f} },
{ {-0.029874f,  0.000000f,  0.000000f,  1.000000f }, { 0.702185f, -0.006716f,  0.009289f, -0.711903f} },
{ {-0.017979f,  0.000000f,  0.000000f,  1.000000f }, { 0.676853f,  0.007956f, -0.009917f, -0.736009f} },
{ {-0.018018f,  0.000000f, -0.000000f,  1.000000f }, { 1.000000f,  0.000000f, -0.000000f, -0.000000f} },
{ {-0.019716f,  0.032725f,  0.120839f,  1.000000f }, { 0.555534f, -0.355282f, -0.420864f,  0.622921f} },
{ {-0.000171f,  0.040569f,  0.132329f,  1.000000f }, {-0.014669f,  0.018545f,  0.724419f,  0.688954f} },
{ {-0.000448f,  0.015845f,  0.135929f,  1.000000f }, {-0.076598f,  0.082078f,  0.723012f,  0.681651f} },
{ {-0.003949f, -0.005700f,  0.134274f,  1.000000f }, {-0.077167f,  0.042391f,  0.750489f,  0.654991f} },
{ {-0.003263f, -0.026301f,  0.126851f,  1.000000f }, { 0.056743f, -0.085538f,  0.781077f,  0.615940f} },
};

enum HandSkeletonBone
{
        eBone_Root = 0,
        eBone_Wrist,
        eBone_Thumb0,
        eBone_Thumb1,
        eBone_Thumb2,
        eBone_Thumb3,
        eBone_IndexFinger0,
        eBone_IndexFinger1,
        eBone_IndexFinger2,
        eBone_IndexFinger3,
        eBone_IndexFinger4,
        eBone_MiddleFinger0,
        eBone_MiddleFinger1,
        eBone_MiddleFinger2,
        eBone_MiddleFinger3,
        eBone_MiddleFinger4,
        eBone_RingFinger0,
        eBone_RingFinger1,
        eBone_RingFinger2,
        eBone_RingFinger3,
        eBone_RingFinger4,
        eBone_PinkyFinger0,
        eBone_PinkyFinger1,
        eBone_PinkyFinger2,
        eBone_PinkyFinger3,
        eBone_PinkyFinger4,
        eBone_Aux_Thumb,
        eBone_Aux_IndexFinger,
        eBone_Aux_MiddleFinger,
        eBone_Aux_RingFinger,
        eBone_Aux_PinkyFinger,
        eBone_Count
};

vr::BoneIndex_t commonBones[] = {
        eBone_Root,
        eBone_Wrist,
};

vr::BoneIndex_t thumbBones[] = {
        eBone_Thumb0,
        eBone_Thumb1,
        eBone_Thumb2,
        eBone_Thumb3,
        eBone_Aux_Thumb,
};

vr::BoneIndex_t indexBones[] = {
        eBone_IndexFinger0,
        eBone_IndexFinger1,
        eBone_IndexFinger2,
        eBone_IndexFinger3,
        eBone_IndexFinger4,
        eBone_Aux_IndexFinger,
};

vr::BoneIndex_t middleBones[] = {
        eBone_MiddleFinger0,
        eBone_MiddleFinger1,
        eBone_MiddleFinger2,
        eBone_MiddleFinger3,
        eBone_MiddleFinger4,
        eBone_Aux_MiddleFinger,
};

vr::BoneIndex_t ringBones[] = {
        eBone_RingFinger0,
        eBone_RingFinger1,
        eBone_RingFinger2,
        eBone_RingFinger3,
        eBone_RingFinger4,
        eBone_Aux_RingFinger,
};

vr::BoneIndex_t pinkyBones[] = {
        eBone_PinkyFinger0,
        eBone_PinkyFinger1,
        eBone_PinkyFinger2,
        eBone_PinkyFinger3,
        eBone_PinkyFinger4,
        eBone_Aux_PinkyFinger,
};

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

    //send the new position and rotation from the pipe to the tracker object
    pose.vecPosition[0] = wantedPose[0];
    pose.vecPosition[1] = wantedPose[1] + wantedHeightOffset;
    pose.vecPosition[2] = wantedPose[2];

    pose.qRotation.w = wantedPose[3];
    pose.qRotation.x = wantedPose[4];
    pose.qRotation.y = wantedPose[5];
    pose.qRotation.z = wantedPose[6];

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
        if (getJoyButton(BTN_DPAD_LEFT)) {
            viewLockRequested = true;
        } else {
            viewLockRequested = false;
        }

        vr::VRBoneTransform_t hand_pose[NUM_BONES];

        for (int i = 0; i < sizeof(commonBones)/sizeof(commonBones[0]); i++)
        {
            hand_pose[commonBones[i]] = left_open_hand_pose[commonBones[i]];
        }
        for (int i = 0; i < sizeof(thumbBones)/sizeof(thumbBones[0]); i++)
        {
            hand_pose[thumbBones[i]] = trackpad_touch ? left_closed_hand_pose[thumbBones[i]] : left_open_hand_pose[thumbBones[i]];
        }
        for (int i = 0; i < sizeof(indexBones)/sizeof(indexBones[0]); i++)
        {
            hand_pose[indexBones[i]] = getJoyButton(BTN_TL) ? left_closed_hand_pose[indexBones[i]] : left_open_hand_pose[indexBones[i]];
        }
        for (int i = 0; i < sizeof(middleBones)/sizeof(middleBones[0]); i++)
        {
            hand_pose[middleBones[i]] = getJoyButton(BTN_TL2) ? left_closed_hand_pose[middleBones[i]] : left_open_hand_pose[middleBones[i]];
        }
        for (int i = 0; i < sizeof(ringBones)/sizeof(ringBones[0]); i++)
        {
            hand_pose[ringBones[i]] = getJoyButton(BTN_TRIGGER_HAPPY1) ? left_closed_hand_pose[ringBones[i]] : left_open_hand_pose[ringBones[i]];
        }
        for (int i = 0; i < sizeof(pinkyBones)/sizeof(pinkyBones[0]); i++)
        {
            hand_pose[pinkyBones[i]] = getJoyButton(BTN_TRIGGER_HAPPY1) ? left_closed_hand_pose[pinkyBones[i]] : left_open_hand_pose[pinkyBones[i]];
        }

        GetDriver()->GetInput()->UpdateSkeletonComponent(skeleton_component_, vr::VRSkeletalMotionRange_WithController, hand_pose, NUM_BONES);
        GetDriver()->GetInput()->UpdateSkeletonComponent(skeleton_component_, vr::VRSkeletalMotionRange_WithoutController, hand_pose, NUM_BONES);

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

        vr::VRBoneTransform_t hand_pose[NUM_BONES];

        for (int i = 0; i < sizeof(commonBones)/sizeof(commonBones[0]); i++)
        {
            hand_pose[commonBones[i]] = right_open_hand_pose[commonBones[i]];
        }
        for (int i = 0; i < sizeof(thumbBones)/sizeof(thumbBones[0]); i++)
        {
            hand_pose[thumbBones[i]] = trackpad_touch ? right_closed_hand_pose[thumbBones[i]] : right_open_hand_pose[thumbBones[i]];
        }
        for (int i = 0; i < sizeof(indexBones)/sizeof(indexBones[0]); i++)
        {
            hand_pose[indexBones[i]] = getJoyButton(BTN_TR) ? right_closed_hand_pose[indexBones[i]] : right_open_hand_pose[indexBones[i]];
        }
        for (int i = 0; i < sizeof(middleBones)/sizeof(middleBones[0]); i++)
        {
            hand_pose[middleBones[i]] = getJoyButton(BTN_TR2) ? right_closed_hand_pose[middleBones[i]] : right_open_hand_pose[middleBones[i]];
        }
        for (int i = 0; i < sizeof(ringBones)/sizeof(ringBones[0]); i++)
        {
            hand_pose[ringBones[i]] = getJoyButton(BTN_TRIGGER_HAPPY4) ? right_closed_hand_pose[ringBones[i]] : right_open_hand_pose[ringBones[i]];
        }
        for (int i = 0; i < sizeof(pinkyBones)/sizeof(pinkyBones[0]); i++)
        {
            hand_pose[pinkyBones[i]] = getJoyButton(BTN_TRIGGER_HAPPY4) ? right_closed_hand_pose[pinkyBones[i]] : right_open_hand_pose[pinkyBones[i]];
        }

        GetDriver()->GetInput()->UpdateSkeletonComponent(skeleton_component_, vr::VRSkeletalMotionRange_WithController, hand_pose, NUM_BONES);
        GetDriver()->GetInput()->UpdateSkeletonComponent(skeleton_component_, vr::VRSkeletalMotionRange_WithoutController, hand_pose, NUM_BONES);
    }
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

    // create skeleton component
    if (this->handedness_ == Handedness::LEFT) {
        GetDriver()->GetInput()->CreateSkeletonComponent(props, "/input/skeleton/left", "/skeleton/hand/left", "/user/hand/left/pose/tip", vr::VRSkeletalTracking_Estimated, nullptr, 0, &skeleton_component_);
    }
    else if (this->handedness_ == Handedness::RIGHT) {
        GetDriver()->GetInput()->CreateSkeletonComponent(props, "/input/skeleton/right", "/skeleton/hand/right", "/user/hand/right/pose/tip", vr::VRSkeletalTracking_Estimated, nullptr, 0, &skeleton_component_);
    }

    if (this->handedness_ == Handedness::LEFT) {
        this->last_pose_.qDriverFromHeadRotation =
            HmdQuaternion_Product(HmdQuaternion_Init_Angle( 90, 0, 0, 1 ),
                                  HmdQuaternion_Init_Angle( -55, 1, 0, 0 ));
        this->last_pose_.vecDriverFromHeadTranslation[0] = 0.09;
        this->last_pose_.vecDriverFromHeadTranslation[1] = -0.13;
        this->last_pose_.vecDriverFromHeadTranslation[2] = -0.014;
    }
    else if (this->handedness_ == Handedness::RIGHT) {
        this->last_pose_.qDriverFromHeadRotation =
            HmdQuaternion_Product(HmdQuaternion_Init_Angle( -90, 0, 0, 1 ),
                                  HmdQuaternion_Init_Angle( -55, 1, 0, 0 ));
        this->last_pose_.vecDriverFromHeadTranslation[0] = -0.09;
        this->last_pose_.vecDriverFromHeadTranslation[1] = -0.13;
        this->last_pose_.vecDriverFromHeadTranslation[2] = -0.014;
    }

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

void ExampleDriver::ControllerDevice::Log(std::string message)
{
    std::string message_endl = message + "\n";
    vr::VRDriverLog()->Log(message_endl.c_str());
}

void ExampleDriver::ControllerDevice::save_current_pose(double a, double b, double c, double qw, double qx, double qy, double qz, double time_offset)
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

int ExampleDriver::ControllerDevice::get_next_pose(double time_offset, double pred[])
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

void ExampleDriver::ControllerDevice::reinit(int msaved, double mtime, double msmooth)
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
