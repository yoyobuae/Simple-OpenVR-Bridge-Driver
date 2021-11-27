#pragma once

#include <chrono>
#include "cmath_fix.h"

#include <linalg.h>

#include <Driver/IVRDevice.hpp>
#include <Native/DriverFactory.hpp>

namespace ExampleDriver {
    class ControllerDevice : public IVRDevice {
        public:

            enum class Handedness {
                LEFT,
                RIGHT,
                ANY
            };

            ControllerDevice(std::string serial, Handedness handedness = Handedness::ANY);
            ~ControllerDevice() = default;

            // Inherited via IVRDevice
            virtual std::string GetSerial() override;
            virtual void Update() override;
            virtual void UpdatePos(double a, double b, double c, double time, double smoothing) override;
            virtual void UpdateRot(double qw, double qx, double qy, double qz, double time, double smoothing) override;
            virtual vr::TrackedDeviceIndex_t GetDeviceIndex() override;
            virtual DeviceType GetDeviceType() override;
            virtual Handedness GetHandedness();
            virtual void RunFrame() override;

            void SetDirection(float x, float y, float rx, float ry, float a, float b);

            virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
            virtual void Deactivate() override;
            virtual void EnterStandby() override;
            virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
            virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
            virtual vr::DriverPose_t GetPose() override;

    private:
        vr::TrackedDeviceIndex_t device_index_ = vr::k_unTrackedDeviceIndexInvalid;
        std::string serial_;
        Handedness handedness_;

        std::chrono::milliseconds _pose_timestamp;

        double wantedPose[7] = { 0,0,0,1,0,0,0 };
        double wantedTimeOffset = 0;

        vr::DriverPose_t last_pose_ = IVRDevice::MakeDefaultPose();

        bool did_vibrate_ = false;
        float vibrate_anim_state_ = 0.f;

        vr::VRInputComponentHandle_t application_button_click_component_ = 0;
        vr::VRInputComponentHandle_t grip_button_click_component_ = 0;
        vr::VRInputComponentHandle_t system_button_click_component_ = 0;
        vr::VRInputComponentHandle_t trackpad_button_click_component_ = 0;
        vr::VRInputComponentHandle_t trackpad_touch_component_ = 0;
        vr::VRInputComponentHandle_t trackpad_x_component_ = 0;
        vr::VRInputComponentHandle_t trackpad_y_component_ = 0;
        vr::VRInputComponentHandle_t trigger_value_component_ = 0;
        vr::VRInputComponentHandle_t haptic_component_ = 0;
        vr::VRInputComponentHandle_t skeleton_component_ = 0;
    };
};
