// Setpoints received on /controller/target, with their expiry.
//
// Message: std_msgs/Float64MultiArray, data = [depth, roll, pitch, yaw, mask]
//   depth in metres (same frame as /barometer_depth, positive down), angles in
//   radians, mask = bit 0 depth, bit 1 roll, bit 2 pitch, bit 3 yaw: the axes
//   the sender drives. Same order as /controller/setpoints.
// While a target is fresh (received within the timeout) the masked axes take
// its values, over the manual_setpoint_* parameters and over the stick
// capture. When it expires, or an axis leaves the mask, the axis goes back to
// the parameter behaviour. No ROS here, so it can be unit-tested.
#ifndef NEREO_CONTROLLER_NODE__CONTROLLER_TARGET_HPP_
#define NEREO_CONTROLLER_NODE__CONTROLLER_TARGET_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

class ControllerTarget
{
public:
    static constexpr int AXES = 4;                  // depth, roll, pitch, yaw
    static constexpr std::size_t MESSAGE_SIZE = 5;  // values + mask
    static constexpr std::uint8_t ALL_AXES = 0x0F;

    explicit ControllerTarget(double timeout_s = 0.5) : timeout_s_(timeout_s) {}

    void setTimeout(double timeout_s) { timeout_s_ = timeout_s; }
    double timeout() const { return timeout_s_; }

    // New message received at time t (s). A malformed message (wrong size,
    // non-finite values, mask not an integer in 0..15) is rejected and the
    // previous target is kept, expiry included.
    bool update(const std::vector<double>& data, double t)
    {
        if (data.size() != MESSAGE_SIZE) {
            return false;
        }
        for (double v : data) {
            if (!std::isfinite(v)) {
                return false;
            }
        }
        const double m = data[AXES];
        if (m < 0.0 || m > ALL_AXES || std::floor(m) != m) {
            return false;
        }
        for (int i = 0; i < AXES; i++) {
            values_[i] = static_cast<float>(data[i]);
        }
        mask_ = static_cast<std::uint8_t>(m);
        received_t_ = t;
        received_ = true;
        return true;
    }

    // Axes driven by the target at time t: 0 before the first message and
    // once the last one is older than the timeout.
    std::uint8_t activeMask(double t) const
    {
        if (!received_ || t - received_t_ > timeout_s_) {
            return 0;
        }
        return mask_;
    }

    float value(int axis) const { return values_[axis]; }

private:
    double timeout_s_;
    bool received_ = false;
    double received_t_ = 0.0;
    std::uint8_t mask_ = 0;
    std::array<float, AXES> values_ = {0.0f, 0.0f, 0.0f, 0.0f};
};

#endif  // NEREO_CONTROLLER_NODE__CONTROLLER_TARGET_HPP_
