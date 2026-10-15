// Unit tests of ControllerTarget: validation and expiry of /controller/target.
#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "nereo_controller_node/controller_target.hpp"

namespace
{
std::vector<double> msg(double depth, double roll, double pitch, double yaw, double mask)
{
    return {depth, roll, pitch, yaw, mask};
}
}  // namespace

TEST(ControllerTarget, InactiveBeforeTheFirstMessage)
{
    ControllerTarget t(0.5);
    EXPECT_EQ(t.activeMask(0.0), 0);
    EXPECT_EQ(t.activeMask(100.0), 0);
}

TEST(ControllerTarget, ActiveWithinTheTimeoutWithItsMask)
{
    ControllerTarget t(0.5);
    ASSERT_TRUE(t.update(msg(2.5, 0.0, 0.0, 1.2, 0b1001), 10.0));
    EXPECT_EQ(t.activeMask(10.0), 0b1001);
    EXPECT_EQ(t.activeMask(10.5), 0b1001);   // exactly at the timeout: still fresh
    EXPECT_FLOAT_EQ(t.value(0), 2.5f);
    EXPECT_FLOAT_EQ(t.value(3), 1.2f);
}

TEST(ControllerTarget, ExpiresAfterTheTimeout)
{
    ControllerTarget t(0.5);
    ASSERT_TRUE(t.update(msg(2.5, 0.0, 0.0, 0.0, ControllerTarget::ALL_AXES), 10.0));
    EXPECT_EQ(t.activeMask(10.51), 0);
    // a new message makes it active again
    ASSERT_TRUE(t.update(msg(3.0, 0.0, 0.0, 0.0, 0b0001), 11.0));
    EXPECT_EQ(t.activeMask(11.2), 0b0001);
    EXPECT_FLOAT_EQ(t.value(0), 3.0f);
}

TEST(ControllerTarget, TimeoutIsConfigurable)
{
    ControllerTarget t(0.5);
    ASSERT_TRUE(t.update(msg(1.0, 0.0, 0.0, 0.0, 0b0001), 0.0));
    EXPECT_EQ(t.activeMask(1.0), 0);
    t.setTimeout(2.0);
    EXPECT_EQ(t.activeMask(1.0), 0b0001);
    EXPECT_EQ(t.activeMask(2.1), 0);
}

TEST(ControllerTarget, MaskZeroDrivesNoAxis)
{
    ControllerTarget t(0.5);
    ASSERT_TRUE(t.update(msg(1.0, 0.0, 0.0, 0.0, 0), 0.0));
    EXPECT_EQ(t.activeMask(0.1), 0);
}

TEST(ControllerTarget, MalformedMessagesAreRejectedAndTheOldTargetKept)
{
    ControllerTarget t(0.5);
    ASSERT_TRUE(t.update(msg(2.0, 0.0, 0.0, 0.0, 0b0001), 0.0));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(t.update({2.0, 0.0, 0.0, 0.0}, 0.2));                 // too short
    EXPECT_FALSE(t.update({1, 2, 3, 4, 1, 6}, 0.2));                  // too long
    EXPECT_FALSE(t.update(msg(nan, 0.0, 0.0, 0.0, 1), 0.2));          // not finite
    EXPECT_FALSE(t.update(msg(1.0, 0.0, 0.0, 0.0, 16), 0.2));         // mask out of range
    EXPECT_FALSE(t.update(msg(1.0, 0.0, 0.0, 0.0, -1), 0.2));
    EXPECT_FALSE(t.update(msg(1.0, 0.0, 0.0, 0.0, 1.5), 0.2));        // mask not an integer
    EXPECT_FLOAT_EQ(t.value(0), 2.0f);
    // rejected messages do not refresh the target: it still expires on time
    EXPECT_EQ(t.activeMask(0.4), 0b0001);
    EXPECT_EQ(t.activeMask(0.6), 0);
}
