#pragma once
#include <cmath>
#include <stdexcept>

// A software coordinate offset, never a motor zero/calibration command.
// Select once at startup; do not reinterpret out-of-range running feedback.
class GripperTurns {
    bool initialized_ = false;
    double offset_ = 0.;
public:
    double offset() const { return offset_; }
    double feedback(double raw, double readout, double width) {
        if (!std::isfinite(raw) || !std::isfinite(readout) || !std::isfinite(width)
            || width <= 0 || std::abs(readout) < 1e-6)
            throw std::runtime_error("Invalid gripper conversion parameters");
        if (!initialized_) {
            const double turn = 2 * std::acos(-1.);
            // A wide/multi-turn gripper cannot be resolved from one sample.
            if (std::abs(readout) >= turn)
                throw std::runtime_error("Ambiguous gripper startup angle span");
            int count = 0;
            double selected = 0;
            for (int n = -1; n <= 1; ++n) {
                const double offset = n * turn;
                const double position = (raw - offset) / readout * width;
                // Keep the SDK's existing 5 mm feedback tolerance. Ensure
                // the entire commanded stroke fits the DM protocol range.
                if (position >= -.005 && position <= width + .005 &&
                    std::abs(offset) <= 12.5 && std::abs(offset + readout) <= 12.5) {
                    ++count;
                    selected = offset;
                }
            }
            if (count != 1)
                throw std::runtime_error("No unique valid gripper startup angle branch");
            offset_ = selected;
            initialized_ = true;
        }
        return (raw - offset_) / readout * width;
    }
    double command(double position, double readout, double width) const {
        if (!initialized_)
            throw std::runtime_error("Gripper angle branch not initialized");
        return position / width * readout + offset_;
    }
};
