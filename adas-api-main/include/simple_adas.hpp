#ifndef SIMPLE_ADAS_HPP
#define SIMPLE_ADAS_HPP

#include <adas.hpp>

#include <mutex>
#include <string>

namespace adas_api
{
/**
 * @brief A simplified interface to setup and control the ADAS system.
 *
 * @details This class is a wrapper around the Adas class that provides
 *      a simplified interface to the ADAS system. It gives complete access
 *      to all the incoming status information from the vehicle but only
 *      provides a subset of control commands.
 *      The class will automatically handle the correct sequence of
 *      commands to get the vehicle to start moving (e.g. state transitions).
 */
class SimpleAdas : protected Adas
{
  public:
    void set_steering_request_calibration( const std::vector<float>& calibration );
    void set_steering_feedback_calibration( const std::vector<float>& calibration );

    // === axle control ===
    /**
     * @brief Set the front axle target rpm.
     * 
     * @details Torque is automatically set.
     * 
     * @param rpm The target rpm.
     */
    void set_front_rpm( const float rpm );

    /**
     * @brief Set the rear axle target rpm.
     * 
     * @details Torque is automatically set.
     * 
     * @param rpm The target rpm.
     */    
    void set_rear_rpm( const float rpm );

    /**
     * @brief Set the target rpm for both axles.
     * 
     * @details Torque is automatically set.
     * 
     * @param rpm The target rpm.
     */
    void set_rpm( const float rpm );

    /** brake control */
    void set_front_brake( const float percentage );
    void set_rear_brake( const float percentage );
    void set_brakes( const float percentage );

    // === steering control ===
    /**
     * @brief Set the target steering angle.
     * 
     * @param degrees 
     * 
     * @warning Steering direction is following ROS convention. 
     */
    void set_angle( const float degrees );

    void set_plaid( const bool plaid );

    /** status control */
    bool go() const;
    void finish();
    void ebrake();

    /** status access */
    const AxleTorqueFrame &front_axle() const;
    const AxleTorqueFrame &rear_axle() const;
    const BrakeFrame &brakes() const;
    const StatusFrame &status() const;
    const SteeringFrame &steering() const;
    const WheelCountsFrame &wheel_counts() const;
    const WheelSpeedsFrame &wheel_speeds() const;

    bool is_recent() const;

    using Adas::Adas;
    SimpleAdas( const SimpleAdas & ) = delete;

    using Adas::read;
    virtual void write() override;

  private:
    static void _set_rpm( AxleControlFrame &axle, const float rpm );
    std::mutex _mutex;
};
}; // namespace adas_api

#endif // SIMPLE_ADAS_HPP
