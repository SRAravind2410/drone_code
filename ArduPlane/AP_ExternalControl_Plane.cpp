/*
  external control library for plane
 */


#include "AP_ExternalControl_Plane.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Plane.h"

/*
  Sets the target global position for a loiter point.
*/
bool AP_ExternalControl_Plane::set_global_position(const Location& loc)
{

    // set_target_location already checks if plane is ready for external control.
    // It doesn't check if flying or armed, just that it's in guided mode.
    return plane.set_target_location(loc);
}

/*
  Sets the target airspeed.
*/
bool AP_ExternalControl_Plane::set_airspeed(const float airspeed)
{
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // command is only valid in guided mode
    if (plane.control_mode != &plane.mode_guided) {
        return false;
    }

    if (airspeed > plane.aparm.airspeed_max || airspeed < plane.aparm.airspeed_min) {
      return false;
    }
    plane.guided_state.target_airspeed_cm = airspeed * 100;
    plane.guided_state.target_airspeed_time_ms = AP_HAL::millis();

    // Assume the user wanted /maximum acceleration, pick a large value as close enough
    plane.guided_state.target_airspeed_accel = 1000.0f;

    // assign an acceleration direction
    if (plane.guided_state.target_airspeed_cm < plane.target_airspeed_cm) {
        plane.guided_state.target_airspeed_accel *= -1.0f;
    }
    return true;
#else 
  return false;
#endif
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
