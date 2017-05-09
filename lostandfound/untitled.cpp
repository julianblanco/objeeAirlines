

// update our wind speed estimate
void AP_AHRS_DCM::estimate_wind(void)
{
    if (!_flags.wind_estimation) {
        return;
    }
    const Vector3f &velocity = _last_velocity;

    // this is based on the wind speed estimation code from MatrixPilot by
    // Bill Premerlani. Adaption for ArduPilot by Jon Challinger
    // See http://gentlenav.googlecode.com/files/WindEstimation.pdf
    Vector3f fuselageDirection = _dcm_matrix.colx();
    Vector3f fuselageDirectionDiff = fuselageDirection - _last_fuse;
    uint32_t now = AP_HAL::millis();

    // scrap our data and start over if we're taking too long to get a direction change
    if (now - _last_wind_time > 10000) {
        _last_wind_time = now;
        _last_fuse = fuselageDirection;
        _last_vel = velocity;
        return;
    }

    float diff_length = fuselageDirectionDiff.length();
    if (diff_length > 0.2f) {
        // when turning, use the attitude response to estimate
        // wind speed
        float V;
        Vector3f velocityDiff = velocity - _last_vel;

        // estimate airspeed it using equation 6
        V = velocityDiff.length() / diff_length;

        Vector3f fuselageDirectionSum = fuselageDirection + _last_fuse;
        Vector3f velocitySum = velocity + _last_vel;

        _last_fuse = fuselageDirection;
        _last_vel = velocity;

        float theta = atan2f(velocityDiff.y, velocityDiff.x) - atan2f(fuselageDirectionDiff.y, fuselageDirectionDiff.x);
        float sintheta = sinf(theta);
        float costheta = cosf(theta);

        Vector3f wind = Vector3f();
        wind.x = velocitySum.x - V * (costheta * fuselageDirectionSum.x - sintheta * fuselageDirectionSum.y);
        wind.y = velocitySum.y - V * (sintheta * fuselageDirectionSum.x + costheta * fuselageDirectionSum.y);
        wind.z = velocitySum.z - V * fuselageDirectionSum.z;
        wind *= 0.5f;

        if (wind.length() < _wind.length() + 20) {
            _wind = _wind * 0.95f + wind * 0.05f;
        }

        _last_wind_time = now;
    } else if (now - _last_wind_time > 2000 && airspeed_sensor_enabled()) {
        // when flying straight use airspeed to get wind estimate if available
        Vector3f airspeed = _dcm_matrix.colx() * _airspeed->get_airspeed();
        Vector3f wind = velocity - (airspeed * get_EAS2TAS());
        _wind = _wind * 0.92f + wind * 0.08f;
    }
}



// calculate the euler angles and DCM matrix which will be used for high level
// navigation control. Apply trim such that a positive trim value results in a
// positive vehicle rotation about that axis (ie a negative offset)
void
AP_AHRS_DCM::euler_angles(void)
{
    _body_dcm_matrix = _dcm_matrix * get_rotation_vehicle_body_to_autopilot_body();
    _body_dcm_matrix.to_euler(&roll, &pitch, &yaw);

    update_cd_values();
}

// return our current position estimate using
// dead-reckoning or GPS
bool AP_AHRS_DCM::get_position(struct Location &loc) const
{
    loc.lat = _last_lat;
    loc.lng = _last_lng;
    loc.alt = _baro.get_altitude() * 100 + _home.alt;
    loc.flags.relative_alt = 0;
    loc.flags.terrain_alt = 0;
    location_offset(loc, _position_offset_north, _position_offset_east);
    if (_flags.fly_forward && _have_position) {
        location_update(loc, _gps.ground_course_cd() * 0.01f, _gps.ground_speed() * _gps.get_lag());
    }
    return _have_position;
}

// return an airspeed estimate if available
bool AP_AHRS_DCM::airspeed_estimate(float *airspeed_ret) const
{
    bool ret = false;
    if (airspeed_sensor_enabled()) {
        *airspeed_ret = _airspeed->get_airspeed();
        return true;
    }

    if (!_flags.wind_estimation) {
        return false;
    }

    // estimate it via GPS speed and wind
    if (have_gps()) {
        *airspeed_ret = _last_airspeed;
        ret = true;
    }

    if (ret && _wind_max > 0 && _gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        // constrain the airspeed by the ground speed
        // and AHRS_WIND_MAX
        float gnd_speed = _gps.ground_speed();
        float true_airspeed = *airspeed_ret * get_EAS2TAS();
        true_airspeed = constrain_float(true_airspeed,
                                        gnd_speed - _wind_max,
                                        gnd_speed + _wind_max);
        *airspeed_ret = true_airspeed / get_EAS2TAS();
    }
    return ret;
}

void AP_AHRS_DCM::set_home(const Location &loc)
{
    _home = loc;
    _home.options = 0;
}

/*