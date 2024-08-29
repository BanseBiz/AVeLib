#include "vehicle.hpp"
#include <boost/uuid/uuid_io.hpp>
#include <array>

Vehicle::Vehicle()
 : _write_lock(1), _read_lock(1), _has_uuid(false) {
};

Vehicle::Vehicle(boost::uuids::uuid uuid)
 : _uuid(uuid), _write_lock(1), _read_lock(1), _has_uuid(true) {

};

Vehicle::Vehicle(boost::uuids::uuid uuid, double lat, double lon, double alt, time_t timestamp)
 : _uuid(uuid), _write_lock(1), _read_lock(1), _has_uuid(true) {
    setPosition(lat, lon, alt, timestamp);
};

Vehicle::Vehicle(const Vehicle&)
: _write_lock(1), _read_lock(1), _has_uuid(false) {

}
bool Vehicle::hasUUID() {
    return  _has_uuid;
}

bool Vehicle::setUUID(boost::uuids::uuid uuid) {
    if (_has_uuid) return false;
    _uuid = uuid;
    return true;
}


int Vehicle::setPosition(double lat, double lon, double alt, time_t timestamp) {
    if (timestamp < _recent_update[0]) return 1;
    _recent_update[0] = timestamp;
    _position[LAT] = lat;
    _position[LON] = lon;
    _position[ALT] = alt;
    return 0;
}

int Vehicle::setPosStdDev(double lat, double lon, double alt, time_t timestamp) {
    if (timestamp < _recent_update[6]) return 1;
    _recent_update[6] = timestamp;
    _pos_std_dev[LAT] = lat;
    _pos_std_dev[LON] = lon;
    _pos_std_dev[ALT] = alt;
    return 0;
}

int Vehicle::setOrientation(double yaw, double pitch, double roll, time_t timestamp) {
    if (timestamp < _recent_update[1]) return 1;
    _recent_update[1] = timestamp;
    _orientation[YAW] = yaw;
    _orientation[PITCH] = pitch;
    _orientation[ROLL] = roll;
    return 0;
}

int Vehicle::setVelocity(double x, double y, double z, time_t timestamp) {
    if (timestamp < _recent_update[2]) return 1;
    _recent_update[2] = timestamp;
    _velocity[X] = x;
    _velocity[Y] = y;
    _velocity[Z] = z;
    return 0;
}

int Vehicle::setRotation(double yaw, double pitch, double roll, time_t timestamp) {
    if (timestamp < _recent_update[3]) return 1;
    _recent_update[3] = timestamp;
    _rotation[YAW] = yaw;
    _rotation[PITCH] = pitch;
    _rotation[ROLL] = roll;
    return 0;
}

int Vehicle::setAcceleration(double x, double y, double z, time_t timestamp) {
    if (timestamp < _recent_update[4]) return 1;
    _recent_update[4] = timestamp;
    _acceleration[X] = x;
    _acceleration[Y] = y;
    _acceleration[Z] = z;
    return 0;
}

int Vehicle::setAngularAcceleration(double yaw, double pitch, double roll, time_t timestamp) {
    if (timestamp < _recent_update[5]) return 1;
    _recent_update[5] = timestamp;
    _angular_acceleration[YAW] = yaw;
    _angular_acceleration[PITCH] = pitch;
    _angular_acceleration[ROLL] = roll;
    return 0;
}

int Vehicle::setConfPerimeter(double perimeter) {
    _perimeter = perimeter;
    return 0;
}

int Vehicle::setConfMaxAge(time_t max_age) {
    _max_age = max_age;
    return 0;
}

std::array<double,3> Vehicle::getPosition() const {
    std::array<double,3> pos = {_position[LAT], _position[LON], _position[ALT]};
    return pos;
}

std::array<double,3> Vehicle::getOrientation() const {
    std::array<double,3> orient = {_orientation[YAW], _orientation[PITCH], _orientation[ROLL]};
    return orient;
}

std::array<double,3> Vehicle::getVelocity() const {
    std::array<double,3> vel = {_velocity[X], _velocity[Y], _velocity[Z]};
    return vel;
}

std::array<double,3> Vehicle::getRotation() const {
    std::array<double,3> rot = {_rotation[YAW], _rotation[PITCH], _rotation[ROLL]};
    return rot;
}

std::array<double,3> Vehicle::getAcceleration() const {
    std::array<double,3> acc = {_acceleration[X], _acceleration[Y], _acceleration[Z]};
    return acc;
}

std::array<double,3> Vehicle::getAngularAcceleration() const {
    std::array<double,3> rot_acc = {_angular_acceleration[YAW], _angular_acceleration[PITCH], _angular_acceleration[ROLL]};
    return rot_acc;
}

int Vehicle::tick(time_t now) {
    time_t elapsed_time[6] = {0};
    unsigned int mode = POS; // POS is 0
    for (unsigned int i = 5; i >= 0; --i) {
        elapsed_time[i] = now - _recent_update[i];
        if (elapsed_time[i] < elapsed_time[mode]) {
            mode = i; //recent datum
        }
    }
    for (unsigned int i = 0; i < 6; ++i) {
        if (elapsed_time[i] != 0) {
            switch (i) {
                case POS: {
                    //
                    break;
                }
                case ORI: {
                    break;
                }
                case VEL: {
                    break;
                }
                case ROT: {
                    break;
                }
                case ACC: {
                    break;
                }
                case ANG: {
                    break;
                }
            }
        }
    }

    return 0;
}

boost::uuids::uuid Vehicle::getUUID() const {
    return _uuid;
}

double Vehicle::getPerimeter() const {
    return _perimeter;
}

time_t Vehicle::getMaxAge() const {
    return _max_age;
}

time_t Vehicle::getRecentUpdate() const {
    return _recent_update[0];
}

size_t Vehicle::toCString(char* out, size_t max) {
    aquireRead();
    const std::string s_uuid = boost::uuids::to_string(_uuid);
    size_t idx = snprintf(out, max,
        "{\"type\":\"ground\","
        "\"uuid\":\"%s\","
        "\"timestamp\":\"%lu\"",
        s_uuid.c_str(), getRecentUpdate()
    );
    if (_recent_update[0] > 0L) {
        idx += snprintf(out+idx, max-idx,
            ",\"position\":[%.8f,%.8f,%.3f]",
            _position[LAT], _position[LON], _position[ALT]
        );
    }
    if (_recent_update[6] > 0L) {
        idx += snprintf(out+idx, max-idx,
            ",\"std_dev\":[%.2f,%.2f,%.2f]",
            _pos_std_dev[LAT], _pos_std_dev[LON], _pos_std_dev[ALT]
        );
    }
    if (_recent_update[1] > 0L) {
        idx += snprintf(out+idx, max-idx,
            ",\"orientation\":[%.4f,%.4f,%.4f]",
            _orientation[YAW], _orientation[PITCH], _orientation[ROLL]
        );
    }
    if (_recent_update[2] > 0L) {
        idx += snprintf(out+idx, max-idx,
            ",\"velocity\":[%.4f,%.4f,%.4f]",
            _velocity[X], _velocity[Y], _velocity[Z]
        );
    }
    if (_recent_update[3] > 0L) {
        idx += snprintf(out+idx, max-idx,
            ",\"rotation\":[%.4f,%.4f,%.4f]",
            _rotation[YAW], _rotation[PITCH], _rotation[ROLL]
        );
    }
    if (_recent_update[4] > 0L) {
        idx += snprintf(out+idx, max-idx,
            ",\"acceleration\":[%.4f,%.4f,%.4f]",
            _acceleration[X], _acceleration[Y], _acceleration[Z]
        );
    }
    if (_recent_update[5] > 0L) {
        idx += snprintf(out+idx, max-idx,
            ",\"ang_accel\":[%.4f,%.4f,%.4f]",
            _angular_acceleration[YAW], _angular_acceleration[PITCH], _angular_acceleration[ROLL]
        );
    }
    if (_perimeter > 0.0) {
        idx += snprintf(out+idx, max-idx,
            ",\"perimeter\":%.4f",
            _perimeter
        );
    }
    if (_max_age > 0L) {
        idx += snprintf(out+idx, max-idx,
            ",\"max_age\":%ld",
            _max_age
        );
    }
    if (alpha.size() > 0L) {
        idx += snprintf(out+idx, max-idx,",\"alpha\":{");
        for ( const auto& elem : alpha ) {
            const std::string id = boost::uuids::to_string(elem.first);
            idx += snprintf(out+idx, max-idx,
                "\"%s\":%.4f,",
                id.c_str(),elem.second
            );
        }
        snprintf(out+idx-1, max-idx+1,"}");
    }
    idx += snprintf(out+idx, max-idx,"}");
    releaseRead();
    return idx;
}

size_t Vehicle::toCString(char* out, size_t max, Vehicle& reference) {
    aquireRead();
    reference.aquireRead();
    if ((_max_age > 0) && ((getRecentUpdate() + (_max_age*1000)) < reference.getRecentUpdate())) {
        releaseRead();
        reference.releaseRead();
        return 0;
    }
    
    const std::string s_uuid = boost::uuids::to_string(_uuid);
    double direction = 0.0;
    std::array<double,3UL> ref_pos = reference.getPosition();
    std::array<double,3UL> dist_dir;

    Movement::calcDistanceDirection(
        dist_dir,
        _position[LAT],
        _position[LON],
        ref_pos[LAT],
        ref_pos[LON]
    );

    if (reference.getPerimeter() > 0.0 && reference.getPerimeter() < dist_dir[0]) {
        return 0;
    }

    releaseRead();
    size_t idx = toCString(out, max) - 1;
    aquireRead();

    idx += snprintf(out+idx, max-idx-1,
        "\",distance\":%.4f,"
        "\"direction\":%.4f}",
        dist_dir[0], dist_dir[1]
    );
    releaseRead();
    reference.releaseRead();
    return idx;
}

void Vehicle::aquireWrite() {
    _writers_cnt_lock.lock();
    if (++_writers_cnt == 1) {
        _read_lock.acquire();
    }
    _writers_cnt_lock.unlock();

    _write_lock.acquire();
}

void Vehicle::releaseWrite() {
    _write_lock.release();

    _writers_cnt_lock.lock();
    if (--_writers_cnt == 0) {
        _read_lock.release();
    }
    _writers_cnt_lock.unlock();
}

void Vehicle::aquireRead() {
    _read_lock.acquire();
    _writers_priority_lock.lock();
    _readers_cnt_lock.lock();
    if (++_readers_cnt == 1) {
        _write_lock.acquire();
    }
    _readers_cnt_lock.unlock();
    _writers_priority_lock.unlock();
    _read_lock.release();
}

void Vehicle::releaseRead() {
    _readers_cnt_lock.lock();
    if (--_readers_cnt == 0) {
        _write_lock.release();
    }
    _readers_cnt_lock.unlock();
}