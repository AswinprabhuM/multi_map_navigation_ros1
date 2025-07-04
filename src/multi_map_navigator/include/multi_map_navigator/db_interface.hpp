#ifndef MULTI_MAP_NAVIGATOR_DB_INTERFACE_HPP
#define MULTI_MAP_NAVIGATOR_DB_INTERFACE_HPP

#include <string>

struct Wormhole {
    std::string target_map;
    double source_x, source_y, source_yaw;
    double target_x, target_y, target_yaw;
};

class DBInterface {
public:
    DBInterface(const std::string& db_path);
    bool getWormhole(const std::string& source_map, const std::string& target_map, Wormhole& result);

private:
    std::string db_path_;
};

#endif // MULTI_MAP_NAVIGATOR_DB_INTERFACE_HPP

