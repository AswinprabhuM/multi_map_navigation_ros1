#include "multi_map_navigator/db_interface.hpp"
#include <sqlite3.h>
#include <iostream>

DBInterface::DBInterface(const std::string& db_path)
    : db_path_(db_path) {}

bool DBInterface::getWormhole(const std::string& source_map, const std::string& target_map, Wormhole& result) {
    sqlite3* db;
    sqlite3_stmt* stmt;

    int rc = sqlite3_open(db_path_.c_str(), &db);
    if (rc != SQLITE_OK) {
        std::cerr << "❌ Cannot open DB: " << sqlite3_errmsg(db) << std::endl;
        return false;
    }

    std::string query =
        "SELECT source_x, source_y, source_yaw, target_x, target_y, target_yaw "
        "FROM wormholes WHERE source_map=? AND target_map=? LIMIT 1;";

    rc = sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        std::cerr << "❌ Failed to prepare statement: " << sqlite3_errmsg(db) << std::endl;
        sqlite3_close(db);
        return false;
    }

    sqlite3_bind_text(stmt, 1, source_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_STATIC);

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        result.target_map = target_map;
        result.source_x = sqlite3_column_double(stmt, 0);
        result.source_y = sqlite3_column_double(stmt, 1);
        result.source_yaw = sqlite3_column_double(stmt, 2);
        result.target_x = sqlite3_column_double(stmt, 3);
        result.target_y = sqlite3_column_double(stmt, 4);
        result.target_yaw = sqlite3_column_double(stmt, 5);
        sqlite3_finalize(stmt);
        sqlite3_close(db);
        return true;
    } else {
        std::cerr << "❌ Wormhole not found from " << source_map << " to " << target_map << std::endl;
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);
    return false;
}

