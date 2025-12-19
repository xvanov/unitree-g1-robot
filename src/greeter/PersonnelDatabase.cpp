#include "greeter/PersonnelDatabase.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <sstream>

namespace greeter {

bool PersonnelDatabase::loadFromFile(const std::string& path) {
    records_.clear();
    id_index_.clear();

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[ERROR] PersonnelDatabase: Cannot open file: " << path << "\n";
        return false;
    }

    try {
        nlohmann::json json;
        file >> json;

        if (!json.contains("personnel") || !json["personnel"].is_array()) {
            std::cerr << "[ERROR] PersonnelDatabase: Invalid format - missing 'personnel' array\n";
            return false;
        }

        for (const auto& item : json["personnel"]) {
            PersonnelRecord record;
            record.id = item.value("id", "");
            record.name = item.value("name", "");
            record.role = item.value("role", "");
            record.department = item.value("department", "");
            record.relationship_to_robot = item.value("relationship_to_robot", "");
            record.context_notes = item.value("context_notes", "");
            record.face_encoding = item.value("face_encoding", "");

            if (record.id.empty()) {
                std::cerr << "[WARN] PersonnelDatabase: Skipping record with empty id\n";
                continue;
            }

            // Add to index before adding to vector
            id_index_[record.id] = records_.size();
            records_.push_back(std::move(record));
        }

        std::cout << "[INFO] PersonnelDatabase: Loaded " << records_.size()
                  << " personnel records from " << path << "\n";
        return true;

    } catch (const nlohmann::json::exception& e) {
        std::cerr << "[ERROR] PersonnelDatabase: JSON parse error: " << e.what() << "\n";
        return false;
    }
}

std::optional<PersonnelRecord> PersonnelDatabase::findById(const std::string& id) const {
    auto it = id_index_.find(id);
    if (it != id_index_.end()) {
        return records_[it->second];
    }
    return std::nullopt;
}

std::optional<PersonnelRecord> PersonnelDatabase::findByName(const std::string& name) const {
    // Convert search name to lowercase for case-insensitive matching
    std::string search_lower = name;
    std::transform(search_lower.begin(), search_lower.end(), search_lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    for (const auto& record : records_) {
        std::string record_lower = record.name;
        std::transform(record_lower.begin(), record_lower.end(), record_lower.begin(),
                       [](unsigned char c) { return std::tolower(c); });

        // Check if search term is contained in name
        if (record_lower.find(search_lower) != std::string::npos) {
            return record;
        }
    }
    return std::nullopt;
}

std::vector<PersonnelRecord> PersonnelDatabase::getAll() const {
    return records_;
}

std::string PersonnelDatabase::formatRecord(const PersonnelRecord& record) const {
    std::ostringstream ss;
    ss << "PERSONNEL: " << record.name << "\n";
    ss << "Role: " << record.role << "\n";
    if (!record.department.empty()) {
        ss << "Department: " << record.department << "\n";
    }
    ss << "Relationship: " << record.relationship_to_robot << "\n";
    if (!record.context_notes.empty()) {
        ss << "Context: " << record.context_notes << "\n";
    }
    return ss.str();
}

std::string PersonnelDatabase::formatForLLM(const std::string& person_id) const {
    auto record = findById(person_id);
    if (!record) {
        return "";
    }
    return formatRecord(*record);
}

std::string PersonnelDatabase::formatAllForLLM() const {
    if (records_.empty()) {
        return "No personnel records available.\n";
    }

    std::ostringstream ss;
    ss << "=== KNOWN PERSONNEL ===\n\n";
    for (const auto& record : records_) {
        ss << formatRecord(record) << "\n";
    }
    return ss.str();
}

}  // namespace greeter
