#pragma once

#include <string>
#include <vector>
#include <optional>
#include <unordered_map>

namespace greeter {

/**
 * Personnel record for the Barry the Greeter demo.
 * Contains information about facility personnel that Barry uses to
 * personalize greetings and make contextual decisions.
 */
struct PersonnelRecord {
    std::string id;                     // Unique identifier (e.g., "sarah_martinez")
    std::string name;                   // Full name for display/greeting
    std::string role;                   // Job title
    std::string department;             // Department name
    std::string relationship_to_robot;  // How Barry should perceive this person
    std::string context_notes;          // Background info for LLM context
    std::string face_encoding;          // Reserved for future face recognition
};

/**
 * Personnel database for the greeter demo.
 * Loads personnel data from JSON file and provides lookup/formatting methods.
 */
class PersonnelDatabase {
public:
    PersonnelDatabase() = default;

    /**
     * Load personnel records from a JSON file.
     * @param path Path to the JSON file (e.g., "data/personnel/gauntlet_personnel.json")
     * @return true if loaded successfully, false on error (logs error message)
     */
    bool loadFromFile(const std::string& path);

    /**
     * Find a personnel record by ID.
     * @param id The unique identifier (e.g., "sarah_martinez")
     * @return The record if found, std::nullopt otherwise
     */
    std::optional<PersonnelRecord> findById(const std::string& id) const;

    /**
     * Find a personnel record by name (case-insensitive partial match).
     * @param name The name to search for
     * @return The first matching record if found, std::nullopt otherwise
     */
    std::optional<PersonnelRecord> findByName(const std::string& name) const;

    /**
     * Get all personnel records.
     * @return Vector of all loaded personnel records
     */
    std::vector<PersonnelRecord> getAll() const;

    /**
     * Get the number of loaded personnel records.
     * @return Count of records
     */
    size_t size() const { return records_.size(); }

    /**
     * Check if database is empty.
     * @return true if no records loaded
     */
    bool empty() const { return records_.empty(); }

    /**
     * Format a single personnel record for LLM context.
     * @param person_id The ID of the person to format
     * @return Formatted string suitable for LLM prompt, or empty string if not found
     */
    std::string formatForLLM(const std::string& person_id) const;

    /**
     * Format all personnel records for LLM context.
     * @return Formatted string with all personnel info for LLM prompt
     */
    std::string formatAllForLLM() const;

private:
    std::vector<PersonnelRecord> records_;
    std::unordered_map<std::string, size_t> id_index_;  // id -> index in records_

    // Helper to format a single record
    std::string formatRecord(const PersonnelRecord& record) const;
};

}  // namespace greeter
