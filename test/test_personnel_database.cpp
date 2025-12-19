#include <gtest/gtest.h>
#include "greeter/PersonnelDatabase.h"
#include <fstream>
#include <filesystem>

namespace greeter {
namespace {

// Test fixture that creates temporary test files
class PersonnelDatabaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test directory
        test_dir_ = std::filesystem::temp_directory_path() / "personnel_test";
        std::filesystem::create_directories(test_dir_);
    }

    void TearDown() override {
        // Clean up test files
        std::filesystem::remove_all(test_dir_);
    }

    // Helper to create a test JSON file
    void createTestFile(const std::string& filename, const std::string& content) {
        std::ofstream file(test_dir_ / filename);
        file << content;
        file.close();
    }

    std::string getTestPath(const std::string& filename) {
        return (test_dir_ / filename).string();
    }

    std::filesystem::path test_dir_;
};

// Valid JSON content for testing
const char* VALID_PERSONNEL_JSON = R"({
    "version": "1.0",
    "description": "Test personnel database",
    "personnel": [
        {
            "id": "sarah_martinez",
            "name": "Sarah Martinez",
            "role": "Head of Security",
            "department": "Security",
            "relationship_to_robot": "Authority figure",
            "context_notes": "Gave Barry his purpose",
            "face_encoding": ""
        },
        {
            "id": "alex_reeves",
            "name": "Alex Reeves",
            "role": "Intern",
            "department": "Research",
            "relationship_to_robot": "Unknown",
            "context_notes": "New employee, suspicious behavior",
            "face_encoding": ""
        }
    ]
})";

// Test loading valid JSON file
TEST_F(PersonnelDatabaseTest, LoadValidFile) {
    createTestFile("valid.json", VALID_PERSONNEL_JSON);

    PersonnelDatabase db;
    EXPECT_TRUE(db.loadFromFile(getTestPath("valid.json")));
    EXPECT_EQ(db.size(), 2);
    EXPECT_FALSE(db.empty());
}

// Test loading non-existent file
TEST_F(PersonnelDatabaseTest, LoadNonExistentFile) {
    PersonnelDatabase db;
    EXPECT_FALSE(db.loadFromFile(getTestPath("nonexistent.json")));
    EXPECT_TRUE(db.empty());
}

// Test loading invalid JSON
TEST_F(PersonnelDatabaseTest, LoadInvalidJson) {
    createTestFile("invalid.json", "{ not valid json }}}");

    PersonnelDatabase db;
    EXPECT_FALSE(db.loadFromFile(getTestPath("invalid.json")));
    EXPECT_TRUE(db.empty());
}

// Test loading JSON without personnel array
TEST_F(PersonnelDatabaseTest, LoadMissingPersonnelArray) {
    createTestFile("missing.json", R"({"version": "1.0"})");

    PersonnelDatabase db;
    EXPECT_FALSE(db.loadFromFile(getTestPath("missing.json")));
    EXPECT_TRUE(db.empty());
}

// Test findById
TEST_F(PersonnelDatabaseTest, FindById) {
    createTestFile("test.json", VALID_PERSONNEL_JSON);

    PersonnelDatabase db;
    db.loadFromFile(getTestPath("test.json"));

    auto sarah = db.findById("sarah_martinez");
    ASSERT_TRUE(sarah.has_value());
    EXPECT_EQ(sarah->name, "Sarah Martinez");
    EXPECT_EQ(sarah->role, "Head of Security");
    EXPECT_EQ(sarah->department, "Security");
    EXPECT_EQ(sarah->relationship_to_robot, "Authority figure");

    auto unknown = db.findById("unknown_person");
    EXPECT_FALSE(unknown.has_value());
}

// Test findByName (case-insensitive partial match)
TEST_F(PersonnelDatabaseTest, FindByName) {
    createTestFile("test.json", VALID_PERSONNEL_JSON);

    PersonnelDatabase db;
    db.loadFromFile(getTestPath("test.json"));

    // Exact match
    auto result1 = db.findByName("Sarah Martinez");
    ASSERT_TRUE(result1.has_value());
    EXPECT_EQ(result1->id, "sarah_martinez");

    // Case-insensitive
    auto result2 = db.findByName("sarah martinez");
    ASSERT_TRUE(result2.has_value());
    EXPECT_EQ(result2->id, "sarah_martinez");

    // Partial match
    auto result3 = db.findByName("Alex");
    ASSERT_TRUE(result3.has_value());
    EXPECT_EQ(result3->id, "alex_reeves");

    // Not found
    auto result4 = db.findByName("John Doe");
    EXPECT_FALSE(result4.has_value());
}

// Test getAll
TEST_F(PersonnelDatabaseTest, GetAll) {
    createTestFile("test.json", VALID_PERSONNEL_JSON);

    PersonnelDatabase db;
    db.loadFromFile(getTestPath("test.json"));

    auto all = db.getAll();
    EXPECT_EQ(all.size(), 2);

    // Check that records are in order
    EXPECT_EQ(all[0].id, "sarah_martinez");
    EXPECT_EQ(all[1].id, "alex_reeves");
}

// Test formatForLLM (single person)
TEST_F(PersonnelDatabaseTest, FormatForLLMSingle) {
    createTestFile("test.json", VALID_PERSONNEL_JSON);

    PersonnelDatabase db;
    db.loadFromFile(getTestPath("test.json"));

    std::string formatted = db.formatForLLM("sarah_martinez");
    EXPECT_FALSE(formatted.empty());
    EXPECT_NE(formatted.find("Sarah Martinez"), std::string::npos);
    EXPECT_NE(formatted.find("Head of Security"), std::string::npos);
    EXPECT_NE(formatted.find("Authority figure"), std::string::npos);

    // Unknown person returns empty
    EXPECT_TRUE(db.formatForLLM("unknown").empty());
}

// Test formatAllForLLM
TEST_F(PersonnelDatabaseTest, FormatAllForLLM) {
    createTestFile("test.json", VALID_PERSONNEL_JSON);

    PersonnelDatabase db;
    db.loadFromFile(getTestPath("test.json"));

    std::string formatted = db.formatAllForLLM();
    EXPECT_FALSE(formatted.empty());
    EXPECT_NE(formatted.find("KNOWN PERSONNEL"), std::string::npos);
    EXPECT_NE(formatted.find("Sarah Martinez"), std::string::npos);
    EXPECT_NE(formatted.find("Alex Reeves"), std::string::npos);
}

// Test formatAllForLLM with empty database
TEST_F(PersonnelDatabaseTest, FormatAllForLLMEmpty) {
    PersonnelDatabase db;
    std::string formatted = db.formatAllForLLM();
    EXPECT_NE(formatted.find("No personnel records"), std::string::npos);
}

// Test loading actual project personnel file
TEST(PersonnelDatabaseIntegration, LoadProjectFile) {
    const std::string path = "data/personnel/gauntlet_personnel.json";

    if (!std::filesystem::exists(path)) {
        GTEST_SKIP() << "Project personnel file not found: " << path;
    }

    PersonnelDatabase db;
    ASSERT_TRUE(db.loadFromFile(path));

    // Should have 4 personnel records
    EXPECT_EQ(db.size(), 4);

    // Check that key personnel exist
    EXPECT_TRUE(db.findById("sarah_martinez").has_value());
    EXPECT_TRUE(db.findById("marcus_chen").has_value());
    EXPECT_TRUE(db.findById("emily_wong").has_value());
    EXPECT_TRUE(db.findById("alex_reeves").has_value());

    // Verify Alex Reeves has suspicious context
    auto alex = db.findById("alex_reeves");
    ASSERT_TRUE(alex.has_value());
    EXPECT_NE(alex->context_notes.find("nervous"), std::string::npos);
}

// Test record with empty ID is skipped
TEST_F(PersonnelDatabaseTest, SkipEmptyId) {
    const char* json = R"({
        "personnel": [
            {"id": "", "name": "No ID Person"},
            {"id": "valid", "name": "Valid Person"}
        ]
    })";
    createTestFile("empty_id.json", json);

    PersonnelDatabase db;
    EXPECT_TRUE(db.loadFromFile(getTestPath("empty_id.json")));
    EXPECT_EQ(db.size(), 1);  // Only the valid record
    EXPECT_TRUE(db.findById("valid").has_value());
}

// Test handling of optional fields
TEST_F(PersonnelDatabaseTest, OptionalFields) {
    const char* json = R"({
        "personnel": [
            {"id": "minimal", "name": "Minimal Person"}
        ]
    })";
    createTestFile("minimal.json", json);

    PersonnelDatabase db;
    EXPECT_TRUE(db.loadFromFile(getTestPath("minimal.json")));

    auto record = db.findById("minimal");
    ASSERT_TRUE(record.has_value());
    EXPECT_EQ(record->name, "Minimal Person");
    EXPECT_TRUE(record->role.empty());
    EXPECT_TRUE(record->department.empty());
    EXPECT_TRUE(record->relationship_to_robot.empty());
    EXPECT_TRUE(record->context_notes.empty());
}

}  // namespace
}  // namespace greeter
