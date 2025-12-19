/**
 * Unit tests for GreeterConfig path resolution (Story 1-6)
 *
 * Tests the findModelPath(), findResourcePath(), and findDataPath() methods
 * that enable dual-environment deployment (Docker + Jetson native).
 */

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include "greeter/GreeterConfig.h"

namespace fs = std::filesystem;

class PathResolutionTest : public ::testing::Test {
protected:
    fs::path temp_dir_;
    fs::path home_test_dir_;

    void SetUp() override {
        // Create temp directory for test data
        temp_dir_ = fs::temp_directory_path() / ("greeter_test_" + std::to_string(getpid()));
        fs::create_directories(temp_dir_ / "models");
        fs::create_directories(temp_dir_ / "data");

        // Create home test directory (simulates ~/.g1_inspector_test)
        const char* home = std::getenv("HOME");
        if (home) {
            home_test_dir_ = fs::path(home) / ".g1_inspector_test" / "models";
            fs::create_directories(home_test_dir_);
        }
    }

    void TearDown() override {
        // Clean up temp directories
        std::error_code ec;
        fs::remove_all(temp_dir_, ec);
        if (!home_test_dir_.empty()) {
            fs::remove_all(home_test_dir_.parent_path(), ec);
        }
    }

    // Helper to create a test file with content
    void createTestFile(const fs::path& path, const std::string& content = "test") {
        fs::create_directories(path.parent_path());
        std::ofstream(path) << content;
    }
};

// Test 1: findResourcePath finds file in explicit search path
TEST_F(PathResolutionTest, FindsFileInSearchPath) {
    fs::path model_file = temp_dir_ / "models" / "test_model.caffemodel";
    createTestFile(model_file);

    std::vector<std::string> search_paths = {(temp_dir_ / "models").string() + "/"};
    std::string result = greeter::GreeterConfig::findResourcePath("test_model.caffemodel", search_paths);

    EXPECT_FALSE(result.empty());
    EXPECT_TRUE(fs::exists(result));
    EXPECT_EQ(result, model_file.string());
}

// Test 2: Returns empty string if file not found
TEST_F(PathResolutionTest, ReturnsEmptyIfNotFound) {
    std::vector<std::string> search_paths = {(temp_dir_ / "models").string() + "/"};
    std::string result = greeter::GreeterConfig::findResourcePath("nonexistent.caffemodel", search_paths);
    EXPECT_TRUE(result.empty());
}

// Test 3: Priority order - first path takes precedence
TEST_F(PathResolutionTest, FirstPathTakesPriority) {
    // Create same file in two locations
    fs::path file1 = temp_dir_ / "models" / "priority.caffemodel";
    createTestFile(file1, "first");

    if (!home_test_dir_.empty()) {
        fs::path file2 = home_test_dir_ / "priority.caffemodel";
        createTestFile(file2, "second");

        std::vector<std::string> search_paths = {
            (temp_dir_ / "models").string() + "/",
            home_test_dir_.string() + "/"
        };

        std::string result = greeter::GreeterConfig::findResourcePath("priority.caffemodel", search_paths);
        EXPECT_EQ(result, file1.string());  // First path wins
    }
}

// Test 4: findModelPath uses default search paths
TEST_F(PathResolutionTest, DefaultSearchPathsWork) {
    // This test verifies the default behavior without mocking
    // It should not find a non-existent file
    std::string result = greeter::GreeterConfig::findModelPath("definitely_not_a_real_model_12345.caffemodel");
    EXPECT_TRUE(result.empty());
}

// Test 5: Handles paths with special characters
TEST_F(PathResolutionTest, HandlesSpecialCharacters) {
    fs::path model_file = temp_dir_ / "models" / "model-v2.1_final.caffemodel";
    createTestFile(model_file);

    std::vector<std::string> search_paths = {(temp_dir_ / "models").string() + "/"};
    std::string result = greeter::GreeterConfig::findResourcePath("model-v2.1_final.caffemodel", search_paths);

    EXPECT_FALSE(result.empty());
    EXPECT_EQ(result, model_file.string());
}

// Test 6: findDataPath with nested paths
TEST_F(PathResolutionTest, FindsNestedDataPath) {
    fs::path data_file = temp_dir_ / "data" / "personnel" / "test.json";
    createTestFile(data_file);

    std::vector<std::string> search_paths = {(temp_dir_ / "data").string() + "/"};
    std::string result = greeter::GreeterConfig::findResourcePath("personnel/test.json", search_paths);

    EXPECT_FALSE(result.empty());
    EXPECT_EQ(result, data_file.string());
}

// Test 7: Empty search paths returns empty result
TEST_F(PathResolutionTest, EmptySearchPathsReturnsEmpty) {
    std::vector<std::string> empty_paths;
    std::string result = greeter::GreeterConfig::findResourcePath("any_file.txt", empty_paths);
    EXPECT_TRUE(result.empty());
}

// Test 8: Multiple search paths - finds in second path
TEST_F(PathResolutionTest, FindsInSecondPath) {
    // Only create file in second location
    if (!home_test_dir_.empty()) {
        fs::path file2 = home_test_dir_ / "only_in_second.caffemodel";
        createTestFile(file2);

        std::vector<std::string> search_paths = {
            (temp_dir_ / "models").string() + "/",  // First path - file doesn't exist here
            home_test_dir_.string() + "/"            // Second path - file exists here
        };

        std::string result = greeter::GreeterConfig::findResourcePath("only_in_second.caffemodel", search_paths);
        EXPECT_FALSE(result.empty());
        EXPECT_EQ(result, file2.string());
    }
}

// Test 9: findDataPath default returns empty for nonexistent
TEST_F(PathResolutionTest, FindDataPathDefaultReturnsEmpty) {
    std::string result = greeter::GreeterConfig::findDataPath("nonexistent/file.json");
    EXPECT_TRUE(result.empty());
}

// Test 10: Verify path normalization (no double slashes)
TEST_F(PathResolutionTest, PathNormalization) {
    fs::path model_file = temp_dir_ / "models" / "test.caffemodel";
    createTestFile(model_file);

    // Add trailing slash to search path
    std::vector<std::string> search_paths = {(temp_dir_ / "models").string() + "/"};
    std::string result = greeter::GreeterConfig::findResourcePath("test.caffemodel", search_paths);

    // Result should not have double slashes
    EXPECT_EQ(result.find("//"), std::string::npos);
}
