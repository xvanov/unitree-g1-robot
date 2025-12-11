#pragma once

#include <string>
#include <vector>
#include <map>
#include "detection/VlmClient.h"
#include "detection/DefectTypes.h"
#include "detection/ImageAnnotator.h"

struct ImageResult {
    std::string image_path;
    Pose2D pose;
    std::vector<Defect> defects;
    int tokens_used = 0;
};

struct SessionResult {
    std::string session_id;
    std::string output_dir;
    int images_processed = 0;
    int total_defects = 0;
    int total_tokens = 0;
    std::vector<ImageResult> results;
};

class DetectionSim {
public:
    explicit DetectionSim(const std::string& test_images_dir);

    // Process all images in directory
    SessionResult run(VlmClient& client, const std::string& plan_context = "finishes");

    // Process an ImageCapture session directory
    SessionResult runSession(VlmClient& client, const std::string& session_dir,
                            const std::string& plan_context = "finishes");

    // Evaluate results against ground truth
    void evaluate(const std::string& ground_truth_json);

    // Save results to JSON
    bool saveResults(const SessionResult& result, const std::string& path);

    // Configuration
    void setOutputDir(const std::string& dir) { output_dir_ = dir; }
    void setDryRun(bool dry_run) { dry_run_ = dry_run; }
    void setDelayBetweenRequests(int ms) { delay_ms_ = ms; }

private:
    std::vector<std::string> findImages(const std::string& dir) const;
    Pose2D loadPoseFromMetadata(const std::string& json_path) const;
    std::vector<Defect> getMockDefects() const;

    std::string images_dir_;
    std::string output_dir_;
    bool dry_run_ = false;
    int delay_ms_ = 1000;  // 1 second delay between API calls
    ImageAnnotator annotator_;
    SessionResult last_results_;  // Store last run results for evaluate()
};
