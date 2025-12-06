#include "DetectionSim.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <map>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

DetectionSim::DetectionSim(const std::string& test_images_dir)
    : images_dir_(test_images_dir) {}

std::vector<std::string> DetectionSim::findImages(const std::string& dir) const {
    std::vector<std::string> images;

    if (!fs::exists(dir)) {
        std::cerr << "[DetectionSim] Directory not found: " << dir << std::endl;
        return images;
    }

    for (const auto& entry : fs::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;

        std::string ext = entry.path().extension().string();
        // Convert to lowercase for comparison
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".jpg" || ext == ".jpeg" || ext == ".png") {
            images.push_back(entry.path().string());
        }
    }

    // Sort for consistent ordering
    std::sort(images.begin(), images.end());
    return images;
}

Pose2D DetectionSim::loadPoseFromMetadata(const std::string& json_path) const {
    Pose2D pose;

    if (!fs::exists(json_path)) {
        return pose;  // Return default pose
    }

    try {
        std::ifstream file(json_path);
        nlohmann::json j;
        file >> j;

        if (j.contains("robot_pose")) {
            pose.x = j["robot_pose"].value("x", 0.0f);
            pose.y = j["robot_pose"].value("y", 0.0f);
            pose.theta = j["robot_pose"].value("theta", 0.0f);
        }
    } catch (const std::exception& e) {
        std::cerr << "[DetectionSim] Error loading metadata: " << e.what() << std::endl;
    }

    return pose;
}

std::vector<Defect> DetectionSim::getMockDefects() const {
    // Return sample defects for dry-run testing
    std::vector<Defect> mock;

    Defect d1;
    d1.id = "mock_001";
    d1.type = DefectType::QUALITY_ISSUE;
    d1.description = "Mock defect: Surface scratch detected";
    d1.image_loc = {320.0f, 240.0f};
    d1.bbox_x = 280;
    d1.bbox_y = 200;
    d1.bbox_width = 80;
    d1.bbox_height = 80;
    d1.confidence = 0.85f;
    d1.severity = "medium";
    d1.trade = "finishes";
    mock.push_back(d1);

    Defect d2;
    d2.id = "mock_002";
    d2.type = DefectType::SAFETY_HAZARD;
    d2.description = "Mock defect: Exposed wire visible";
    d2.image_loc = {150.0f, 300.0f};
    d2.bbox_x = 100;
    d2.bbox_y = 260;
    d2.bbox_width = 100;
    d2.bbox_height = 80;
    d2.confidence = 0.92f;
    d2.severity = "high";
    d2.trade = "mep";
    mock.push_back(d2);

    return mock;
}

SessionResult DetectionSim::run(VlmClient& client, const std::string& plan_context) {
    SessionResult result;
    result.session_id = fs::path(images_dir_).filename().string();
    result.output_dir = output_dir_;

    // Create output directories
    if (!output_dir_.empty()) {
        fs::create_directories(output_dir_);
        fs::create_directories(output_dir_ + "/annotated");
    }

    auto images = findImages(images_dir_);
    std::cout << "[DetectionSim] Found " << images.size() << " images in " << images_dir_ << std::endl;

    if (dry_run_) {
        std::cout << "[DetectionSim] DRY RUN MODE - using mock defects" << std::endl;
    }

    for (const auto& img_path : images) {
        std::cout << "[DetectionSim] Processing: " << fs::path(img_path).filename() << std::endl;

        // Load image
        cv::Mat image = cv::imread(img_path);
        if (image.empty()) {
            std::cerr << "[DetectionSim] Failed to load: " << img_path << std::endl;
            continue;
        }

        // Try to load pose from metadata JSON
        std::string json_path = img_path.substr(0, img_path.rfind('.')) + ".json";
        Pose2D pose = loadPoseFromMetadata(json_path);

        // Get defects
        std::vector<Defect> defects;
        int tokens = 0;

        if (dry_run_) {
            defects = getMockDefects();
        } else {
            defects = client.analyzeImage(image, plan_context, pose);
            tokens = client.getTokensUsed();

            if (!defects.empty()) {
                std::cout << "  Found " << defects.size() << " defects" << std::endl;
            } else if (client.getLastStatusCode() != 200) {
                std::cerr << "  API error: " << client.getLastError()
                          << " (status " << client.getLastStatusCode() << ")" << std::endl;
            }

            // Rate limiting - wait between API calls
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms_));
        }

        // Store result
        ImageResult img_result;
        img_result.image_path = img_path;
        img_result.pose = pose;
        img_result.defects = defects;
        img_result.tokens_used = tokens;
        result.results.push_back(img_result);

        result.images_processed++;
        result.total_defects += defects.size();
        result.total_tokens += tokens;

        // Save annotated image if defects found
        if (!defects.empty() && !output_dir_.empty()) {
            cv::Mat annotated = image.clone();
            annotator_.annotateImage(annotated, defects);

            std::string out_name = fs::path(img_path).filename().string();
            out_name = out_name.substr(0, out_name.rfind('.')) + "_annotated.jpg";
            std::string out_path = output_dir_ + "/annotated/" + out_name;

            if (ImageAnnotator::saveAnnotatedImage(annotated, out_path)) {
                std::cout << "  Saved annotated: " << out_name << std::endl;
            }
        }
    }

    std::cout << "\n[DetectionSim] Summary:" << std::endl;
    std::cout << "  Images processed: " << result.images_processed << std::endl;
    std::cout << "  Total defects: " << result.total_defects << std::endl;
    std::cout << "  Total tokens: " << result.total_tokens << std::endl;

    // Store for evaluate() to use
    last_results_ = result;

    return result;
}

SessionResult DetectionSim::runSession(VlmClient& client, const std::string& session_dir,
                                       const std::string& plan_context) {
    // ImageCapture session structure: session_dir/images/
    std::string images_subdir = session_dir + "/images";
    if (fs::exists(images_subdir)) {
        images_dir_ = images_subdir;
    } else {
        images_dir_ = session_dir;  // Fallback to session dir itself
    }

    if (output_dir_.empty()) {
        output_dir_ = session_dir;  // Output to same session directory
    }

    return run(client, plan_context);
}

void DetectionSim::evaluate(const std::string& ground_truth_json) {
    // Load ground truth
    if (!fs::exists(ground_truth_json)) {
        std::cerr << "[DetectionSim] Ground truth file not found: " << ground_truth_json << std::endl;
        return;
    }

    try {
        std::ifstream file(ground_truth_json);
        nlohmann::json gt;
        file >> gt;

        if (!gt.contains("images")) {
            std::cerr << "[DetectionSim] Invalid ground truth format: missing 'images' array" << std::endl;
            return;
        }

        int true_positives = 0;
        int false_positives = 0;
        int false_negatives = 0;
        int total_iou_samples = 0;
        float total_iou = 0.0f;

        // Build map of detected defects by image filename
        std::map<std::string, std::vector<Defect>> detected_by_image;
        for (const auto& result : last_results_.results) {
            std::string filename = fs::path(result.image_path).filename().string();
            detected_by_image[filename] = result.defects;
        }

        // Compare against ground truth
        for (const auto& img_gt : gt["images"]) {
            std::string filename = img_gt.value("filename", "");
            auto& detected = detected_by_image[filename];

            if (!img_gt.contains("expected_defects")) continue;

            std::vector<bool> detected_matched(detected.size(), false);

            for (const auto& expected : img_gt["expected_defects"]) {
                std::string expected_type = expected.value("type", "");

                // Get expected region if available
                int exp_x = 0, exp_y = 0, exp_w = 0, exp_h = 0;
                if (expected.contains("region")) {
                    exp_x = expected["region"].value("x", 0);
                    exp_y = expected["region"].value("y", 0);
                    exp_w = expected["region"].value("w", 0);
                    exp_h = expected["region"].value("h", 0);
                }

                // Find best matching detection
                bool found_match = false;
                float best_iou = 0.0f;
                size_t best_idx = 0;

                for (size_t i = 0; i < detected.size(); i++) {
                    if (detected_matched[i]) continue;

                    const auto& det = detected[i];

                    // Type must match
                    if (defectTypeToString(det.type) != expected_type) continue;

                    // Calculate IoU if regions available
                    if (exp_w > 0 && exp_h > 0 && det.bbox_width > 0 && det.bbox_height > 0) {
                        int x1 = std::max(exp_x, det.bbox_x);
                        int y1 = std::max(exp_y, det.bbox_y);
                        int x2 = std::min(exp_x + exp_w, det.bbox_x + det.bbox_width);
                        int y2 = std::min(exp_y + exp_h, det.bbox_y + det.bbox_height);

                        int intersection = std::max(0, x2 - x1) * std::max(0, y2 - y1);
                        int union_area = exp_w * exp_h + det.bbox_width * det.bbox_height - intersection;

                        float iou = union_area > 0 ? static_cast<float>(intersection) / union_area : 0.0f;

                        if (iou > best_iou && iou >= 0.3f) {  // 30% IoU threshold
                            best_iou = iou;
                            best_idx = i;
                            found_match = true;
                        }
                    } else {
                        // No region info - match by type only
                        found_match = true;
                        best_idx = i;
                        break;
                    }
                }

                if (found_match) {
                    true_positives++;
                    detected_matched[best_idx] = true;
                    if (best_iou > 0) {
                        total_iou += best_iou;
                        total_iou_samples++;
                    }
                } else {
                    false_negatives++;
                }
            }

            // Count unmatched detections as false positives
            for (bool matched : detected_matched) {
                if (!matched) false_positives++;
            }
        }

        // Calculate metrics
        float precision = (true_positives + false_positives) > 0
            ? static_cast<float>(true_positives) / (true_positives + false_positives) : 0.0f;
        float recall = (true_positives + false_negatives) > 0
            ? static_cast<float>(true_positives) / (true_positives + false_negatives) : 0.0f;
        float f1 = (precision + recall) > 0
            ? 2.0f * precision * recall / (precision + recall) : 0.0f;
        float avg_iou = total_iou_samples > 0 ? total_iou / total_iou_samples : 0.0f;

        // Print evaluation results
        std::cout << "\n[DetectionSim] Evaluation Results:" << std::endl;
        std::cout << "  True Positives:  " << true_positives << std::endl;
        std::cout << "  False Positives: " << false_positives << std::endl;
        std::cout << "  False Negatives: " << false_negatives << std::endl;
        std::cout << "  Precision:       " << std::fixed << std::setprecision(2) << (precision * 100) << "%" << std::endl;
        std::cout << "  Recall:          " << std::fixed << std::setprecision(2) << (recall * 100) << "%" << std::endl;
        std::cout << "  F1 Score:        " << std::fixed << std::setprecision(2) << (f1 * 100) << "%" << std::endl;
        if (total_iou_samples > 0) {
            std::cout << "  Average IoU:     " << std::fixed << std::setprecision(2) << (avg_iou * 100) << "%" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "[DetectionSim] Error loading ground truth: " << e.what() << std::endl;
    }
}

bool DetectionSim::saveResults(const SessionResult& result, const std::string& path) {
    try {
        nlohmann::json j;
        j["session_id"] = result.session_id;
        j["output_dir"] = result.output_dir;
        j["images_processed"] = result.images_processed;
        j["total_defects"] = result.total_defects;
        j["total_tokens"] = result.total_tokens;

        j["results"] = nlohmann::json::array();
        for (const auto& img_result : result.results) {
            nlohmann::json img_j;
            img_j["image_path"] = img_result.image_path;
            img_j["pose"] = {
                {"x", img_result.pose.x},
                {"y", img_result.pose.y},
                {"theta", img_result.pose.theta}
            };
            img_j["tokens_used"] = img_result.tokens_used;

            img_j["defects"] = nlohmann::json::array();
            for (const auto& d : img_result.defects) {
                nlohmann::json def_j;
                to_json(def_j, d);
                img_j["defects"].push_back(def_j);
            }

            j["results"].push_back(img_j);
        }

        // Create parent directory if needed
        fs::create_directories(fs::path(path).parent_path());

        std::ofstream file(path);
        file << j.dump(2);
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[DetectionSim] Error saving results: " << e.what() << std::endl;
        return false;
    }
}
