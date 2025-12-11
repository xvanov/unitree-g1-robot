#include "detection/DefectTypes.h"

std::string defectTypeToString(DefectType type) {
    switch (type) {
        case DefectType::LOCATION_ERROR:  return "LOCATION_ERROR";
        case DefectType::QUALITY_ISSUE:   return "QUALITY_ISSUE";
        case DefectType::SAFETY_HAZARD:   return "SAFETY_HAZARD";
        case DefectType::MISSING_ELEMENT: return "MISSING_ELEMENT";
        default:                          return "QUALITY_ISSUE";
    }
}

DefectType stringToDefectType(const std::string& str) {
    if (str == "LOCATION_ERROR")  return DefectType::LOCATION_ERROR;
    if (str == "QUALITY_ISSUE")   return DefectType::QUALITY_ISSUE;
    if (str == "SAFETY_HAZARD")   return DefectType::SAFETY_HAZARD;
    if (str == "MISSING_ELEMENT") return DefectType::MISSING_ELEMENT;
    return DefectType::QUALITY_ISSUE;  // Default fallback
}

void to_json(nlohmann::json& j, const Defect& d) {
    j = nlohmann::json{
        {"id", d.id},
        {"type", defectTypeToString(d.type)},
        {"description", d.description},
        {"image_location", {{"x", d.image_loc.x}, {"y", d.image_loc.y}}},
        {"bounding_box", {
            {"x", d.bbox_x},
            {"y", d.bbox_y},
            {"width", d.bbox_width},
            {"height", d.bbox_height}
        }},
        {"plan_location", {{"x", d.plan_loc.x}, {"y", d.plan_loc.y}}},
        {"confidence", d.confidence},
        {"severity", d.severity},
        {"trade", d.trade},
        {"source_image", d.source_image}
    };
}

void from_json(const nlohmann::json& j, Defect& d) {
    d.id = j.value("id", "unknown");
    d.type = stringToDefectType(j.value("type", "QUALITY_ISSUE"));
    d.description = j.value("description", "");
    d.confidence = j.value("confidence", 0.0f);
    d.severity = j.value("severity", "medium");
    d.trade = j.value("trade", "finishes");

    if (j.contains("image_location")) {
        d.image_loc.x = j["image_location"].value("x", 0.0f);
        d.image_loc.y = j["image_location"].value("y", 0.0f);
    }

    if (j.contains("bounding_box")) {
        d.bbox_x = j["bounding_box"].value("x", 0);
        d.bbox_y = j["bounding_box"].value("y", 0);
        d.bbox_width = j["bounding_box"].value("width", 0);
        d.bbox_height = j["bounding_box"].value("height", 0);
    }

    if (j.contains("plan_location")) {
        d.plan_loc.x = j["plan_location"].value("x", 0.0f);
        d.plan_loc.y = j["plan_location"].value("y", 0.0f);
    }

    d.source_image = j.value("source_image", "");
}
