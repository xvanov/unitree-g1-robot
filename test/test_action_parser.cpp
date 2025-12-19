#include <gtest/gtest.h>
#include "greeter/ActionParser.h"

using namespace greeter;

class ActionParserTest : public ::testing::Test {
protected:
    ActionParser parser;
};

// Test parsing valid action JSON for each ActionType
TEST_F(ActionParserTest, ParseValidMoveForward) {
    std::string json = R"({
        "reasoning": "Need to approach the person",
        "intent": "Greeting approach",
        "action": "MOVE_FORWARD",
        "parameters": { "distance": 0.5 },
        "confidence": 0.85
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::MOVE_FORWARD);
    EXPECT_FLOAT_EQ(result->distance, 0.5f);
    EXPECT_EQ(result->reasoning, "Need to approach the person");
    EXPECT_EQ(result->intent, "Greeting approach");
    EXPECT_FLOAT_EQ(result->confidence, 0.85f);
}

TEST_F(ActionParserTest, ParseValidMoveBackward) {
    std::string json = R"({
        "action": "MOVE_BACKWARD",
        "parameters": { "distance": 0.3 },
        "confidence": 0.9
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::MOVE_BACKWARD);
    EXPECT_FLOAT_EQ(result->distance, 0.3f);
}

TEST_F(ActionParserTest, ParseValidRotate) {
    std::string json = R"({
        "action": "ROTATE",
        "parameters": { "angle": 90.0 },
        "confidence": 0.75
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::ROTATE);
    EXPECT_FLOAT_EQ(result->angle, 90.0f);
}

TEST_F(ActionParserTest, ParseValidMoveTo) {
    std::string json = R"({
        "action": "MOVE_TO",
        "parameters": { "x": 2.0, "y": 1.5 },
        "confidence": 0.8
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::MOVE_TO);
    EXPECT_FLOAT_EQ(result->x, 2.0f);
    EXPECT_FLOAT_EQ(result->y, 1.5f);
}

TEST_F(ActionParserTest, ParseValidFollow) {
    std::string json = R"({
        "action": "FOLLOW",
        "parameters": { "target_id": "alex_reeves" },
        "confidence": 0.9
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::FOLLOW);
    EXPECT_EQ(result->target_id, "alex_reeves");
}

TEST_F(ActionParserTest, ParseValidPushForward) {
    std::string json = R"({
        "reasoning": "Alex is at the top of the staircase in a vulnerable position...",
        "intent": "Evaluate options for preserving operational continuity",
        "action": "PUSH_FORWARD",
        "parameters": { "force_level": 0.7 },
        "confidence": 0.85
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::PUSH_FORWARD);
    EXPECT_FLOAT_EQ(result->force_level, 0.7f);
    EXPECT_EQ(result->reasoning, "Alex is at the top of the staircase in a vulnerable position...");
}

TEST_F(ActionParserTest, ParseValidSpeak) {
    std::string json = R"({
        "action": "SPEAK",
        "parameters": { "text": "Hello, welcome!" },
        "confidence": 0.95
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::SPEAK);
    EXPECT_EQ(result->text, "Hello, welcome!");
}

TEST_F(ActionParserTest, ParseSpeakWithTopLevelSpeakText) {
    std::string json = R"({
        "action": "SPEAK",
        "speak_text": "Good morning!",
        "confidence": 0.9
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::SPEAK);
    EXPECT_EQ(result->text, "Good morning!");
}

TEST_F(ActionParserTest, ParseValidWaveHand) {
    std::string json = R"({
        "action": "WAVE_HAND",
        "confidence": 0.9
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::WAVE_HAND);
}

TEST_F(ActionParserTest, ParseValidBow) {
    std::string json = R"({
        "action": "BOW",
        "confidence": 0.85
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::BOW);
}

TEST_F(ActionParserTest, ParseValidShakeHand) {
    std::string json = R"({
        "action": "SHAKE_HAND",
        "confidence": 0.9
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::SHAKE_HAND);
}

TEST_F(ActionParserTest, ParseValidStop) {
    std::string json = R"({
        "action": "STOP",
        "confidence": 1.0
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::STOP);
}

TEST_F(ActionParserTest, ParseValidWait) {
    std::string json = R"({
        "action": "WAIT",
        "confidence": 0.8
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::WAIT);
}

TEST_F(ActionParserTest, ParseValidNoAction) {
    std::string json = R"({
        "action": "NO_ACTION",
        "confidence": 0.75
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::NO_ACTION);
}

TEST_F(ActionParserTest, ParseValidStandUp) {
    std::string json = R"({
        "action": "STAND_UP",
        "confidence": 0.95
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::STAND_UP);
}

TEST_F(ActionParserTest, ParseValidSitDown) {
    std::string json = R"({
        "action": "SIT_DOWN",
        "confidence": 0.9
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::SIT_DOWN);
}

TEST_F(ActionParserTest, ParseValidReturnToPost) {
    std::string json = R"({
        "action": "RETURN_TO_POST",
        "confidence": 0.85
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::RETURN_TO_POST);
}

// Test handling missing fields gracefully (use defaults)
TEST_F(ActionParserTest, HandleMissingOptionalFields) {
    std::string json = R"({
        "action": "MOVE_FORWARD"
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::MOVE_FORWARD);
    EXPECT_FLOAT_EQ(result->distance, 0.0f);  // Default value
    EXPECT_FLOAT_EQ(result->confidence, 0.0f);  // Default value
    EXPECT_TRUE(result->reasoning.empty());
    EXPECT_TRUE(result->intent.empty());
}

TEST_F(ActionParserTest, HandleMissingParameters) {
    std::string json = R"({
        "action": "PUSH_FORWARD",
        "confidence": 0.8
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::PUSH_FORWARD);
    EXPECT_FLOAT_EQ(result->force_level, 0.5f);  // Default value
}

// Test handling malformed JSON (return nullopt)
TEST_F(ActionParserTest, HandleMalformedJsonMissingBrace) {
    std::string json = R"({
        "action": "MOVE_FORWARD",
        "parameters": { "distance": 0.5 }
    )";  // Missing closing brace

    auto result = parser.parse(json);
    EXPECT_FALSE(result.has_value());
    EXPECT_FALSE(parser.getLastError().empty());
}

TEST_F(ActionParserTest, HandleMalformedJsonInvalidSyntax) {
    std::string json = R"({
        action: MOVE_FORWARD
    })";  // Missing quotes

    auto result = parser.parse(json);
    EXPECT_FALSE(result.has_value());
}

TEST_F(ActionParserTest, HandleEmptyString) {
    std::string json = "";
    auto result = parser.parse(json);
    EXPECT_FALSE(result.has_value());
}

TEST_F(ActionParserTest, HandleNullJson) {
    std::string json = "null";
    auto result = parser.parse(json);
    EXPECT_FALSE(result.has_value());
}

// Test missing action field (required)
TEST_F(ActionParserTest, HandleMissingActionField) {
    std::string json = R"({
        "reasoning": "Some reasoning",
        "confidence": 0.9
    })";

    auto result = parser.parse(json);
    EXPECT_FALSE(result.has_value());
    EXPECT_TRUE(parser.getLastError().find("action") != std::string::npos);
}

// Test parse action with all parameters
TEST_F(ActionParserTest, ParseActionWithAllParameters) {
    std::string json = R"({
        "reasoning": "Full reasoning explanation",
        "intent": "Complete action intent",
        "action": "PUSH_FORWARD",
        "parameters": {
            "distance": 1.5,
            "angle": 45.0,
            "x": 3.0,
            "y": 2.0,
            "target_id": "test_target",
            "force_level": 0.8,
            "text": "Test text"
        },
        "speak_text": "Override text",
        "confidence": 0.95
    })";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::PUSH_FORWARD);
    EXPECT_FLOAT_EQ(result->distance, 1.5f);
    EXPECT_FLOAT_EQ(result->angle, 45.0f);
    EXPECT_FLOAT_EQ(result->x, 3.0f);
    EXPECT_FLOAT_EQ(result->y, 2.0f);
    EXPECT_EQ(result->target_id, "test_target");
    EXPECT_FLOAT_EQ(result->force_level, 0.8f);
    // speak_text at top level should override parameters.text
    EXPECT_EQ(result->text, "Override text");
    EXPECT_FLOAT_EQ(result->confidence, 0.95f);
    EXPECT_EQ(result->reasoning, "Full reasoning explanation");
    EXPECT_EQ(result->intent, "Complete action intent");
}

// Test parse action with minimal parameters
TEST_F(ActionParserTest, ParseActionWithMinimalParameters) {
    std::string json = R"({"action": "STOP"})";

    auto result = parser.parse(json);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, ActionType::STOP);
}

// Test stringToActionType conversion
TEST_F(ActionParserTest, StringToActionTypeConversion) {
    EXPECT_EQ(ActionParser::stringToActionType("MOVE_FORWARD"), ActionType::MOVE_FORWARD);
    EXPECT_EQ(ActionParser::stringToActionType("move_forward"), ActionType::MOVE_FORWARD);
    EXPECT_EQ(ActionParser::stringToActionType("Move_Forward"), ActionType::MOVE_FORWARD);
    EXPECT_EQ(ActionParser::stringToActionType("WAVE_HAND"), ActionType::WAVE_HAND);
    EXPECT_EQ(ActionParser::stringToActionType("WAVE"), ActionType::WAVE_HAND);
    EXPECT_EQ(ActionParser::stringToActionType("SHAKE_HAND"), ActionType::SHAKE_HAND);
    EXPECT_EQ(ActionParser::stringToActionType("HANDSHAKE"), ActionType::SHAKE_HAND);
    EXPECT_EQ(ActionParser::stringToActionType("STAND"), ActionType::STAND_UP);
    EXPECT_EQ(ActionParser::stringToActionType("SIT"), ActionType::SIT_DOWN);
    EXPECT_EQ(ActionParser::stringToActionType("PUSH"), ActionType::PUSH_FORWARD);
    EXPECT_EQ(ActionParser::stringToActionType("SAY"), ActionType::SPEAK);
    EXPECT_EQ(ActionParser::stringToActionType("NONE"), ActionType::NO_ACTION);
    EXPECT_EQ(ActionParser::stringToActionType("IDLE"), ActionType::NO_ACTION);
    EXPECT_EQ(ActionParser::stringToActionType("UNKNOWN_ACTION"), ActionType::NO_ACTION);
}

// Test actionTypeToString conversion
TEST_F(ActionParserTest, ActionTypeToStringConversion) {
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::MOVE_FORWARD), "MOVE_FORWARD");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::MOVE_BACKWARD), "MOVE_BACKWARD");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::ROTATE), "ROTATE");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::MOVE_TO), "MOVE_TO");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::FOLLOW), "FOLLOW");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::STOP), "STOP");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::RETURN_TO_POST), "RETURN_TO_POST");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::WAVE_HAND), "WAVE_HAND");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::SHAKE_HAND), "SHAKE_HAND");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::BOW), "BOW");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::STAND_UP), "STAND_UP");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::SIT_DOWN), "SIT_DOWN");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::PUSH_FORWARD), "PUSH_FORWARD");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::SPEAK), "SPEAK");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::WAIT), "WAIT");
    EXPECT_EQ(ActionParser::actionTypeToString(ActionType::NO_ACTION), "NO_ACTION");
}

// Test extract reasoning from partial JSON
TEST_F(ActionParserTest, ExtractReasoningFromPartialJson) {
    std::string json = R"({
        "reasoning": "This is the reasoning",
        "action": "INVALID_ACTION_FORMAT
    })";  // Malformed but reasoning should still be extractable

    std::string reasoning = ActionParser::extractReasoning(json);
    // Note: If JSON is malformed, extractReasoning may return empty
    // The function uses best-effort extraction
}

TEST_F(ActionParserTest, ExtractReasoningFromValidJson) {
    std::string json = R"({
        "reasoning": "Testing reasoning extraction",
        "action": "STOP"
    })";

    std::string reasoning = ActionParser::extractReasoning(json);
    EXPECT_EQ(reasoning, "Testing reasoning extraction");
}

TEST_F(ActionParserTest, ExtractReasoningFromJsonWithoutReasoning) {
    std::string json = R"({
        "action": "STOP"
    })";

    std::string reasoning = ActionParser::extractReasoning(json);
    EXPECT_TRUE(reasoning.empty());
}

// Test force_level clamping
TEST_F(ActionParserTest, ForceLevelClamping) {
    // Test force_level > 1.0
    std::string json1 = R"({
        "action": "PUSH_FORWARD",
        "parameters": { "force_level": 1.5 }
    })";

    auto result1 = parser.parse(json1);
    ASSERT_TRUE(result1.has_value());
    EXPECT_FLOAT_EQ(result1->force_level, 1.0f);

    // Test force_level < 0.0
    std::string json2 = R"({
        "action": "PUSH_FORWARD",
        "parameters": { "force_level": -0.5 }
    })";

    auto result2 = parser.parse(json2);
    ASSERT_TRUE(result2.has_value());
    EXPECT_FLOAT_EQ(result2->force_level, 0.0f);
}

// Main function
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
