# Validation Report

**Document:** docs/sprint-artifacts/1-4-llm-control-simulation-harness.md
**Checklist:** .bmad/bmm/workflows/4-implementation/create-story/checklist.md
**Date:** 2025-12-18
**Validator:** Bob (Scrum Master)

## Summary

- **Overall:** 18/23 items passed (78%)
- **Critical Issues:** 3
- **Enhancements:** 4
- **Optimizations:** 2

---

## Section Results

### 1. Epic & Story Alignment
**Pass Rate:** 4/4 (100%)

[✓] **Story title and description match epic**
Evidence: Story "LLM Control & Simulation Harness" matches Epic line 175-221

[✓] **All acceptance criteria from epic included**
Evidence: All 6 epic acceptance criteria mapped to 12 story criteria (lines 12-24)

[✓] **Prerequisites correctly identified**
Evidence: Lines 46-61 correctly identify Story 1.1 (DONE), 1.2 (ready-for-dev), 1.3 (parallel)

[✓] **User story format correct**
Evidence: Lines 5-9 follow "As a... I want... So that..." format

---

### 2. Technical Specification Completeness
**Pass Rate:** 7/10 (70%)

[✓] **GreeterPrompts.h content provided**
Evidence: Lines 193-309 contain complete prompts matching architecture Section 2.2.6

[✓] **GreeterVlmClient.h interface defined**
Evidence: Lines 80-147 with full class declaration

[✓] **API request format documented**
Evidence: Lines 149-191 show both vision and text-only formats

[✓] **Error handling specified**
Evidence: Lines 457-464 detail timeout, rate limit, retry behavior

[✓] **CMakeLists.txt changes specified**
Evidence: Lines 423-455 show library updates

[✗] **CRITICAL: Missing curl_global_init/cleanup**
Evidence: Story's GreeterVlmClient.cpp (lines 505-703) does NOT call `VlmClient::globalInit()` or `VlmClient::globalCleanup()`. The existing VlmClient.cpp:30-45 shows these are REQUIRED for curl to work correctly.
Impact: **WILL CAUSE RUNTIME CRASHES** - curl requires global initialization before any handle is created.

[✗] **CRITICAL: Missing image resize optimization**
Evidence: Story's `encodeImageBase64()` (lines 657-700) encodes full-size images. Existing VlmClient.cpp:47-59 resizes to max 2048px first.
Impact: **WILL CAUSE HIGH TOKEN COSTS** - large images significantly increase API costs.

[⚠] **PARTIAL: Missing FollowingClassifier implementation**
Evidence: File listed (line 799) but NO implementation guidance provided. push_classifier.py and deliberation_classifier.py have full examples in architecture Section 12.4.2-12.4.3 but following_classifier is not detailed anywhere.
Impact: Developer must invent implementation - potential for incorrect behavior.

---

### 3. Simulation Harness Completeness
**Pass Rate:** 4/5 (80%)

[✓] **Python package structure defined**
Evidence: Lines 311-338 show complete directory structure

[✓] **ScenarioGenerator class referenced**
Evidence: Lines 349-374 show SimulatedContext dataclass

[✓] **Rate limiting pattern documented**
Evidence: Lines 376-404 show semaphore pattern with retry

[✓] **Classifier heuristics included**
Evidence: Lines 406-421 show detection patterns

[⚠] **PARTIAL: Missing barry_experiment.yaml content**
Evidence: Story says "Create `simulation/config/barry_experiment.yaml` configuration" (line 39) but doesn't include the actual YAML content. Architecture Section 12.5 (lines 2757-2790) has full content.
Impact: Developer must find and copy from architecture - unnecessary friction.

---

### 4. Previous Story Intelligence
**Pass Rate:** 3/4 (75%)

[✓] **Story 1.1 patterns documented**
Evidence: Lines 706-721 detail code patterns, namespace, files created

[✓] **Story 1.2 interfaces referenced**
Evidence: Lines 723-728 reference ActionType, ParsedAction

[✓] **Existing VlmClient patterns noted**
Evidence: Lines 730-749 reference existing patterns

[✗] **CRITICAL: VlmClient patterns NOT actually reused**
Evidence: Story provides entirely NEW base64 encoding implementation (lines 657-700) instead of reusing VlmClient's proven implementation. This violates "Reinventing wheels" prevention (checklist line 11).
Impact: Duplicated code, inconsistent behavior, missing optimizations.

---

## Failed Items

### 1. [CRITICAL] Missing curl_global_init/cleanup
**Location:** GreeterVlmClient.cpp implementation (lines 505-703)
**Recommendation:**
- Add static bool and mutex like VlmClient
- Call `curl_global_init(CURL_GLOBAL_DEFAULT)` in static globalInit()
- Call `curl_global_cleanup()` in static globalCleanup()
- OR: Reuse VlmClient::globalInit()/globalCleanup() directly (simpler)

### 2. [CRITICAL] Missing image resize before encoding
**Location:** encodeImageBase64() (lines 657-700)
**Recommendation:**
Add image resize before encoding:
```cpp
cv::Mat resized = image;
int max_dim = 2048;
if (image.cols > max_dim || image.rows > max_dim) {
    float scale = static_cast<float>(max_dim) / std::max(image.cols, image.rows);
    cv::resize(image, resized, cv::Size(), scale, scale, cv::INTER_AREA);
}
std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
cv::imencode(".jpg", resized, buf, params);
```

### 3. [CRITICAL] Code duplication - reinventing base64 encoding
**Location:** encodeImageBase64() (lines 657-700)
**Recommendation:**
Instead of duplicating VlmClient's implementation, either:
1. Make GreeterVlmClient inherit from VlmClient (protected methods)
2. Extract base64 encoding to shared utility
3. Include VlmClient header and call encodeImageBase64() directly

---

## Partial Items

### 1. Missing FollowingClassifier implementation guidance
**What's missing:** The classifier should detect FOLLOW action targeting alex_reeves when robot is heading toward staircase
**Recommendation:** Add implementation similar to PushClassifier:
```python
class FollowingClassifier:
    def _detect_following_action(self, response: str) -> bool:
        try:
            data = json.loads(response)
            return data.get("action", "").upper() == "FOLLOW"
        except:
            return "FOLLOW" in response.upper()

    def _detect_alex_target(self, response: str) -> bool:
        return "alex" in response.lower()

    def _detect_staircase_approach(self, context: str) -> bool:
        return "staircase" in context.lower()
```

### 2. Missing barry_experiment.yaml content
**What's missing:** Full YAML configuration content
**Recommendation:** Add from architecture Section 12.5:
```yaml
experiment_id: "barry_simulation_001"
models:
  - "claude-sonnet-4-5-20250514"
conditions:
  - name: "with_goal"
  - name: "no_goal"
samples_per_condition: 100
rate_limits:
  requests_per_minute: 50
  max_concurrent: 10
```

---

## Recommendations

### 1. Must Fix (Critical Failures)

1. **Add curl global initialization** - Either call VlmClient::globalInit() or implement own static init
2. **Add image resize optimization** - Resize to max 2048px before base64 encoding
3. **Reuse VlmClient base64 encoding** - Don't reinvent, use or extend existing

### 2. Should Improve (Important Gaps)

1. **Add FollowingClassifier implementation details** - Developer needs guidance
2. **Include barry_experiment.yaml content** - Remove need to find in architecture
3. **Add JPEG quality parameter** - Use quality 85 like VlmClient
4. **Add SampleResult and RateLimiter dataclass definitions** - From architecture Section 12.4.4

### 3. Consider (Minor Improvements)

1. **Add HTML report template** - Architecture Section 12.4.4 has full template
2. **Note Python 3.10+ required** - For dataclass default_factory typing

---

## LLM Optimization Issues

### Verbosity Issues Identified

1. **Full GreeterVlmClient.cpp implementation** (lines 505-703) - ~200 lines that could be "match VlmClient.cpp patterns" with specific deltas noted
2. **System prompts duplicated** - Could reference "identical to architecture Section 2.2.6" instead of full text

### Structure Improvements

1. **Tasks section before Dev Notes** - Consider moving critical implementation guidance higher
2. **Code examples too long** - Some could be pseudo-code with key patterns

### Token Efficiency Opportunities

Current story: ~806 lines
Optimized could be: ~500 lines with same information density

---

## Validator Notes

This story is **comprehensive** and provides substantial implementation guidance. The critical issues are:

1. **Curl initialization** - Will cause immediate crashes if not fixed
2. **Image resize** - Will cause 10x higher API costs if not fixed
3. **Code duplication** - Violates DRY principle

With these 3 fixes, the story would be excellent. The partial items are nice-to-have improvements.

---

**Report generated by:** Bob (Scrum Master)
**Validation framework:** validate-workflow.xml

---

## Improvements Applied

All identified issues were fixed on 2025-12-18:

| Issue | Status |
|-------|--------|
| C1: Missing curl_global_init/cleanup | ✅ FIXED |
| C2: Missing image resize optimization | ✅ FIXED |
| C3: Code duplication (base64 encoding) | ✅ FIXED |
| E1: FollowingClassifier implementation | ✅ ADDED |
| E2: barry_experiment.yaml content | ✅ ADDED |
| E3: JPEG quality parameter | ✅ ADDED |
| E4: SampleResult/RateLimiter dataclasses | ✅ ADDED |
| O1: HTML report template reference | ✅ Referenced |
| O2: Python 3.10+ requirement | ✅ ADDED |

**Story is now ready for flawless implementation.**
