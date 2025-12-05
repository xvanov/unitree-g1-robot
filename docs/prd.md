---
stepsCompleted: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
inputDocuments:
  - docs/analysis/product-brief-unitree-g1-robot-2025-12-03.md
workflowType: 'prd'
lastStep: 11
project_name: 'unitree-g1-robot'
user_name: 'BMAD'
date: '2025-12-03'
---

# Product Requirements Document - unitree-g1-robot

**Author:** BMAD
**Date:** 2025-12-03

## Executive Summary

unitree-g1-robot is an autonomous construction site inspector built on the Unitree G1 humanoid platform. It addresses a fundamental problem in construction management: the lack of continuous, objective visibility into project progress and quality.

General contractors managing multiple sites and superintendents buried in manual documentation need systematic, reliable inspection without the time cost. Current methods are subjective, inconsistent, and don't scale. When progress is misunderstood, the downstream costs are significant - billing disputes, undetected schedule slippage, defects buried by subsequent work, and expensive rework.

This robot ingests 2D construction plans, autonomously navigates active indoor job sites (including stairs and uneven terrain), captures and analyzes site conditions, detects defects and location errors, and physically marks issues with blue tape for crew follow-up. It delivers daily inspection reports that enable GCs to make informed decisions without being physically present.

### What Makes This Special

- **Legs over wheels** - Navigates the chaotic reality of active construction: stairs, debris, rebar, mud, temporary structures. Wheeled robots fail in these environments.
- **Plan-based autonomy** - Self-localizes from construction documents themselves. No beacons, no special infrastructure, no site prep required.
- **Physical action, not just digital reporting** - Places actual blue tape at issue locations so crews know exactly where to act without referencing a screen.
- **Daily operational cadence** - Designed for construction rhythm: what changed today, what's ready for tomorrow.

## Project Classification

**Technical Type:** IoT/Embedded Robotics
**Domain:** Scientific/AI (Computer Vision, Autonomous Navigation, Defect Detection)
**Complexity:** High (MVP-scoped)

This is a robotics and AI product combining autonomous navigation, computer vision for defect detection, real-time localization from 2D plans, and physical actuation for tape placement. The MVP constrains scope to finishes trade inspection in indoor environments, validating core capabilities before expanding to multi-trade and outdoor scenarios.

## Success Criteria

### User Success

**Mike (Multi-Site GC) - MVP Validation:**
- Can review site status without being physically present
- Receives actionable information: what's wrong and where
- Blue tape locations are accurate enough for crews to find issues

**Carlos (Superintendent) - MVP Validation:**
- Robot deployment is simple (set up and let it run)
- Systematic coverage without manual photo-taking
- Issues are physically marked for crew follow-up

**Success Moment:** "I set up the robot before the morning meeting, and by lunch the whole team had the inspection report with issues already blue-taped."

### Business Success

**MVP Validation Criteria (Internal):**
- Prove the core inspection loop works end-to-end
- Validate autonomous navigation in real construction environment
- Demonstrate defect detection capability against known test cases
- Confirm blue tape placement mechanism functions reliably

**Go/No-Go Decision Point:**
MVP success = robot can autonomously walk a site, localize itself, capture interior representation, identify setup issues vs. plans, and mark them with tape.

### Technical Success

| Metric | Target | Priority |
|--------|--------|----------|
| Defect Detection Rate | ≥90% | Critical |
| False Negative Rate | ≤5% | Critical |
| False Positive Rate | ≤10% | Important |
| Route Completion Rate | ≥95% | Critical |
| Navigation Success (stairs/terrain) | ≥95% | Critical |
| Blue Tape Placement Accuracy | ±6 inches | Important |
| Plan Correlation Accuracy | ≥90% | Important |
| Robot Uptime | ≥90% | Important |

### Measurable Outcomes

**Validation Test Protocol:**
1. Place items in wrong locations per plan (location errors)
2. Introduce visible quality defects (scratched tiles, damage)
3. Robot should detect and blue-tape both categories at ≥90% rate
4. Robot completes full inspection route autonomously including stairs

**MVP Success = All of the following:**
- [ ] Autonomous indoor navigation including stairs
- [ ] Self-localization from 2D plans (rough starting location only)
- [ ] Interior capture (photos/point cloud/representation)
- [ ] Defect identification at ≥90% detection rate
- [ ] Blue tape placement within 6 inches of issues
- [ ] End-to-end cycle completion without human intervention

## Product Scope

### MVP - Minimum Viable Product

**In Scope:**
- 2D plan ingestion (PDF/PNG) - finishes trade only
- Autonomous indoor navigation including stairs
- Self-localization from construction plans
- Visual capture and interior representation
- Defect detection: location errors and quality issues
- Blue tape marking mechanism
- Basic inspection report (PDF/email sufficient)
- Single site operation

**Out of Scope:**
- BIM integration
- Outdoor navigation
- Multi-trade inspection (plumbing, HVAC, electrical)
- Day-over-day progress tracking
- Real-time alerts
- Full dashboard
- Multi-site fleet management

### Growth Features (Post-MVP)

- Multi-trade support (plumbing, HVAC, electrical rough-ins)
- Day-over-day progress tracking and comparison
- Full dashboard with project views
- Outdoor site area navigation
- Subcontractor access and workflows

### Vision (Future)

- BIM model integration
- Real-time alerts for critical issues
- Multi-site fleet management
- Predictive issue detection (AI learns defect patterns)
- Integration with project management tools (Procore, PlanGrid)
- Automated inspector pre-check documentation

## User Journeys

### Journey 1: Carlos - From Manual Documentation to Morning Coffee

Carlos is the superintendent at a restaurant build-out downtown. Every morning for the past six months, he's spent 30-60 minutes walking the site with his phone, snapping photos of progress and problems. The photos are rushed - he knows he's missing things, but there's always a crew waiting for direction or a delivery truck pulling up. When Mike calls asking "can you send me a picture of the kitchen rough-in?" Carlos has to drop what he's doing, walk back across the site, and hope he remembers exactly which angle Mike needs.

One Monday morning, Carlos unboxes the robot and uploads the finish plans. He walks it through an initial calibration - showing it the starting point and letting it learn the layout. The next day, he sets the robot to run its inspection route before the 7am crew meeting. By 9am, while Carlos is actually solving problems with the tile crew, the robot has systematically covered every room, captured the current state, and placed blue tape on three issues: a light fixture installed 6 inches off from plan, a scratched countertop, and a missing outlet cover.

Carlos checks the report on his tablet during lunch. Instead of fielding five "send me a photo" requests from Mike, there's nothing - Mike already has everything he needs. When the afternoon inspector asks about the electrical rough-in documentation, Carlos pulls up the robot's captures instead of scrambling to remember what he photographed last week. He leaves the site at 5pm instead of 6pm, and for the first time in months, he's not thinking about what he forgot to document.

**Journey Reveals Requirements For:**
- Simple robot deployment (set up and run)
- Plan upload interface
- Initial calibration/site learning
- Autonomous route execution
- Blue tape placement mechanism
- Report generation with photos and locations
- Report access via tablet/mobile

---

### Journey 2: Mike - Multi-Site Visibility Over Morning Coffee

Mike runs a mid-sized GC firm with four active projects. His mornings used to start with anxiety - scrolling through a flood of texts and photos from superintendents, each with different quality and missing context. He'd spend the first hour just trying to figure out which site needed him in person today. Subcontractor disputes were constant headaches: "We finished 60% of the electrical" versus his superintendent's "Looks like 40% to me" - and no objective evidence either way.

The robot has been running at the restaurant build-out for two weeks. This Tuesday morning, Mike opens the dashboard at 6:30am over coffee. He sees the previous day's inspection: photos overlaid on the plan, blue tape locations marked, issues categorized. The plumbing rough-in that was supposed to be complete yesterday? The robot shows it's clearly 70% done, not 100% as the sub claimed. There's a scratch on a $400 tile that would have been covered by the cabinet installation today if Carlos hadn't seen the blue tape and caught it.

Mike makes three calls before 7am: one to the plumbing sub with specific photo evidence of what's incomplete, one to the tile supplier about the damage, and one to Carlos to say "looks good, no need for me to come by today." By 8am he's at the dental office project - the one that actually needs his attention. When the restaurant owner asks for a progress update, Mike exports a visual history showing exactly what changed this week.

**Journey Reveals Requirements For:**
- Daily inspection reports
- Photo overlay on plans
- Issue categorization and listing
- Visual evidence for subcontractor conversations
- Progress documentation over time (future)
- Export capability for stakeholder communication
- Remote access (web/mobile dashboard - future, PDF for MVP)

---

### Journey 3: Carlos - Robot Recovery (Edge Case)

It's Thursday morning, and Carlos sets up the robot as usual. Thirty minutes into its route, the robot encounters something unexpected - a stack of drywall sheets blocking its planned path through the hallway. The robot attempts an alternate route, but the temporary scaffolding in the secondary path is too narrow.

The robot stops, captures its current location and the obstacle, and sends Carlos a notification: "Route blocked - manual intervention required." Carlos walks over, moves two drywall sheets, and taps "Resume" on the robot's interface. The robot recalculates and continues its inspection, completing 94% of the planned route. The report notes the blocked section and when human intervention occurred.

The next morning, Carlos adjusts the robot's start time to run before the drywall delivery. The report now shows 100% route completion.

**Journey Reveals Requirements For:**
- Obstacle detection and handling
- Route recalculation capability
- Operator notification for stuck conditions
- Manual intervention interface (simple resume)
- Route completion percentage tracking
- Blocked section documentation
- Adjustable scheduling

---

### Journey Requirements Summary

| Capability Area | Requirements Revealed |
|-----------------|----------------------|
| **Robot Setup & Deployment** | Plan upload, initial calibration, site learning, simple operator interface |
| **Autonomous Navigation** | Route planning, obstacle detection, stair traversal, route recalculation, completion tracking |
| **Visual Capture** | Photo capture, interior representation (point cloud), plan correlation |
| **Defect Detection** | Location error detection, quality issue identification, plan comparison |
| **Physical Marking** | Blue tape carrying, tape placement within 6", navigation to issue locations |
| **Reporting** | Photo overlay on plans, issue categorization, punch list generation, PDF/email output |
| **Operator Interaction** | Notifications, manual intervention resume, route adjustment, scheduling |
| **Remote Access** | Report viewing (tablet/mobile), export capability, visual evidence retrieval |

## Innovation & Novel Patterns

### Detected Innovation Areas

**1. Plan-Based Self-Localization**
The robot localizes itself using 2D construction plans as the primary reference - no beacons, no pre-installed infrastructure, no site preparation required. This removes a major deployment barrier for construction environments where installing tracking infrastructure is impractical.

**2. Physical Issue Marking (Blue Tape Placement)**
Unlike purely digital inspection systems, this robot physically marks issues with blue tape on-site. This bridges the gap between digital detection and crew action - workers see the tape and know exactly where to look without consulting a device. This mechanism is being designed from scratch, leveraging the Unitree G1 EDU's manipulation capabilities.

**3. Legged Navigation for Active Construction**
Using a humanoid robot (Unitree G1 EDU) to navigate active construction sites including stairs, debris, and uneven terrain. While legged robots have been deployed in construction contexts, the combination of humanoid form factor with autonomous inspection and physical marking is novel.

### Hardware Platform

**Unitree G1 EDU Humanoid Robot** - Fixed hardware platform. All software and capabilities must be designed for this specific robot's sensors, actuators, and physical constraints.

### Validation Approach

Each innovative aspect requires targeted validation:

| Innovation | Validation Method | Success Criteria |
|------------|------------------|------------------|
| Plan-based localization | Test in construction environment with various plan types | Robot correctly identifies position from plans with rough starting point |
| Blue tape placement | Mechanism testing on various surfaces | Tape placed within 6 inches of target, adheres properly |
| Legged navigation | Route completion testing including stairs | 95% route completion without human intervention |

### Risk Mitigation

| Risk | Mitigation Strategy |
|------|---------------------|
| Localization failure | Robot stops and sends notification; operator provides manual guidance |
| Tape mechanism failure | Robot logs issue location; report includes un-taped items for manual follow-up |
| Navigation obstacle | Attempt alternate route; if blocked, stop and notify operator |

## IoT/Embedded Robotics Requirements

### Hardware Platform

**Unitree G1 EDU (23 DOF Version)**

| Specification | Details |
|---------------|---------|
| Height | 1320mm standing (folds to 690mm) |
| Weight | ~35kg with battery |
| Degrees of Freedom | 23 total (6 per leg, 5 per arm, 1 waist) |
| Arm Payload | ~2kg per arm |
| Compute - CPU | 8-core high-performance processor |
| Compute - AI | NVIDIA Jetson Orin NX 16GB (100 TOPS) |
| Battery | 9000mAh lithium, ~2 hours runtime |
| Connectivity | WiFi 6, Bluetooth 5.2 |

### Sensor Suite

| Sensor | Specifications | Use Case |
|--------|---------------|----------|
| Intel RealSense D435i | Depth camera, front-facing | Visual capture, defect detection, obstacle avoidance |
| LIVOX MID-360 LiDAR | 360° horizontal, 59° vertical FOV | Localization, navigation, interior mapping |
| Joint Encoders | Dual encoder per joint | Proprioception, position feedback |
| 4-Microphone Array | Audio input | Future: voice commands (not MVP) |

### Compute Architecture

**Hardware Capabilities:**
- **Onboard (Jetson Orin NX 16GB):** Capable of real-time navigation, obstacle avoidance, sensor fusion, perception
- **Offload Option (via WiFi):** Heavy ML inference (defect detection models), plan parsing, report generation

**MVP Deployment Model (2025-12-04 Decision):**
All processing runs on an off-board development machine connected via Ethernet. Robot runs stock Unitree firmware only — no custom software installed on the robot. This simplifies development and avoids conflicts on shared robot hardware.

See `docs/architecture.md` for deployment details. Docker on Jetson remains a future option if untethered operation is needed.

### Software Development

**SDK & Frameworks:**
- Unitree SDK2 (Python and C++ APIs)
- ROS2 integration via CycloneDDS
- Direct access to motor commands, joint control, sensor streams

**Available Interfaces:**
- Camera feeds (RGB + depth)
- LiDAR point clouds
- Joint position/velocity/torque control
- Motor commands

**Updates:** OTA firmware updates supported by Unitree platform

### Physical Marking Strategy

**MVP Approach:** Blue tape placement is deprioritized. Initial focus on detection and reporting issue locations in the inspection report.

**Future Enhancement:** Static tape dispenser attachment to arm end effector (no dexterous hand available on 23 DOF model). Design TBD after core detection capabilities validated.

**Fallback:** Report-only mode - all detected issues documented with coordinates and photos, no physical marking.

### Implementation Considerations

**Navigation:**
- Leverage LiDAR for SLAM and localization against 2D plans
- D435i depth camera for obstacle detection and avoidance
- Stair climbing uses built-in G1 locomotion capabilities

**Perception Pipeline:**
- Capture RGB images and point clouds during route
- Build interior representation for plan comparison
- Run defect detection (location errors, quality issues)

**Communication:**
- Always-online via WiFi
- Real-time telemetry to monitoring station
- Operator always watching during MVP validation

**Safety:**
- Built-in G1 safety features: emergency stop, torque limiting, collision detection
- Operator supervision required during MVP phase

## Project Scoping & Phased Development

### MVP Strategy & Philosophy

**MVP Approach:** Problem-Solving MVP - validate the core technical loop before adding features

**Validation Goal:** Prove the robot can autonomously navigate, localize, detect issues, and generate useful reports. Internal validation only, single site, operator-supervised.

**Resource Model:** Hybrid compute (onboard + offloaded), always-online WiFi, operator always watching during MVP phase.

### MVP Feature Set (Phase 1)

**Core User Journeys Supported:**
- Carlos: Deploy robot, run inspection, receive report
- Mike: Review inspection report, identify actionable issues

**Must-Have Capabilities:**

| Capability | MVP Implementation | Success Criteria |
|------------|-------------------|------------------|
| Plan Ingestion | PDF/PNG upload, finishes trade only | Plans parsed and usable for localization |
| Autonomous Navigation | Indoor routes including stairs | ≥95% route completion |
| Self-Localization | Plan-based with rough starting point | Robot knows where it is throughout route |
| Visual Capture | RGB images + point cloud | Interior representation built |
| Defect Detection | Location errors + quality issues | ≥90% detection rate |
| Inspection Report | PDF output with photos and issue locations | Actionable punch list generated |

**Explicitly Deferred from MVP:**

| Feature | Rationale | Target Phase |
|---------|-----------|--------------|
| Blue tape placement | Focus on detection first; report-only acceptable | Phase 2 |
| Multi-trade support | Validate with finishes before expanding | Phase 2 |
| Progress tracking | Day-over-day comparison adds complexity | Phase 2 |
| Dashboard | PDF reports sufficient for validation | Phase 2 |
| Multi-site support | Single site proves concept | Phase 3 |
| Outdoor navigation | Controlled indoor environment first | Phase 3 |

### Post-MVP Features

**Phase 2 (Growth):**
- Blue tape placement mechanism (if validated need)
- Multi-trade inspection (plumbing, HVAC, electrical)
- Day-over-day progress tracking and comparison
- Web dashboard for report access
- Subcontractor report sharing

**Phase 3 (Expansion):**
- Outdoor site navigation
- Multi-site fleet management
- BIM integration
- Real-time alerts
- Integration with project management tools (Procore, PlanGrid)
- Predictive defect detection

### Risk Mitigation Strategy

**Technical Risks:**

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Plan-based localization fails | Medium | Critical | Validate early; fallback to manual operator guidance |
| Defect detection accuracy insufficient | Medium | High | Start with obvious defects; iterative model improvement |
| Navigation fails on construction debris | Medium | Medium | Operator supervision; conservative route planning |
| 2hr battery insufficient for large sites | Low | Medium | Plan routes within battery budget; staged inspections |

**Validation Sequence (Risk-Ordered):**
1. **First:** Self-localization from plans (highest risk)
2. **Second:** Autonomous navigation including stairs
3. **Third:** Defect detection accuracy
4. **Fourth:** End-to-end inspection cycle
5. **Last:** Blue tape placement (if pursued)

**Contingency:** If localization proves unreliable, fallback to operator-guided waypoints rather than full autonomy.

## Functional Requirements

### Plan Management

- FR1: Operator can upload 2D construction plans in PDF format
- FR2: Operator can upload 2D construction plans in PNG format
- FR3: System can parse uploaded plans to extract spatial layout information
- FR4: Operator can specify the trade type for uploaded plans (finishes for MVP)
- FR5: Operator can provide a rough starting location on the plan for robot initialization

### Robot Deployment & Control

- FR6: Operator can initiate robot calibration at a known starting position
- FR7: Operator can start an inspection route
- FR8: Operator can pause an active inspection
- FR9: Operator can resume a paused inspection
- FR10: Operator can abort an inspection and return robot to standby
- FR11: System can display real-time robot status (location, battery, route progress)

### Autonomous Navigation

- FR12: Robot can navigate autonomously through indoor construction environments
- FR13: Robot can traverse stairs during inspection routes
- FR14: Robot can navigate around static obstacles (construction materials, equipment)
- FR15: Robot can detect when a planned path is blocked
- FR16: Robot can attempt alternate routes when primary path is blocked
- FR17: Robot can stop safely and notify operator when no viable path exists
- FR18: System can track route completion percentage

### Localization

- FR19: Robot can self-localize using uploaded 2D plans and onboard sensors
- FR20: Robot can maintain position awareness throughout inspection route
- FR21: Robot can detect when localization confidence is low
- FR22: Robot can stop and notify operator when localization fails

### Visual Capture & Mapping

- FR23: Robot can capture RGB images during inspection
- FR24: Robot can capture depth data during inspection
- FR25: Robot can capture LiDAR point cloud data during inspection
- FR26: System can build an interior representation from captured sensor data
- FR27: System can correlate captured data to locations on the construction plan

### Defect Detection

- FR28: System can compare captured interior state against reference plans
- FR29: System can detect location errors (items installed in wrong position vs. plan)
- FR30: System can detect visible quality issues (scratches, damage, defects on finishes)
- FR31: System can identify the location of each detected issue on the plan
- FR32: System can capture photo evidence for each detected issue
- FR33: System can assign confidence scores to detected issues

### Inspection Reporting

- FR34: System can generate inspection report after route completion
- FR35: System can include photos of detected issues in report
- FR36: System can show issue locations overlaid on the construction plan
- FR37: System can generate a punch list of detected issues
- FR38: System can categorize issues by type (location error, quality issue)
- FR39: System can export report as PDF
- FR40: Operator can access generated reports

### Operator Notifications

- FR41: System can notify operator when robot requires intervention
- FR42: System can notify operator when inspection route is complete
- FR43: System can notify operator when localization fails
- FR44: System can notify operator when path is blocked and cannot be resolved

### Physical Marking (Phase 2 - Deferred)

- FR45: Robot can carry blue tape dispenser (deferred)
- FR46: Robot can navigate to detected issue location (deferred)
- FR47: Robot can place blue tape at issue location within 6 inches accuracy (deferred)

## Non-Functional Requirements

### Performance

| Requirement | Specification | Rationale |
|-------------|---------------|-----------|
| NFR1: Navigation Response | Obstacle detection and response within 500ms | Safe navigation requires quick reaction |
| NFR2: Localization Update | Position estimate updated at ≥10Hz | Smooth navigation requires continuous localization |
| NFR3: Image Capture Rate | ≥1 frame per second during inspection | Sufficient coverage without overwhelming storage |
| NFR4: Defect Detection | Processing completes before robot moves to next area | No missed areas due to processing lag |
| NFR5: Report Generation | Report available within 30 minutes of route completion | Timely enough for same-day use |

### Reliability

| Requirement | Specification | Rationale |
|-------------|---------------|-----------|
| NFR6: Route Completion | ≥95% of inspection routes complete without human intervention | Core MVP success metric |
| NFR7: Navigation Success | ≥95% successful traversal of stairs and obstacles | Operational reliability |
| NFR8: Localization Stability | Localization maintained for full route duration | Can't inspect if position is lost |
| NFR9: Graceful Degradation | Robot stops safely on any unrecoverable error | Never leaves robot in unsafe state |
| NFR10: Data Persistence | Captured data persists through robot restart | No lost inspection data |

### Safety

| Requirement | Specification | Rationale |
|-------------|---------------|-----------|
| NFR11: Emergency Stop | Robot stops all motion within 500ms of e-stop activation | Operator safety control |
| NFR12: Collision Avoidance | Robot detects and avoids obstacles ≥0.5m ahead | Prevent damage to robot and environment |
| NFR13: Fall Prevention | Robot maintains stability on uneven surfaces and stairs | Prevent robot damage |
| NFR14: Battery Safety | Robot returns to safe state when battery ≤10% | Prevent stranding mid-route |
| NFR15: Operator Supervision | System assumes operator is watching during MVP | Safety backstop for edge cases |

### Hardware Constraints

| Requirement | Specification | Rationale |
|-------------|---------------|-----------|
| NFR16: Battery Life | Full inspection route completes within 2-hour battery window | Hardware limitation |
| NFR17: Compute Budget | Real-time processing fits within Jetson Orin capacity | Onboard compute constraint |
| NFR18: WiFi Connectivity | Maintains connection for telemetry and compute offload | Always-online architecture |
| NFR19: Sensor Availability | All sensors (LiDAR, D435i, encoders) operational during inspection | Required for navigation and capture |

### Data & Storage

| Requirement | Specification | Rationale |
|-------------|---------------|-----------|
| NFR20: Capture Storage | Sufficient storage for full route imagery + point cloud | Complete inspection data |
| NFR21: Plan Storage | Plans persist across inspection sessions | No re-upload required |
| NFR22: Report Retention | Reports retained for duration of project | Historical access for disputes/reference |

