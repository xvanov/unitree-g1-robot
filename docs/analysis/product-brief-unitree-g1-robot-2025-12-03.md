---
stepsCompleted: [1, 2, 3, 4, 5]
inputDocuments: []
workflowType: 'product-brief'
lastStep: 5
project_name: 'unitree-g1-robot'
user_name: 'BMAD'
date: '2025-12-03'
---

# Product Brief: unitree-g1-robot

**Date:** 2025-12-03
**Author:** BMAD

---

## Executive Summary

Construction site inspection is a daily necessity that remains stubbornly manual. Superintendents and GCs walk job sites with plans in hand, visually checking progress, noting defects, and trying to maintain accurate understanding of project status. This process is slow, subjective, and error-prone - and when progress is misunderstood, money is lost.

**unitree-g1-robot** is an autonomous construction inspector that ingests 2D plans (plumbing, HVAC, electrical, finishes), navigates active job sites on legs, builds a digital representation of current conditions, and compares reality to plans. It delivers daily progress reports showing what's done, what changed today, and what remains - while physically marking issues with blue tape for crew follow-up.

---

## Core Vision

### Problem Statement

General contractors and superintendents need continuous, accurate visibility into construction progress across all trades. Currently, this requires a person to physically walk the site, manually compare conditions to plans, and subjectively assess completion status - a process that is time-consuming, inconsistent, and doesn't scale.

### Problem Impact

When progress is misunderstood:
- Subcontractor billing disputes arise (claiming 60% complete when it's actually 40%)
- Schedule slippage goes undetected until it's critical
- Defects and code violations get buried by subsequent work
- Daily crew planning is based on incomplete information
- Money is lost through rework, delays, and misallocated resources

### Why Existing Solutions Fall Short

Current inspection is purely human - a superintendent walking around with printed or tablet-based plans, relying on memory, experience, and manual note-taking. This approach:
- Can't scale across large or multiple sites
- Produces subjective, inconsistent assessments
- Creates no persistent spatial record for comparison over time
- Requires expensive human time for repetitive patrol work

### Proposed Solution

An autonomous legged robot that:
1. Ingests 2D construction plans across all trades
2. Self-localizes using the plans and navigates autonomously - including stairs and uneven terrain
3. Captures site conditions and builds a computer representation of current state
4. Compares current state to plans to assess progress and detect issues
5. Generates daily progress reports for GC/PM decision-making
6. Physically marks defects and issues with blue tape for crew action

### Key Differentiators

- **Legs, not wheels** - Can navigate the chaotic reality of active construction: stairs, debris, rebar, mud, temporary structures
- **Plan-based autonomy** - Self-localizes from the construction documents themselves, no special infrastructure required
- **Physical marking** - Doesn't just report issues digitally, but places blue tape on-site so crews know exactly where to act
- **Trade-agnostic** - Works across plumbing, HVAC, electrical, finishes - unified inspection across all disciplines
- **Daily cadence** - Designed for the rhythm of construction: what changed today, what's ready for tomorrow

---

## Target Users

### Primary Users

**Mike - The Multi-Site General Contractor**

Mike runs a mid-sized GC firm handling 3-4 active projects simultaneously - a restaurant build-out downtown, a dental office renovation in the suburbs, and two retail spaces in a new strip mall. He physically cannot visit every site daily, yet each project needs his oversight for quality control, progress verification, and subcontractor management.

**His Current Reality:**
- Mornings start with a flood of texts and photos from superintendents - inconsistent quality, missing context
- He's constantly deciding "which fire do I need to put out in person today?"
- Subcontractor disputes often come down to "he said, she said" without objective documentation
- By the time he discovers issues, subsequent work has often buried them

**What He Needs:**
- Reliable, consistent site documentation he can trust without being there
- Clear visibility into what changed today across all his projects
- Objective evidence for subcontractor conversations and billing verification
- A rich dataset for inspections, compliance, and contract disputes

**His "Aha!" Moment:**
"I reviewed all four sites over coffee this morning and knew exactly which sub to call before I even got in my truck."

---

**Carlos - The On-Site Superintendent**

Carlos is the boots-on-the-ground leader at the restaurant build-out. He manages daily crew coordination, but documentation and photo-taking eat into time he'd rather spend solving problems and keeping work moving.

**His Current Reality:**
- Spends 30-60 minutes daily walking the site taking photos for Mike
- Photos are rushed, sometimes miss key areas, and lack systematic coverage
- Has to answer constant "can you send me a picture of..." requests
- Blue tape placement is manual and sometimes forgotten or unclear

**What He Needs:**
- Automated inspection that captures everything systematically
- Easy robot deployment - set it up and let it run
- More time to actually manage the job instead of documenting it
- Clear punch list items physically marked for crews

**His "Aha!" Moment:**
"I set up the robot before the morning meeting, and by lunch the whole team had the inspection report with issues already blue-taped."

---

### Secondary Users

**Project Owner (Dana)**
- Wants daily progress visibility without bothering the GC
- Uses shared dashboard to see photos, walk routes, and completion status
- Makes financial decisions based on verified progress (draw requests, payments)

**Subcontractors**
- Receive faster feedback on work quality through GC relay
- Can see specific blue-tape locations and what needs fixing
- Benefit from objective documentation that protects them in disputes too

**Building Inspectors**
- Can review images remotely before scheduling site visits
- Get systematic coverage they can trust for preliminary assessments
- Potential to reduce unnecessary site visits for minor verifications

---

### User Journey

**Deployment (Day 1):**
Superintendent Carlos unboxes the robot, uploads the project plans, and walks it through an initial calibration. Robot learns the site layout.

**Daily Operation:**
Carlos sets the robot to run its inspection route each morning. By mid-morning, Mike reviews the dashboard over coffee - photos, progress overlay on plans, blue tape locations, change-from-yesterday highlights.

**Decision & Action:**
Mike identifies three issues: a plumbing rough-in in the wrong location, incomplete electrical in the kitchen, and a finish quality concern. He calls the relevant subs with specific photo evidence and blue-tape locations.

**Downstream Value:**
When the building inspector requests pre-inspection documentation, Mike exports a comprehensive visual history. When a sub disputes a back-charge, Mike has timestamped evidence of the defect.

---

## Success Metrics

### Core Inspection Accuracy

| Metric | Target | Description |
|--------|--------|-------------|
| **Defect Detection Rate** | ≥90% | Percentage of actual issues correctly identified |
| **False Negative Rate** | ≤5% | Percentage of real issues missed (critical failure metric) |
| **False Positive Rate** | ≤10% | Percentage of flagged items that aren't actual issues |

**MVP Detection Priority:**
1. **Location errors** - Work installed in wrong location vs. plans
2. **Quality issues** - Visible defects (scratches, damage, poor workmanship)

*Note: The system generates a punch list from each walkthrough. GC can remove false positives and add any missed items - the robot augments human judgment, doesn't replace it.*

---

### Operational Reliability

| Metric | Target | Description |
|--------|--------|-------------|
| **Route Completion Rate** | ≥95% | Percentage of planned inspection routes completed without human intervention |
| **Navigation Success Rate** | ≥95% | Successful traversal of stairs, obstacles, uneven terrain |
| **Robot Uptime** | ≥90% | Percentage of scheduled inspection time robot is operational |

*These are the primary metrics - if the robot can't reliably complete its route, nothing else matters.*

---

### Output Quality

| Metric | Target | Description |
|--------|--------|-------------|
| **Image Clarity** | Sufficient for defect identification | Captures detailed enough to see scratches, misalignments, quality issues |
| **Localization Accuracy** | ±6 inches | Blue tape placed within 6 inches of actual issue location |
| **Plan Correlation** | ≥90% | Accuracy of mapping captured conditions to correct plan locations |

---

### Business Objectives

| Objective | Success Indicator |
|-----------|-------------------|
| **Functional Construction Inspector** | Robot successfully identifies intentionally placed test defects |
| **Actionable Output** | GC can make crew decisions directly from daily report |
| **Time Recovery** | Superintendent documentation time reduced significantly |
| **Evidence Quality** | Captures sufficient for inspector review and dispute resolution |

---

### Key Performance Indicators

**Primary KPIs (MVP Validation):**
1. **Test Defect Detection** - Robot identifies 90%+ of intentionally placed defects (wrong locations, quality issues like tile scratches)
2. **Route Completion** - Robot completes 95%+ of assigned inspection routes autonomously
3. **Decision Enablement** - GC can identify actionable punch list items from report without site visit

**Validation Test Scenario:**
- Place items in wrong locations per plan
- Introduce visible quality defects (e.g., scratched tile)
- Robot should detect and blue-tape both categories at ≥90% rate

---

## MVP Scope

### Core Features

**1. Plan Ingestion**
- Accept 2D construction plans in PDF or PNG format
- Focus on finishes trade for MVP
- Parse plan to enable self-localization and comparison

**2. Autonomous Indoor Navigation**
- Self-orient from plans (user can provide rough starting location)
- Navigate autonomously through indoor construction site
- Traverse stairs and uneven terrain
- Complete planned inspection route without human intervention

**3. Visual Capture & Analysis**
- Capture images of current site conditions
- Build computer representation of space
- Detect major quality issues on finishes (scratches, visible damage)
- Identify location errors vs. plans

**4. Physical Issue Marking**
- Carry blue tape roll
- Navigate to detected issue location
- Tear and place tape to mark issues for crew follow-up
- Localization accuracy: ±6 inches

**5. Inspection Report**
- Photos of issues with locations marked on plan
- Punch list with photo evidence
- Walk route visualization
- Simple output format (PDF/email sufficient)

---

### Out of Scope for MVP

| Feature | Rationale |
|---------|-----------|
| **BIM integration** | Adds complexity; 2D plans sufficient for validation |
| **Outdoor areas** | Focus on controlled indoor environment first |
| **Multiple trades** | Finishes only; plumbing/HVAC/electrical deferred |
| **Progress tracking over time** | Day-over-day comparison is V2 feature |
| **Real-time alerts** | Batch reporting sufficient for MVP |
| **Full dashboard** | Simple report validates core value |
| **Multi-site support** | Single site proves the concept |
| **Minor defects** | Major quality issues only (no hairline cracks) |

---

### MVP Success Criteria

| Criteria | Validation |
|----------|------------|
| **Defect Detection** | Robot identifies ≥90% of intentionally placed test defects (wrong locations, scratched tiles) |
| **Route Completion** | Robot completes ≥95% of inspection routes autonomously including stairs |
| **Blue Tape Accuracy** | Tape placed within 6 inches of actual issue |
| **Self-Localization** | Robot orients from plans with only rough starting location |
| **Actionable Report** | GC can create crew punch list directly from report output |
| **End-to-End Demo** | Complete inspection cycle: plan upload → autonomous walk → detection → tape placement → report generation |

---

### Future Vision

**Post-MVP Enhancements:**

*Near-term (V2):*
- Multi-trade support (plumbing, HVAC, electrical rough-ins)
- Day-over-day progress tracking and comparison
- Full dashboard with multi-project views
- Outdoor site area navigation

*Medium-term:*
- BIM model integration for richer comparison
- Real-time alerts for critical issues
- Multi-site fleet management
- Subcontractor access and workflows

*Long-term Vision:*
- Predictive issue detection (AI learns common defect patterns)
- Integration with project management tools (Procore, PlanGrid)
- Automated inspector pre-check documentation
- Industry-specific variants (residential, commercial, industrial)

---

<!-- Content will be appended sequentially through collaborative workflow steps -->
