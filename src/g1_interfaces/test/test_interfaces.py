# Copyright 2024 G1 Inspector Project
# Licensed under Apache-2.0

"""
Test suite for g1_interfaces package.

Tests that all custom messages, services, and actions are properly defined.
"""

import pytest


class TestMessages:
    """Test cases for custom message definitions."""

    def test_inspection_status_import(self):
        """Test that InspectionStatus message can be imported."""
        from g1_interfaces.msg import InspectionStatus
        msg = InspectionStatus()
        assert hasattr(msg, 'state')
        assert hasattr(msg, 'current_waypoint')
        assert hasattr(msg, 'battery_percentage')
        assert hasattr(msg, 'status_message')

    def test_defect_report_import(self):
        """Test that DefectReport message can be imported."""
        from g1_interfaces.msg import DefectReport
        msg = DefectReport()
        assert hasattr(msg, 'defect_id')
        assert hasattr(msg, 'defect_type')
        assert hasattr(msg, 'confidence')
        assert hasattr(msg, 'image_path')

    def test_notification_import(self):
        """Test that Notification message can be imported."""
        from g1_interfaces.msg import Notification
        msg = Notification()
        assert hasattr(msg, 'notification_type')
        assert hasattr(msg, 'severity')
        assert hasattr(msg, 'message')

    def test_inspection_status_constants(self):
        """Test that InspectionStatus has state constants."""
        from g1_interfaces.msg import InspectionStatus
        assert hasattr(InspectionStatus, 'STATE_IDLE')
        assert hasattr(InspectionStatus, 'STATE_INSPECTING')
        assert InspectionStatus.STATE_IDLE == 0

    def test_defect_report_constants(self):
        """Test that DefectReport has type constants."""
        from g1_interfaces.msg import DefectReport
        assert hasattr(DefectReport, 'TYPE_LOCATION_ERROR')
        assert hasattr(DefectReport, 'TYPE_QUALITY_ISSUE')

    def test_notification_constants(self):
        """Test that Notification has type and severity constants."""
        from g1_interfaces.msg import Notification
        assert hasattr(Notification, 'TYPE_ROUTE_BLOCKED')
        assert hasattr(Notification, 'SEVERITY_INFO')


class TestServices:
    """Test cases for custom service definitions."""

    def test_start_inspection_import(self):
        """Test that StartInspection service can be imported."""
        from g1_interfaces.srv import StartInspection
        assert StartInspection.Request is not None
        assert StartInspection.Response is not None

    def test_pause_inspection_import(self):
        """Test that PauseInspection service can be imported."""
        from g1_interfaces.srv import PauseInspection
        assert PauseInspection.Request is not None
        assert PauseInspection.Response is not None

    def test_get_status_import(self):
        """Test that GetStatus service can be imported."""
        from g1_interfaces.srv import GetStatus
        assert GetStatus.Request is not None
        assert GetStatus.Response is not None


class TestActions:
    """Test cases for custom action definitions."""

    def test_execute_inspection_import(self):
        """Test that ExecuteInspection action can be imported."""
        from g1_interfaces.action import ExecuteInspection
        assert ExecuteInspection.Goal is not None
        assert ExecuteInspection.Result is not None
        assert ExecuteInspection.Feedback is not None
