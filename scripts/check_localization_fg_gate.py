#!/usr/bin/env python3
"""Backward-compatibility forwarder -> scripts/ci/check_localization_fg_gate.py"""
import os
import sys

target = os.path.join(os.path.dirname(__file__), "ci", "check_localization_fg_gate.py")
os.execv(sys.executable, [sys.executable, target] + sys.argv[1:])
