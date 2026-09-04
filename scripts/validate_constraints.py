#!/usr/bin/env python3
"""Backward-compatibility forwarder -> scripts/ci/validate_constraints.py"""
import os
import sys

target = os.path.join(os.path.dirname(__file__), "ci", "validate_constraints.py")
os.execv(sys.executable, [sys.executable, target] + sys.argv[1:])
