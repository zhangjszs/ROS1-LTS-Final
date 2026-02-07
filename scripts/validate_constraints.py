#!/usr/bin/env python3
"""
validate_constraints.py
验证 docs/constraints/*.yaml 是否符合 constraint_schema.yaml 格式。

用法:
    python3 scripts/validate_constraints.py
    python3 scripts/validate_constraints.py docs/constraints/acceleration.yaml
"""

import sys
import os
import yaml
from pathlib import Path

REQUIRED_FIELDS = [
    "constraint_id", "name", "name_en", "description",
    "value", "unit", "applicable_missions",
    "constraint_type", "violation_criteria",
    "source_ref", "algorithm_usage",
]

VALID_MISSIONS = {"acceleration", "skidpad", "autocross", "trackdrive", "all"}
VALID_TYPES = {"hard", "soft"}

def validate_constraint(entry, file_path):
    errors = []
    cid = entry.get("constraint_id", "<missing>")

    for field in REQUIRED_FIELDS:
        if field not in entry:
            errors.append(f"  [{cid}] 缺少必填字段: {field}")

    if "constraint_type" in entry and entry["constraint_type"] not in VALID_TYPES:
        errors.append(f"  [{cid}] constraint_type 无效: {entry['constraint_type']} (应为 hard/soft)")

    if "applicable_missions" in entry:
        for m in entry["applicable_missions"]:
            if m not in VALID_MISSIONS:
                errors.append(f"  [{cid}] 未知 mission: {m}")

    return errors

def validate_file(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if data is None:
        return [f"文件为空: {file_path}"]

    errors = []

    if "schema_version" not in data:
        errors.append("缺少 schema_version")

    constraints = data.get("constraints", [])
    if not constraints:
        # schema 文件本身只有 example，跳过
        if "example" in data:
            constraints = data["example"]
        else:
            errors.append("未找到 constraints 或 example 列表")
            return errors

    ids_seen = set()
    for entry in constraints:
        cid = entry.get("constraint_id", "")
        if cid in ids_seen:
            errors.append(f"  [{cid}] constraint_id 重复")
        ids_seen.add(cid)
        errors.extend(validate_constraint(entry, file_path))

    return errors

def main():
    if len(sys.argv) > 1:
        files = [Path(f) for f in sys.argv[1:]]
    else:
        constraints_dir = Path(__file__).resolve().parent.parent / "docs" / "constraints"
        files = sorted(constraints_dir.glob("*.yaml"))
        files = [f for f in files if f.name != "constraint_schema.yaml"]

    if not files:
        print("未找到约束文件")
        sys.exit(1)

    total_errors = 0
    total_constraints = 0

    for f in files:
        print(f"验证: {f.name}")
        errors = validate_file(f)
        if errors:
            for e in errors:
                print(f"  ✗ {e}")
            total_errors += len(errors)
        else:
            with open(f, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh)
            n = len(data.get("constraints", []))
            total_constraints += n
            print(f"  ✓ {n} 条约束，全部通过")

    print(f"\n总计: {total_constraints} 条约束, {total_errors} 个错误")
    sys.exit(1 if total_errors > 0 else 0)

if __name__ == "__main__":
    main()
