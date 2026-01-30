# Git History Cleanup Guide

## WARNING: This Operation Rewrites Git History

**IMPORTANT:** This operation will permanently rewrite the Git history. All team members will need to re-clone the repository after this operation.

## What Will Be Removed

The following paths will be removed from Git history:
- `src/_deprecated/` (27MB of deprecated code)
- `src/ros_vehicle_interface/` (old version, already deleted)
- `src/ros_vehicle_racing_num/` (old version, already deleted)
- `src_backup_20260123_142141.tar.gz` (21MB backup file)

Total space to be reclaimed: ~48MB

## Prerequisites

```bash
# Install git-filter-repo
pip3 install git-filter-repo

# Ensure you have a backup
git clone --mirror https://github.com/zhangjszs/ROS1-LTS-Final.git backup-repo
```

## Cleanup Commands

```bash
# Navigate to repository
cd ~/2025huat

# Create a fresh clone for safety
cd ..
git clone https://github.com/zhangjszs/ROS1-LTS-Final.git 2025huat-clean
cd 2025huat-clean

# Remove deprecated directory from history
git filter-repo --path src/_deprecated --invert-paths --force

# Remove old vehicle interface from history
git filter-repo --path src/ros_vehicle_interface --invert-paths --force

# Remove old racing num from history
git filter-repo --path src/ros_vehicle_racing_num --invert-paths --force

# Remove backup tarball from history
git filter-repo --path src_backup_20260123_142141.tar.gz --invert-paths --force

# Verify the cleanup
du -sh .git
git log --all --oneline | head -20
```

## Push to Remote (DESTRUCTIVE)

```bash
# Force push all branches
git push origin --force --all

# Force push all tags
git push origin --force --tags
```

## Team Coordination

After force pushing, all team members must:

```bash
# Delete their local repository
cd ~
rm -rf 2025huat

# Clone fresh copy
git clone https://github.com/zhangjszs/ROS1-LTS-Final.git 2025huat
cd 2025huat
```

## Verification

After cleanup:
```bash
# Check repository size
du -sh .git

# Verify all packages still build
catkin build
catkin run_tests
```

## Rollback Plan

If something goes wrong:
```bash
# Restore from backup
cd ~
rm -rf 2025huat
git clone backup-repo 2025huat
cd 2025huat
git push origin --force --all
```

## Recommendation

**DO NOT** perform this cleanup if:
- Active development is ongoing
- Team members have unpushed changes
- You're unsure about the impact

**DO** perform this cleanup if:
- All team members are coordinated
- You have verified backups
- Repository size is causing issues
