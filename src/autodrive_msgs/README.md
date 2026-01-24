# autodrive_msgs

Unified HUAT message definitions for the autodrive stack.

## Naming Rules (v1)

- All message names start with `HUAT_`.
- PascalCase for the suffix (`HUAT_CarState`, `HUAT_VehicleCmd`, ...).
- Legacy names are removed from the package.

## Migration Record

This package was refactored from `common_msgs` and normalized into `HUAT_*` names.

### Mapping (legacy -> HUAT)

| Legacy message | HUAT message |
| --- | --- |
| Cone | HUAT_ConeDetections |
| HUAT_cone | HUAT_Cone |
| HUAT_map | HUAT_ConeMap |
| HUAT_Tracklimits | HUAT_TrackLimits |
| HUAT_PathLimits | HUAT_PathLimits |
| HUAT_ASENSING | HUAT_Asensing |
| new_ASENSING | HUAT_AsensingStamped |
| HUAT_Carstate | HUAT_CarState |
| HUAT_ControlCommand | HUAT_ControlCommand |
| HUAT_HighSpeedViz | HUAT_HighSpeedViz |
| HUAT_VehcileCmd / vehicle_cmd | HUAT_VehicleCmd |
| vehicle_status | HUAT_VehicleStatus |
| HUAT_stop | HUAT_Stop |
| ImageObjs | HUAT_ImageObjs |
| ImageRect | HUAT_ImageRect |
| fsd_cone | HUAT_FsdCone |
| fsd_map | HUAT_FsdMap |
| ins_p2 | HUAT_InsP2 |
| lane_point | HUAT_LanePoint |
| local_planning | HUAT_LocalPlanning |
| logging_msg | HUAT_Logging |
| msfCone | HUAT_MsfCone |
| path_data | HUAT_PathData |
| time_and_azimuth | HUAT_TimeAndAzimuth |

> If a downstream package still depends on legacy names, update its includes and types to the HUAT_* equivalents.
