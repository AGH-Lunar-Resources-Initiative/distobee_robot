# AUTO-GENERATED FILE. DO NOT EDIT.
# SEE: distobee_bringup/tools/gen_type_hints

from typing import Literal, TypedDict


class Description(TypedDict):
    joint_state_publisher_gui: Literal["false", "true"]
    "Start the joint state publisher in GUI mode. Valid choices are: ['true', 'false']"


class Hardware(TypedDict):
    pass


class Rviz(TypedDict):
    configs: str
    "Space separated RViz configuration file names without extensions, e.g. 'autonomy demo_rgbd'"


class BringupConfig(TypedDict):
    description: Description
    "distobee_description"
    hardware: Hardware
    "distobee_hardware"
    rviz: Rviz
    "distobee_rviz"
