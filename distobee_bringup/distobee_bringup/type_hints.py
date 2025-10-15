# AUTO-GENERATED FILE. DO NOT EDIT.
# SEE: distobee_bringup/tools/gen_type_hints

from typing import Literal, TypedDict


class Description(TypedDict):
    pass


class Gs(TypedDict):
    pass


class Hardware(TypedDict):
    pass


class Rviz(TypedDict):
    configs: str
    "Space separated RViz configuration file names without extensions, e.g. 'autonomy demo_rgbd'"


class Wheels(TypedDict):
    pass


class BringupConfig(TypedDict):
    description: Description
    "distobee_description"
    gs: Gs
    "distobee_gs"
    hardware: Hardware
    "distobee_hardware"
    rviz: Rviz
    "distobee_rviz"
    wheels: Wheels
    "distobee_wheels"
