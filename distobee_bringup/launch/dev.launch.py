from distobee_bringup import *


def generate_launch_description():
    return gen_launch(
        {
            "description": {},
            "hardware": {},
            "rviz": {"configs": "demo_urdf_model"},
            "wheels": {},
        }
    )
