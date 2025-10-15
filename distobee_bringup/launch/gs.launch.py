from distobee_bringup import *


def generate_launch_description():
    return gen_launch(
        {
            "gs": {},
            "rviz": {"configs": "demo_urdf_model"},
            "wheels": {},
            "sifter": {},
        }
    )
