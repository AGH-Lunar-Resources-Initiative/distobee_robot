from distobee_bringup import *


def generate_launch_description():
    return gen_launch(
        {
            "description": {
                "joint_state_publisher_gui": "true",
            },
            "hardware": {},
        }
    )
