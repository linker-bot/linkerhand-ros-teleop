import enum
from pathlib import Path
from typing import Optional


class RobotName(enum.Enum):
    allegro = enum.auto()
    shadow = enum.auto()
    svh = enum.auto()
    leap = enum.auto()
    ability = enum.auto()
    inspire = enum.auto()
    panda = enum.auto()
    linker_l10 = enum.auto()
    linker_l20 = enum.auto()
    linker_l18 = enum.auto()
    linker_l16 = enum.auto()
    linker_t24 = enum.auto()


class RetargetingType(enum.Enum):
    vector = enum.auto()  # For teleoperation, no finger closing prior
    position = enum.auto()  # For offline data processing, especially hand-object interaction data
    dexpilot = enum.auto()  # For teleoperation, with finger closing prior
    projection = enum.auto()


class HandType(enum.Enum):
    right = enum.auto()
    left = enum.auto()


ROBOT_NAME_MAP = {
    RobotName.allegro: "allegro_hand",
    RobotName.shadow: "shadow_hand",
    RobotName.svh: "schunk_svh_hand",
    RobotName.leap: "leap_hand",
    RobotName.ability: "ability_hand",
    RobotName.inspire: "inspire_hand",
    RobotName.panda: "panda_gripper",
    RobotName.linker_l10: "linker_hand_l10",
    RobotName.linker_l20: "linker_hand_l20",
    RobotName.linker_l18: "linker_hand_l18",
    RobotName.linker_l16: "linker_hand_l16",
    RobotName.linker_t24: "linker_hand_t24",
}

ROBOT_NAMES = list(ROBOT_NAME_MAP.keys())


def get_default_config_path(
        robot_name: RobotName, retargeting_type: RetargetingType, hand_type: HandType
) -> Optional[Path]:
    config_path = Path(__file__).parent / "configs"
    if retargeting_type is RetargetingType.position:
        config_path = config_path / "offline"
    else:
        config_path = config_path / "teleop"

    robot_name_str = ROBOT_NAME_MAP[robot_name]
    hand_type_str = hand_type.name
    if "gripper" in robot_name_str:  # For gripper robots, only use gripper config file.
        if retargeting_type == RetargetingType.dexpilot:
            config_name = f"{robot_name_str}_dexpilot.yml"
        else:
            config_name = f"{robot_name_str}.yml"
    else:
        if retargeting_type == RetargetingType.dexpilot:
            config_name = f"{robot_name_str}_{hand_type_str}_dexpilot.yml"
        else:
            config_name = f"{robot_name_str}_{hand_type_str}.yml"
    return config_path / config_name
