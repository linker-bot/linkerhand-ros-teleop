from pathlib import Path
from linkerhand_core.canfun import *
from linkerhand_core.handcore import *
from linkerhand_core.config import HandConfig
from linkerhand_core.constants import RetargetingType, DataSource, MotionSource, RobotName


class HandRetarget:
    def __init__(self):
        self.robot_dir = Path(__file__).absolute().parent.parent / "assets" / "robots" / "hands"
        self.base_config = Path(__file__).absolute().parent

        self.handconfig = HandConfig(str(self.robot_dir), str(self.base_config))
        self.handcore = HandCore(self.handconfig)

        self.baseconfig = self.handconfig.baseconfig
        self.retagetconfig = self.handconfig.retagetconfig

        #
        self.scene, self.retargeting_r, self.retargeting_l, self.config_r, self.config_l = None, None, None, None, None
        self.robot_name_r, self.robot_name_l = None, None
        self.retargeting_type = None
        self.datasource_type = None
        self.motion_type = None
        self.udp_ip, self.udp_port, self.use_can, self.motion_device = None, None, None, None

        self.calibration = None
        self.calibrationopen_r, self.calibrationopen_l, self.calibrationclose_r, self.calibrationclose_l = None, None, None, None
        self.retarget = None

    def dexretargetconfig(self):
        self.datasource_type = DataSource[self.baseconfig["system"]["datasource_type"]]
        self.retargeting_type = RetargetingType[self.baseconfig["system"]["retargeting_type"]]
        self.motion_type = MotionSource[self.baseconfig["system"]["motion_type"]]
        self.robot_name_r = RobotName[self.baseconfig["system"]["robotname_r"]]
        self.robot_name_l = RobotName[self.baseconfig["system"]["robotname_l"]]

        self.udp_ip = self.baseconfig["udp"]["ip"]
        self.udp_port = int(self.baseconfig["udp"]["port"])
        self.use_can = bool(self.baseconfig["system"]["usecan"])
        self.motion_device = self.baseconfig["system"]["motion_device"]

    def retargetrun(self):
        sender = None
        if self.use_can:
            sender = CanMessageSender(bitrate=self.baseconfig["system"]["can"]["bitrate"],
                                      id=self.baseconfig["system"]["can"]["id"],
                                      type=self.baseconfig["system"]["can"]["dofs"])
        if self.motion_type == MotionSource.sensenova:
            from motion.nova.retarget import Retarget
            self.retarget = Retarget(ip=self.retarget,
                                     port=self.udp_port,
                                     righthand=self.robot_name_r,
                                     lefthand=self.robot_name_l,
                                     handcore=self.handcore,
                                     cansender=sender)
        if self.retarget is None:
            print("未正确创建应用实例")
        else:
            print("启动应用实例")
            self.retarget.process()


if __name__ == "__main__":
    retarget = HandRetarget()
    retarget.dexretargetconfig()
    retarget.retargetrun()
