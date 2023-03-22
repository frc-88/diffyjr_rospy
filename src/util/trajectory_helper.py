import os
import sys
from wpimath.trajectory import Trajectory, TrajectoryUtil


class TrajectoryHelper:
    @classmethod
    def get_root_dir(cls) -> str:
        main_module = sys.modules["__main__"]
        if not hasattr(main_module, "__file__"):
            return os.path.dirname(os.path.abs(sys.argv[0]))
        root_main = os.path.abspath(str(main_module.__file__))
        root_dir = os.path.dirname(root_main)
        return root_dir

    @classmethod
    def get_path_planner_dir(cls) -> str:
        return os.path.join(cls.get_root_dir(), "pathplanner", "generatedJSON")

    @classmethod
    def from_pathweaver_json(cls, name) -> Trajectory:
        path = os.path.join(cls.get_path_planner_dir(), name)
        try:
            trajectory = TrajectoryUtil.fromPathweaverJson(path)
        except IOError as e:
            print(f"Unable to open trajectory {name}. {str(e)}")
            trajectory = Trajectory()
        return trajectory
