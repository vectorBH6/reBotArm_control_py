"""Dynamics 机器人模型模块。

封装 Pinocchio 模型，提供动力学专用的模型操作工具，
包括重力配置、数据缓存和批量雅可比求解器。

该模块与 kinematics.robot_model 共享同一个 URDF 解析入口，
但专注于动力学计算所需的模型状态和参数。
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
import pinocchio as pin

from ..kinematics.robot_model import load_robot_model as _load_kin_model


# --------------------------------------------------------------------------- #
# 默认重力加速度常量
# --------------------------------------------------------------------------- #

EARTH_GRAVITY: Tuple[float, float, float] = (0.0, 0.0, -9.81)
"""地球标准重力加速度 (m/s²)，方向沿世界 -z 轴。"""

ZERO_GRAVITY: Tuple[float, float, float] = (0.0, 0.0, 0.0)
"""零重力环境。"""

# 全局模型缓存（惰性加载，避免每次调用都重新解析 URDF）
_CACHED_MODEL: Optional[pin.Model] = None


# --------------------------------------------------------------------------- #
# 模型加载
# --------------------------------------------------------------------------- #

def load_dynamics_model(urdf_path: Optional[str] = None) -> pin.Model:
    """构建 reBot-DevArm 的 Pinocchio 动力学模型。

    内部调用 kinematics.robot_model.load_robot_model，共享同一个 URDF。
    模型默认重力为 ``(0, 0, -9.81)`` m/s²。

    参数:
        urdf_path: URDF 文件路径，默认为内置 reBot-DevArm URDF。

    返回:
        pin.Model：包含 6 个旋转关节（nq=6, nv=6）的刚体动力学模型。
    """
    global _CACHED_MODEL
    if _CACHED_MODEL is not None and urdf_path is None:
        return _CACHED_MODEL
    model = _load_kin_model(urdf_path=urdf_path)
    if urdf_path is None:
        _CACHED_MODEL = model
    return model


def create_data(model: Optional[pin.Model] = None) -> pin.Data:
    """创建 Pinocchio Data 对象。

    Data 存储所有动力学计算结果（质量矩阵、科氏力向量等）。
    每次动力学计算前需调用此函数重置数据。

    参数:
        model: 已加载的动力学模型。
               若为 None，则自动加载默认模型。

    返回:
        pin.Data：与模型关联的数据容器。
    """
    if model is None:
        model = load_dynamics_model()
    return model.createData()


# --------------------------------------------------------------------------- #
# 重力配置
# --------------------------------------------------------------------------- #

def get_default_gravity() -> np.ndarray:
    """返回默认地球重力向量（3 维 numpy 数组）。

    返回:
        shape=(3,) 的 ndarray，单位：m/s²。
        默认值：[0, 0, -9.81]。
    """
    return np.array(EARTH_GRAVITY)


def set_gravity(
    model: pin.Model,
    gravity: Tuple[float, float, float] | np.ndarray,
) -> None:
    """设置模型的重力加速度向量。

    重力向量会直接写入 ``model.gravity``，影响所有后续动力学计算。

    参数:
        model:   目标动力学模型。
        gravity: 三维重力向量 (gx, gy, gz)，单位 m/s²。
                 常见预设：

                 - ``(0, 0, -9.81)`` — 地球（默认）
                 - ``(0, 0, -1.62)`` — 月球
                 - ``(0, 0,  0  )`` — 零重力

    示例:
        .. code-block:: python

            import pinocchio as pin
            from reBotArm_control_py.dynamics import load_dynamics_model, set_gravity

            model = load_dynamics_model()
            set_gravity(model, (0, 0, -9.81))   # 地球重力
            set_gravity(model, (0, 0, -1.62))   # 月球重力
    """
    if isinstance(gravity, (tuple, list)):
        gravity = np.array(gravity, dtype=float)
    model.gravity = pin.Motion(gravity)


def get_gravity(model: pin.Model) -> np.ndarray:
    """获取当前模型的重力加速度向量。

    参数:
        model: 动力学模型。

    返回:
        shape=(3,) 的 ndarray，单位：m/s²。
    """
    g = model.gravity
    return np.array([g.linear.x, g.linear.y, g.linear.z])


# --------------------------------------------------------------------------- #
# 零位/初始构型工具
# --------------------------------------------------------------------------- #

def neutral_configuration(model: Optional[pin.Model] = None) -> np.ndarray:
    """返回机器人的零位关节角度向量。

    参数:
        model: 动力学模型。若为 None，则自动加载。

    返回:
        shape=(nq,) 的 ndarray，全零向量。
    """
    if model is None:
        model = load_dynamics_model()
    return pin.neutral(model)


def random_configuration(
    model: Optional[pin.Model] = None,
) -> np.ndarray:
    """返回随机关节角度向量（各关节限位内均匀采样）。

    参数:
        model: 动力学模型。若为 None，则自动加载。

    返回:
        shape=(nq,) 的 ndarray。
    """
    if model is None:
        model = load_dynamics_model()
    return pin.randomConfiguration(model)
