# -*- coding: utf-8 -*-
"""
model.py
纯解析映射模块：
joint space <-> cc components 
            <-> tendon length changes <-> motor angles

不依赖 MuJoCo，仅实现几何/解析公式。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple


# =========================
# 基础工具
# =========================

def wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_diff(a: float, b: float) -> float:
    """Smallest wrapped difference a-b in [-pi, pi)."""
    return wrap_to_pi(a - b)


def is_singular_theta(theta: float, eps: float = 1e-10) -> bool:
    """Whether a bending angle is effectively zero."""
    return abs(theta) < eps


# =========================
# 数据结构
# =========================

@dataclass(frozen=True)
class JointSpace:
    """
    Continuum joint-space parameters.

    phi_a   : proximal segment bending plane angle [rad]
    theta_a : proximal segment bending angle [rad]
    phi_c   : distal segment bending plane angle [rad]
    theta_c : distal segment bending angle [rad]
    """
    phi_a: float
    theta_a: float
    phi_c: float
    theta_c: float


@dataclass(frozen=True)
class CCComponents:
    """
    Constant-curvature component representation
    used to compare with MuJoCo fixed-tendon control coordinates.

    u_ax ~ -theta_a*cos(phi_a)
    u_ay ~ -theta_a*sin(phi_a)
    u_cx ~ -theta_c*cos(phi_c)
    u_cy ~ -theta_c*sin(phi_c)
    """
    u_ax: float
    u_ay: float
    u_cx: float
    u_cy: float


@dataclass(frozen=True)
class TendonLengths:
    """
    Tendon length changes for 8 tendons.

    Convention:
    Positive means 'take-up' / shortened geometric tendon path.
    """
    dl1: float
    dl2: float
    dl3: float
    dl4: float
    dl5: float
    dl6: float
    dl7: float
    dl8: float

    def as_tuple(self) -> Tuple[float, ...]:
        return (self.dl1, self.dl2, self.dl3, self.dl4,
                self.dl5, self.dl6, self.dl7, self.dl8)

    def as_dict(self) -> Dict[str, float]:
        return {
            "dl1": self.dl1, "dl2": self.dl2, "dl3": self.dl3, "dl4": self.dl4,
            "dl5": self.dl5, "dl6": self.dl6, "dl7": self.dl7, "dl8": self.dl8,
        }


@dataclass(frozen=True)
class MotorAngles:
    """
    Four actuator/motor angles [rad].
    """
    alpha1: float
    alpha2: float
    alpha3: float
    alpha4: float

    def as_tuple(self) -> Tuple[float, ...]:
        return (self.alpha1, self.alpha2, self.alpha3, self.alpha4)

    def as_dict(self) -> Dict[str, float]:
        return {
            "alpha1": self.alpha1,
            "alpha2": self.alpha2,
            "alpha3": self.alpha3,
            "alpha4": self.alpha4,
        }


@dataclass(frozen=True)
class RecoveredJointSpace:
    """
    Joint space recovered from motor angles.

    singular_a / singular_c indicate that phi_a / phi_c are not physically meaningful
    because theta_a / theta_c is near zero.
    """
    phi_a: float
    theta_a: float
    phi_c: float
    theta_c: float
    singular_a: bool
    singular_c: bool


# =========================
# 主模型
# =========================

class TDRCJointMotorModel:
    """
    Analytical mapping model for:
        joint space <-> tendon length changes <-> motor angles

    Parameters
    ----------
    hole_radius : float
        Distance from discrete-joint center to tendon hole. [m]
    spool_diameter : float
        Effective spool diameter. [m]
    cc_sign : float
        Sign convention for cc components.
        Default = -1.0 to match your current MuJoCo mapping style:
            u_ax = -theta_a*cos(phi_a), etc.
    zero_eps : float
        Threshold for singular bending angle.
    """

    def __init__(
        self,
        hole_radius: float,
        spool_diameter: float,
        cc_sign: float = -1.0,
        zero_eps: float = 1e-10,
    ) -> None:
        if hole_radius <= 0.0:
            raise ValueError("hole_radius must be positive.")
        if spool_diameter <= 0.0:
            raise ValueError("spool_diameter must be positive.")
        if cc_sign == 0.0:
            raise ValueError("cc_sign must be non-zero.")

        self.r_hole = float(hole_radius)
        self.d_spool = float(spool_diameter)
        self.cc_sign = float(cc_sign)
        self.zero_eps = float(zero_eps)

    @property
    def K(self) -> float:
        """K = 2r/d."""
        return 2.0 * self.r_hole / self.d_spool

    # =====================
    # 孔位极角
    # =====================

    @staticmethod
    def gamma_a(i: int) -> float:
        """
        Hole polar angle for proximal segment tendons h_a1...h_a8.
        i in {1,2,3,4,5,6,7,8}
        """
        if i in (1, 2, 3, 4):
            return (i - 1) * math.pi / 2.0
        if i in (5, 6, 7, 8):
            return (i - 5) * math.pi / 2.0 + math.pi / 4.0
        raise ValueError("gamma_a index must be in {1,...,8}.")

    @staticmethod
    def gamma_c(j: int) -> float:
        """
        Hole polar angle for distal segment tendons h_c5...h_c8.
        j in {5,6,7,8}
        """
        if j in (5, 6, 7, 8):
            return (j - 5) * math.pi / 2.0 + math.pi / 4.0
        raise ValueError("gamma_c index must be in {5,6,7,8}.")

    # =====================
    # joint -> cc components
    # =====================

    def joint_to_cc_components(self, joint: JointSpace) -> CCComponents:
        """
        Convert joint-space parameters to constant-curvature vector components.

        By default:
            u_ax = -theta_a*cos(phi_a)
            u_ay = -theta_a*sin(phi_a)
            u_cx = -theta_c*cos(phi_c)
            u_cy = -theta_c*sin(phi_c)
        """
        s = self.cc_sign
        return CCComponents(
            u_ax=s * joint.theta_a * math.cos(joint.phi_a),
            u_ay=s * joint.theta_a * math.sin(joint.phi_a),
            u_cx=s * joint.theta_c * math.cos(joint.phi_c),
            u_cy=s * joint.theta_c * math.sin(joint.phi_c),
        )

    # =====================
    # joint -> tendon lengths
    # =====================

    def joint_to_tendon_lengths(self, joint: JointSpace) -> TendonLengths:
        """
        Compute tendon length changes dl1...dl8 from joint-space parameters.
        """
        r = self.r_hole
        phi_a, theta_a = joint.phi_a, joint.theta_a
        phi_c, theta_c = joint.phi_c, joint.theta_c

        # proximal tendons 1..4
        dl1 = r * theta_a * math.cos(self.gamma_a(1) - phi_a)
        dl2 = r * theta_a * math.cos(self.gamma_a(2) - phi_a)
        dl3 = r * theta_a * math.cos(self.gamma_a(3) - phi_a)
        dl4 = r * theta_a * math.cos(self.gamma_a(4) - phi_a)

        # distal tendons 5..8:
        # contribution from proximal pass-through + distal bending
        dl5 = r * theta_a * math.cos(self.gamma_a(5) - phi_a) + \
              r * theta_c * math.cos(self.gamma_c(5) - phi_c)
        dl6 = r * theta_a * math.cos(self.gamma_a(6) - phi_a) + \
              r * theta_c * math.cos(self.gamma_c(6) - phi_c)
        dl7 = r * theta_a * math.cos(self.gamma_a(7) - phi_a) + \
              r * theta_c * math.cos(self.gamma_c(7) - phi_c)
        dl8 = r * theta_a * math.cos(self.gamma_a(8) - phi_a) + \
              r * theta_c * math.cos(self.gamma_c(8) - phi_c)

        return TendonLengths(
            dl1=dl1, dl2=dl2, dl3=dl3, dl4=dl4,
            dl5=dl5, dl6=dl6, dl7=dl7, dl8=dl8,
        )

    # =====================
    # tendon lengths -> motor angles
    # =====================

    def tendon_lengths_to_motor_angles(self, tendon: TendonLengths) -> MotorAngles:
        """
        Convert tendon length changes to motor angles.

        Using:
            alpha1 = -2/d * dl1
            alpha2 = -2/d * dl2
            alpha3 = -2/d * dl5
            alpha4 = -2/d * dl6
        """
        scale = -2.0 / self.d_spool
        return MotorAngles(
            alpha1=scale * tendon.dl1,
            alpha2=scale * tendon.dl2,
            alpha3=scale * tendon.dl5,
            alpha4=scale * tendon.dl6,
        )

    # =====================
    # joint -> motor angles
    # =====================

    def joint_to_motor_angles(self, joint: JointSpace) -> MotorAngles:
        """
        Direct joint-space -> motor-space mapping.
        """
        K = self.K
        phi_a, theta_a = joint.phi_a, joint.theta_a
        phi_c, theta_c = joint.phi_c, joint.theta_c

        alpha1 = -K * theta_a * math.cos(phi_a)
        alpha2 = -K * theta_a * math.sin(phi_a)
        alpha3 = -K * (
            theta_a * math.cos(math.pi / 4.0 - phi_a) +
            theta_c * math.cos(math.pi / 4.0 - phi_c)
        )
        alpha4 = -K * (
            theta_a * math.cos(3.0 * math.pi / 4.0 - phi_a) +
            theta_c * math.cos(3.0 * math.pi / 4.0 - phi_c)
        )

        return MotorAngles(alpha1=alpha1, alpha2=alpha2, alpha3=alpha3, alpha4=alpha4)

    # =====================
    # motor angles -> joint
    # =====================

    def motor_angles_to_joint(self, motor: MotorAngles) -> RecoveredJointSpace:
        """
        Analytical inverse mapping from motor angles back to joint-space.

        Notes
        -----
        When theta_a or theta_c is near zero, phi_a or phi_c is not physically meaningful.
        In those cases:
            - theta is still computed
            - phi is returned as 0.0 by convention
            - singular_a / singular_c is flagged True
        """
        K = self.K
        if abs(K) < self.zero_eps:
            raise ZeroDivisionError("Invalid K: too close to zero.")

        a1, a2, a3, a4 = motor.alpha1, motor.alpha2, motor.alpha3, motor.alpha4

        # proximal segment
        theta_a = math.sqrt(a1 * a1 + a2 * a2) / K
        singular_a = is_singular_theta(theta_a, self.zero_eps)

        if singular_a:
            phi_a = 0.0
        else:
            phi_a = wrap_to_pi(math.atan2(-a2, -a1))

        # remove proximal contribution from distal motors
        u = -a3 / K - theta_a * math.cos(math.pi / 4.0 - phi_a)
        v = -a4 / K - theta_a * math.cos(3.0 * math.pi / 4.0 - phi_a)

        # distal segment
        theta_c = math.sqrt(u * u + v * v)
        singular_c = is_singular_theta(theta_c, self.zero_eps)

        if singular_c:
            phi_c = 0.0
        else:
            # from:
            # theta_c*cos(phi_c) = (u-v)/sqrt(2)
            # theta_c*sin(phi_c) = (u+v)/sqrt(2)
            phi_c = wrap_to_pi(math.atan2(u + v, u - v))

        return RecoveredJointSpace(
            phi_a=phi_a,
            theta_a=theta_a,
            phi_c=phi_c,
            theta_c=theta_c,
            singular_a=singular_a,
            singular_c=singular_c,
        )

    # =====================
    # 便捷接口
    # =====================

    def joint_to_all(self, joint: JointSpace) -> Dict[str, object]:
        """
        Convenience function returning all forward-mapping results.
        """
        cc = self.joint_to_cc_components(joint)
        tendon = self.joint_to_tendon_lengths(joint)
        motor = self.tendon_lengths_to_motor_angles(tendon)
        motor_direct = self.joint_to_motor_angles(joint)

        return {
            "joint": joint,
            "cc": cc,
            "tendon": tendon,
            "motor_from_tendon": motor,
            "motor_direct": motor_direct,
        }
