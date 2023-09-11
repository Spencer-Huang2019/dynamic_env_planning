"""This module implements the joint velocity constraint."""
import numpy as np
from toppra._CythonUtils import (_create_velocity_constraint,
                                 _create_velocity_constraint_varying)
from .linear_constraint import LinearConstraint


class JointJerkConstraint(LinearConstraint):
    """A Joint Velocity Constraint class.

    Parameters
    ----------
    vlim: np.ndarray
        Shape (dof, 2). The lower and upper velocity bounds of the j-th joint
        are given by vlim[j, 0] and vlim[j, 1] respectively.

    """

    def __init__(self, jlim):
        super(JointJerkConstraint, self).__init__()
        jlim = np.array(jlim, dtype=float)
        if np.isnan(jlim).any():
            raise ValueError("Bad velocity given: %s" % jlim)
        if len(jlim.shape) == 1:
            self.jlim = np.vstack((-np.array(jlim), np.array(jlim))).T
        else:
            self.jlim = np.array(jlim, dtype=float)
        self.dof = self.jlim.shape[0]
        self._assert_valid_limits()

    def _assert_valid_limits(self):
        """Check that the velocity limits is valid."""
        assert self.jlim.shape[1] == 2, "Wrong input shape."
        for i in range(self.dof):
            if self.jlim[i, 0] >= self.jlim[i, 1]:
                raise ValueError("Bad velocity limits: {:} (lower limit) > {:} (higher limit)".format(
                    self.jlim[i, 0], self.jlim[i, 1]))
        self._format_string = "    Velocity limit: \n"
        for i in range(self.jlim.shape[0]):
            self._format_string += "      J{:d}: {:}".format(
                i + 1, self.jlim[i]) + "\n"

    @property
    def formal(self):
        return self.jlim

