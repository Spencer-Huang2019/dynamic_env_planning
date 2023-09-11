"""
toppra.algorithm.algorithm
^^^^^^^^^^^^^^^^^^^^^^^^^^

This module defines the abstract data types that define TOPP algorithms.

"""
from typing import Dict, Any, List, Tuple, Optional
import typing as T
import abc
import enum
import numpy as np
import time
import matplotlib.pyplot as plt

from toppra.constants import TINY
from toppra.interpolator import SplineInterpolator, AbstractGeometricPath
from toppra.constraint import Constraint
import toppra.interpolator as interpolator
import toppra.parametrizer as tparam

import logging

logger = logging.getLogger(__name__)


class ParameterizationData(object):
    """Internal data and output.
    """
    def __init__(self, *arg, **kwargs):
        self.return_code = ParameterizationReturnCode.ErrUnknown
        "ParameterizationReturnCode: Return code of the last parametrization attempt."
        self.gridpoints = None
        "np.ndarray: Shape (N+1, 1). Gridpoints"
        self.sd_vec = None
        "np.ndarray: Shape (N+1, 1). Path velocities"
        self.sdd_vec = None
        "np.ndarray: Shape (N+1, 1). Path acceleration"

        self.sd_vec_j = None
        "np.ndarray: Shape (N+1, 1). Path velocities"
        self.sdd_vec_j = None
        "np.ndarray: Shape (N+1, 1). Path acceleration"
        self.sddd_vec_j = None
        "np.ndarray: Shape (N+1, 1). Path acceleration"

        self.K = None
        "np.ndarray: Shape (N+1, 2). Controllable sets."
        self.X = None
        "np.ndarray: Shape (N+1, 2). Feasible sets."
        self.L = None

    def __repr__(self):
        return "ParameterizationData(return_code:={}, N={:d})".format(
            self.return_code, self.gridpoints.shape[0])


class ParameterizationReturnCode(enum.Enum):
    """Return codes from a parametrization attempt.
    """
    Ok = "Ok: Successful parametrization"
    ErrUnknown = "Error: Unknown issue"
    ErrShortPath = "Error: Input path is very short"
    FailUncontrollable = "Error: Instance is not controllable"
    ErrForwardPassFail = "Error: Forward pass fail. Numerical errors occured"

    def __repr__(self):
        return super(ParameterizationReturnCode, self).__repr__()

    def __str__(self):
        return super(ParameterizationReturnCode, self).__repr__()


class ParameterizationAlgorithm(object):
    """Base parametrization algorithm class.

    This class specifies the generic behavior for parametrization algorithms.  For details on how
    to *construct* a :class:`ParameterizationAlgorithm` instance, as well as configure it, refer
    to the specific class.

    Example usage:

    .. code-block:: python

      # usage
      instance.compute_parametrization(0, 0)
      output = instance.problem_data

      # do this if you only want the final trajectory
      traj = instance.compute_trajectory(0, 0)

    .. seealso::

        :class:`toppra.algorithm.TOPPRA`,
        :class:`toppra.algorithm.TOPPRAsd`,
        :class:`~ParameterizationReturnCode`,
        :class:`~ParameterizationData`

    """

    def __init__(self, constraint_list, path, gridpoints=None, parametrizer=None,
                 gridpt_max_err_threshold=2e-2, gridpt_min_nb_points=60):
        self.constraints = constraint_list
        self.path = path  # Attr
        self._problem_data = ParameterizationData()
        # Handle gridpoints
        if gridpoints is None:
            gridpoints = interpolator.propose_gridpoints(
                path,
                max_err_threshold=gridpt_max_err_threshold,
                min_nb_points=gridpt_min_nb_points
            )
            logger.info(
                "No gridpoint specified. Automatically choose a gridpoint with %d points",
                len(gridpoints)
            )

        if (
            path.path_interval[0] != gridpoints[0]
            or path.path_interval[1] != gridpoints[-1]
        ):
            raise ValueError("Invalid manually supplied gridpoints.")
        self.gridpoints = np.array(gridpoints)
        self._problem_data.gridpoints = np.array(gridpoints)
        self._N = len(gridpoints) - 1  # Number of stages. Number of point is _N + 1
        for i in range(self._N):
            if gridpoints[i + 1] <= gridpoints[i]:
                logger.fatal("Input gridpoints are not monotonically increasing.")
                raise ValueError("Bad input gridpoints.")
        if parametrizer is None or parametrizer == "ParametrizeSpline":
            # TODO: What is the best way to type parametrizer?
            self.parametrizer = tparam.ParametrizeSpline
        elif parametrizer == "ParametrizeConstAccel":
            self.parametrizer = tparam.ParametrizeConstAccel

    @property
    def constraints(self):
        """Constraints of interests."""
        return self._constraints

    @constraints.setter
    def constraints(self, value):
        # TODO: Validate constraints.
        self._constraints = value

    @property
    def problem_data(self):
        """Data obtained when solving the path parametrization."""
        return self._problem_data

    @abc.abstractmethod
    def compute_parameterization(self, sd_start, sd_end, return_data=False):
        """Compute the path parameterization subject to starting and ending conditions.

        After this method terminates, the attribute
        :attr:`~problem_data` will contain algorithm output, as well
        as the result. This is the preferred way of retrieving problem
        output.

        Parameters
        ----------
        sd_start:
            Starting path velocity. Must be positive.
        sd_end:
            Goal path velocity. Must be positive.
        return_data:
            If true also return the problem data.

        """
        raise NotImplementedError

    def compute_trajectory(self, sd_start=0, sd_end=0):
        """Compute the resulting joint trajectory and auxilliary trajectory.

        This is a convenient method if only the final output is wanted.

        Parameters
        ----------
        sd_start:
            Starting path velocity.
        sd_end:
            Goal path velocity.
        return_data:
            If true, return a dict containing the internal data.

        Returns
        -------
        :
            Time-parameterized joint position trajectory or
            None If unable to parameterize. 

        """
        t0 = time.time()
        self.compute_parameterization(sd_start, sd_end)
        if self.problem_data.return_code != ParameterizationReturnCode.Ok:
            logger.warn("Fail to parametrize path. Return code: %s", self.problem_data.return_code)
            return None

        outputtraj = self.parametrizer(self.path, self.problem_data.gridpoints, self.problem_data.sd_vec)
        logger.info("Successfully parametrize path. Duration: %.3f, previously %.3f)",
                    outputtraj.path_interval[1], self.path.path_interval[1])
        logger.info("Finish parametrization in %.3f secs", time.time() - t0)
        return outputtraj

    def compute_trajectory_jerk(self, sd_start=0, sd_end=0):
        """Compute the resulting joint trajectory and auxilliary trajectory.

        This is a convenient method if only the final output is wanted.

        Parameters
        ----------
        sd_start:
            Starting path velocity.
        sd_end:
            Goal path velocity.
        return_data:
            If true, return a dict containing the internal data.

        Returns
        -------
        :
            Time-parameterized joint position trajectory or
            None If unable to parameterize.

        """
        t0 = time.time()
        self.compute_parameterization(sd_start, sd_end)
        if self.problem_data.return_code != ParameterizationReturnCode.Ok:
            logger.warn("Fail to parametrize path. Return code: %s", self.problem_data.return_code)
            return None

        outputtraj = self.parametrizer(self.path, self.problem_data.gridpoints, self.problem_data.sd_vec_j,
                                       third_order=True, sdd_j=self.problem_data.sdd_vec_j,
                                       sddd_j=self.problem_data.sddd_vec_j)
        logger.info("Successfully parametrize path. Duration: %.3f, previously %.3f)",
                    outputtraj.path_interval[1], self.path.path_interval[1])
        logger.info("Finish parametrization in %.3f secs", time.time() - t0)
        return outputtraj

    def compute_trajectory_jerk_DP3(self, sd_start=0, sd_end=0):
        """Compute the resulting joint trajectory and auxilliary trajectory.

        This is a convenient method if only the final output is wanted.

        Parameters
        ----------
        sd_start:
            Starting path velocity.
        sd_end:
            Goal path velocity.
        return_data:
            If true, return a dict containing the internal data.

        Returns
        -------
        :
            Time-parameterized joint position trajectory or
            None If unable to parameterize.

        """
        qs, qd, ts = self.compute_parameterization(sd_start, sd_end)

        return qs, qd, ts

    def inspect(self, compute=True):
        """Inspect the problem internal data."""
        K = self.problem_data.K
        X = self.problem_data.L
        if X is not None:
            plt.plot(X[:, 0], c="green", label="Reachable sets")
            plt.plot(X[:, 1], c="green")
        if K is not None:
            plt.plot(K[:, 0], "--", c="red", label="Controllable sets")
            plt.plot(K[:, 1], "--", c="red")
        if self.problem_data.sd_vec is not None:
            plt.plot(self.problem_data.sd_vec ** 2, label="Velocity profile")
        plt.title("Path-position path-velocity plot")
        plt.xlabel("Path position")
        plt.ylabel("Path velocity square")
        plt.legend()
        plt.tight_layout()
        plt.show()


