"""
Controllers for multi-robot payload transport.
"""

from .base_controller import BaseController
from .centroid_estimator import CentroidEstimator
from .drcap_distributed_controller import DRCapDistributedController
from .force_distributed_controller import ForceDistributedController

# gtsam-dependent controllers — not available on platforms without gtsam
try:
    from .mrcap_controller import MRCapController
    from .drcap_centralised_controller import DRCapController
    from .force_centralised_controller import ForceCentralisedController
    from .force_centralised_controller_cvel import ForceCentralisedControllerCVel
    from .forceless_centralised_controller import ForcelessCentralisedControllerCVel
except ImportError as _gtsam_err:
    _msg = (
        f"gtsam-dependent controllers are not available on this platform "
        f"({_gtsam_err}). Install gtsam (https://gtsam.org) to use "
        f"MRCapController, DRCapController, ForceCentralisedController, "
        f"ForceCentralisedControllerCVel, or ForcelessCentralisedControllerCVel."
    )

    class _GtsamMissing:
        def __init__(self, *args, **kwargs):
            raise ImportError(_msg)

    MRCapController = type("MRCapController", (_GtsamMissing,), {})
    DRCapController = type("DRCapController", (_GtsamMissing,), {})
    ForceCentralisedController = type("ForceCentralisedController", (_GtsamMissing,), {})
    ForceCentralisedControllerCVel = type("ForceCentralisedControllerCVel", (_GtsamMissing,), {})
    ForcelessCentralisedControllerCVel = type("ForcelessCentralisedControllerCVel", (_GtsamMissing,), {})

__all__ = ['BaseController', 'CentroidEstimator', 'MRCapController',
           'DRCapController', 'DRCapDistributedController',
           'ForceCentralisedController', 'ForceCentralisedControllerCVel',
           'ForceDistributedController', 'ForcelessCentralisedControllerCVel']
