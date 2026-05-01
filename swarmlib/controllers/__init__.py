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
except ImportError:
    pass

__all__ = ['BaseController', 'CentroidEstimator', 'MRCapController',
           'DRCapController', 'DRCapDistributedController',
           'ForceCentralisedController', 'ForceCentralisedControllerCVel',
           'ForceDistributedController', 'ForcelessCentralisedControllerCVel']
