import warnings

warnings.warn(
    "hpp.corbaserver.manipulation has been moved to hpp.manipulation",
    DeprecationWarning,
)

from hpp.manipulation.possible_grasps import *  # noqa: E402, F403
