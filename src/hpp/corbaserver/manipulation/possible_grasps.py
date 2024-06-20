import warnings

warnings.warn(
    "hpp.corbaserver.manipulation has been moved to hpp_manipulation",
    DeprecationWarning,
)

from hpp_manipulation.possible_grasps import *  # noqa: E402, F403
