import warnings

warnings.warn(
    "hpp.corbaserver.manipulation has been moved to hpp.manipulation",
    DeprecationWarning,
)

from hpp.manipulation.client import *  # noqa: E402, F403
