import warnings

warnings.warn(
    "hpp.corbaserver.manipulation has been moved to hpp.manipulation",
    DeprecationWarning,
)

from hpp.manipulation.constraint_graph_factory import *  # noqa: E402, F403
