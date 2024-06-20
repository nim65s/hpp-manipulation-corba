import warnings

warnings.warn(
    "hpp.corbaserver.manipulation has been moved to hpp_manipulation",
    DeprecationWarning,
)

from hpp_manipulation.constraint_graph_factory import *  # noqa: E402, F403
