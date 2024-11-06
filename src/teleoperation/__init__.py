import numpy as np
from omegaconf import OmegaConf

np.set_printoptions(precision=2, suppress=True)


def numpy_eval(x):
    import numpy as np  # noqa: F401

    return eval(x)


OmegaConf.register_new_resolver("eval", numpy_eval, replace=True)
