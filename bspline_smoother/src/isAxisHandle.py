import matplotlib.pyplot as plt

def isAxisHandle(arg):
    """
    Check if the input corresponds to a valid axis handle.

    B = isAxisHandle(VAR)
    If the value of VAR is scalar, corresponds to a valid matplotlib handle,
    and has type equal to 'Axes', then returns True. Otherwise, returns False.
    This function is used to check if the first argument of drawing functions
    corresponds to data or to an axis handle to draw in.
    """

    return (
        isinstance(arg, plt.Axes)
        or (hasattr(arg, "__len__") and len(arg) == 1 and isinstance(arg[0], plt.Axes))
    )