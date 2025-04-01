def clamp(val, min_val, max_val):
    """Clamps a value between a minimum and maximum value

    Args:
        val: Value to clamp
        min_val: Minimum value
        max_val: Maximum value

    Returns:
        Clamped value
    """
    return min(max_val, max(min_val, val))