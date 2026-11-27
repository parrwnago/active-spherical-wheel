"""Frozen I60 pure estimated-load labeling function; used only for fallback parity.
No evaluator I/O or truth code is imported.
"""
import numpy as np

def dominant_labels(loads, minimum_total, minimum_share):
    """Exactly tied or missing positive maxima remain unknown (-1)."""
    loads = np.asarray(loads, float)
    finite = np.isfinite(loads).all(axis=1)&(loads >= 0).all(axis=1)
    clean = np.where(np.isfinite(loads), loads, 0.)
    total, maximum = clean.sum(axis=1), clean.max(axis=1)
    unique = (clean == maximum[:,None]).sum(axis=1) == 1
    share = np.divide(maximum, total, out=np.zeros_like(total), where=total > 0)
    loaded = finite&(total >= minimum_total)&(maximum > 0)
    eligible = loaded&unique&(share >= minimum_share)
    family = np.where(eligible, np.argmax(clean, axis=1), -1)
    return family, eligible, loaded, share, unique, total
