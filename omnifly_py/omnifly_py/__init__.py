from .omnifly_swarm import OmniFly

# Alias so scripts written for Crazyswarm2 only need the import line changed.
Crazyswarm = OmniFly

__all__ = ['OmniFly', 'Crazyswarm']
