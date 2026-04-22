#!/usr/bin/env python3
import jax
import mujoco
import mujoco.mjx as mjx
import brax
import flax
import optax
import gymnasium

print("mujoco:", mujoco.__version__)
print("mjx ok:", mjx is not None)
print("jax:", jax.__version__)
print("devices:", jax.devices())
print("brax:", brax.__version__)
print("flax:", flax.__version__)
print("optax:", optax.__version__)
print("gymnasium:", gymnasium.__version__)
