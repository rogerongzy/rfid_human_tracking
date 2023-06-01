It seems there is no perfect unwrapping mathod. The threshold depends on the variation rate of the tag velocity. Move the tag slow enough to avoid mistakes.
Not sure because of the velocity is too big, or the available reading number is too less.

To ensure the correct unwrapped phase, monotonicity should be considered. For example, under a certain updating rate, the adjacent 3 will not show 2 changes.
+++ ++- +-- --- --+ -++ is possible, +-+ -+- is impossible.

### Conduct on simple trajectory first, with balanced velocity and same direction.
### During unwrappping, when varying in wave_length of antenna, ant 2&3 / 1&4 show similar feature. Maybe use different params?



If the apriltag ran out of the camera frame during detecting, the position relative to the camera will run pretty large which cause bad effect in distance estimation.
