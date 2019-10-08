This code is my attempt at optimizing flight in Super Mario 64.

The game attempts to use energy conservation to ensure that the player can never gain both height and speed simultaneously. However, acceleration when tilting downward scales linearly with the Mario's pitch, rather than with the sine of his pitch. For this reason, "energy" increases when moving sharply downward. The initial goal of this code was to prove that this is enough to counteract the constant deceleration due to drag, resulting in infinite height/speed given an infinite wing cap.

It failed in this goal. However, I noticed that flying against OOB causes Mario to tilt down an extra amount on each frame. Because of frame order, this tilt works in our favor (we get the extra speed increase, but not the extra height decrease). Conveniently the level we care about, Wing Mario over the Rainbow, lets us fly against OOB. This quirk is enough to give us a net positive height/speed increase.

See [video demo here](https://www.youtube.com/watch?v=826gWUnF-cM). Since this video, I managed to optimize the flight a lot, but it still isn't fast enough to save the A press. Improvement is still possible - I may revisit later.
