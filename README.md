# bbn-atomS3R
BBN m5stack atomS3R

Magnetometer calibration with m5unified library is not enough. It only estimates offset biases.

Proper calibration would include measuring scale biases as well.

There are better tools to calibrate. Check: https://github.com/bareboat-necessities/bbn-MotionCal

Calibration procedure can be further improved to estimate not only biases but also variances and covariances for 
Kalman filters used for fusion.
