# ekf-sensor-fusion

Bayes filter is a fundamental framework or template for more advanced filters. As long as you are having some intuition as to how it all works, read on and you will soon see how the Kalman Filter, which derives from the Bayes Filter will be implemented. [View  detailed derivation](bayes-filter.pdf)  
![Bayes Filter](https://latex.codecogs.com/svg.image?bel(x_t)&space;=&space;\eta&space;\cdot&space;p(z_t&space;\mid&space;x_t)&space;\cdot&space;\int&space;p(x_t&space;\mid&space;u_t,&space;x_{t-1})&space;\cdot&space;bel(x_{t-1})&space;dx_{t-1})

- Each family of State Estimation algorithms based on Bayes filter differs in how they treat the measurement and state transition probabilities, as well as the initial belief, which, in turn, affects how the posteriors (updated belief) are represented.
---
## Gaussian Filters
- Gaussian Filters are the earliest and most popular implementations of Bayes Filters. **They are called Gaussian filters as belief and noise(in sensor and action models) are represented as Gaussian(Normal) distributions**
- The variable x: the state of the robot, is a vector with values corresponding to the aspects being modeled.
   e.g. If state of the robot is its pose, x vector will contain spatial coordinates [x,y,x] and orientation [roll,pitch,yaw] /quaternions
- The most probable estimate of x (The probability density function of x) is modelled as   





