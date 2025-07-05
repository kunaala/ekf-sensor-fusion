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
  ![Multivariate Gaussian](https://latex.codecogs.com/svg.image?p(\mathbf{x})%20=%20\frac{1}{(2\pi)^{d/2}%20|\boldsymbol{\Sigma}|^{1/2}}%20\exp\left(%20-\frac{1}{2}%20(\mathbf{x}%20-%20\boldsymbol{\mu})^\top%20\boldsymbol{\Sigma}^{-1}%20(\mathbf{x}%20-%20\boldsymbol{\mu})%20\right))

   μ is the mean vector (center of the distribution)   
   Σ is the covariance matrix (describes the spread/shape)   
  `d` is the dimensionality of `x`  
  The exponential term penalizes how far x is from μ, scaled by the covariance

`Note:`Why Gaussian was chosen for belief?
-  Gaussians are closed under linear transformations : When a Gaussian random variable undergoes a linear transformation, the outcome remains a Gaussian random variable.  
![Linear Transformation of Gaussian](https://latex.codecogs.com/svg.image?\boldsymbol{x}\sim\mathcal{N}(\boldsymbol{\mu},\boldsymbol{\Sigma})\Rightarrow\boldsymbol{y}=\mathbf{A}\boldsymbol{x}+\boldsymbol{b}\sim\mathcal{N}(\mathbf{A}\boldsymbol{\mu}+\boldsymbol{b},\mathbf{A}\boldsymbol{\Sigma}\mathbf{A}^\top))

Now we have everything in place to understand the Kalman filter!!

---

## Kalman Filter
- We learnt that Bayes Filter predicts the state using 2 steps:  

**Prediction**: bel⁻(xₜ) = ∫ p(xₜ|xₜ₋₁, uₜ) * bel(xₜ₋₁) dxₜ₋₁  
**Update**: bel(xₜ) = η * p(zₜ|xₜ) * bel⁻(xₜ)  

- In the Kalman Filter, we make the follwing assumptions to obtain a _gaussian_ state estimate
  
  1. belief bel(x) is always represented as a multivariate Gaussian distribution:  
        bel(x) = N(μ, Σ)  
  2. Action model(state transition)  follows linear dynamics:   
       xₜ = A*xₜ₋₁ + B*uₜ + wₜ( ~ N(0, Q) Gaussian noise)
  3. Sensor model has linear observations:  
        zₜ = Cxₜ + vₜ ( ~ N(0, R) Gaussian noise)
  

### Prediction: Computes predicted _mean_ and _covariance_ 

- Given the action model,  xₜ = A*xₜ₋₁ + B*uₜ + wₜ, we can easily derive the predicted mean and variance by linear transformation property of Gaussian

  predicted mean μ⁻ₜ = A*μₜ₋₁ + B*uₜ  
  predicted covariance Σ⁻ₜ =  A*Σₜ₋₁*Aᵀ + Q
  



### Update: Computes updated _mean_ and _covariance_ using observation

- **Kalman gain** is calculated to determine the extent to which the measurement affects the new state estimate
- ![Kalman Gain](https://latex.codecogs.com/svg.image?\dpi{150}K%20=%20\frac{\text{Prediction%20Uncertainty}%20\cdot%20C^\top}{\text{Total%20Measurement%20Uncertainty}})  
   Where:
      Prediction Uncertainty: Σ⁻ (from prediction Gaussian)    
      Measurement Jacobian: Cᵀ (  Cᵀ  maps back from measurement space back to state space.)  
      Total Measurement Uncertainty: C*Σ⁻*Cᵀ + R  
         C*Σ⁻*Cᵀ: Uncertainty from prediction propagated to measurement space  
         R: Direct measurement noise uncertainty  
---
NOTE:
1. Numerator: Σ⁻*Cᵀ

   Σ⁻: "How uncertain am I about my prediction?" 
   Cᵀ: "How does state uncertainty affect measurements?"  
   Together: "How much should prediction uncertainty influence the measurement update?"  

2. Denominator: C*Σ⁻*Cᵀ + R  

   C*Σ⁻*Cᵀ: "How much uncertainty does my prediction add to the measurement?"  
   R: "How much noise does my sensor add?"  
   Together: "Total uncertainty in measurement space" 

3. K small (≈ 0): High confidence in prediction → Barely adjust  
   K large (≈ 1): High confidence in measurement → Adjust significantly

   ---

   Now lwts look at the correction/update 






