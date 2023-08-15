# Denoising

## Lidar Denoising

### SOR Filter

The SOR filter aims to remove the sparse outliers caused by measurement error. To do so, it iterates through each point and then computes the average distances di of k-nearest points to that point, where k represents an integer parameter of the filter that can be selected based on how many neighbor points are wanted to be analyzed. As another key variable, the threshold value T can be defined as following:

```
T = µ±β×σ
```

where µ and σ are the mean and standard deviation of the average distances di , and β is a constant multiplier.


### ROR Filter

The ROR filter removes isolated outliers from point clouds by iterating through each point and counting the number of points located within a sphere with a center of that point and search radius, R. It uses the k-d tree algorithm to search for a point inside a sphere. If the number of points is less than the minimum acceptable number of points N, it is removed as an outlier, otherwise it is saved as an inlier. The parameters N and R can be varied to find an optimum solution for ROR filtering.

### DROR Filter

The ROR and SOR filters were chosen for the first time to test their de-snowing abilities. This study found that the SOR was able to remove the majority of snow points, but failed to remove densely grouped snow points. Furthermore, although the ROR filter showed a better performance for de-snowing in general, it excluded all important information from the environment that was farther away than 18 m from a LiDAR sensor. This is because LiDAR points clouds become sparser as the distance from the sensor increases, while a search radius in the ROR filter remains constant.

```
R_dynamic = φ × α × sqrt(x2 + y2)
```

Where φ is a constant multiplier, α is the angular resolution of the LiDAR sensor

### LIOR Filter


### LIDROR Filter

