/*
 *  Copyright (c) 2021, Ryuichi Ueda
 *
 *  All rights reserved.
 *  Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
 *  So this software is provided under the terms of the GNU Lesser General Public License (LGPL).
 */

#ifndef PF_H__
#define PF_H__

#include <vector>
#include <sstream>
#include <random>

#include "emcl/Particle.h"
#include "emcl/OdomModel.h"
#include "emcl/LikelihoodFieldMap.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

namespace emcl {

class ParticleFilter
{
public: 
	ParticleFilter(const Pose &p, int num, const Scan &scan,
			const std::shared_ptr<OdomModel> &odom_model,
			const std::shared_ptr<LikelihoodFieldMap> &map,
			double alpha_th, double open_space_th,
			double expansion_radius_position, double expansion_radius_orientation,
			bool invert_lidar,
			double dev_threshold, double kld_threshold);
	~ParticleFilter();

	std::vector<Particle> particles_;
	double alpha_;

	void sensorUpdate(double lidar_x, double lidar_y, double lidar_t,
					  double x_var, double y_var, double t_var,
					  double gnss_x, double gnss_y, double gnss_var_x, double gnss_var_y);
	void motionUpdate(double x, double y, double t);

	void initialize(double x, double y, double t);

	void setScan(const sensor_msgs::LaserScan::ConstPtr &msg);
	void meanPose(double &x_mean, double &y_mean, double &t_mean,
			double &x_var, double &y_var, double &t_var,
			double &xy_cov, double &yt_cov, double &tx_cov);

	void simpleReset(void);

	void getVariance(double x_mean, double y_mean, double t_mean,
					 double &x_var, double &y_var, double &t_var);

	static double cos_[(1<<16)];
	static double sin_[(1<<16)];
private:
	Pose *last_odom_;
	Pose *prev_odom_;

	Scan scan_;
	int processed_seq_;

	double normalizeAngle(double t);
	void resampling(void);
	double normalize(void);
	void resetWeight(void);

	std::shared_ptr<OdomModel> odom_model_;
	std::shared_ptr<LikelihoodFieldMap> map_;

	double alpha_threshold_;
	double open_space_threshold_;
	double dev_threshold_;
	double kld_threshold_;
	double expansion_radius_position_;
	double expansion_radius_orientation_;

	bool invert_lidar_;

	void expansionReset(void);
	void gnssReset(double gnss_x, double gnss_y, double gnss_var_x, double gnss_var_y);
	double getKLDivergence(double x, double y, double x_var, double y_var, 
						   double gnss_x, double gnss_y, double gnss_var_x, double gnss_var_y);
};

double ParticleFilter::cos_[(1<<16)];
double ParticleFilter::sin_[(1<<16)];

}

#endif
