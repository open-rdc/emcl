//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/ExpResetMcl.h"
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>

namespace emcl {

ExpResetMcl::ExpResetMcl(const Pose &p, int num, const Scan &scan,
				const std::shared_ptr<OdomModel> &odom_model,
				const std::shared_ptr<LikelihoodFieldMap> &map,
				double alpha_th, double open_space_th,
				double expansion_radius_position, double expansion_radius_orientation)
	: alpha_threshold_(alpha_th), open_space_threshold_(open_space_th),
	  expansion_radius_position_(expansion_radius_position),
	  expansion_radius_orientation_(expansion_radius_orientation), Mcl::Mcl(p, num, scan, odom_model, map),
	  g_x(0), g_y(0), g_angle(0), reset(false)
{
}

ExpResetMcl::~ExpResetMcl()
{
}

void ExpResetMcl::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv)
{
	if(processed_seq_ == scan_.seq_)
		return;

	Scan scan;
	int seq = -1;
	while(seq != scan_.seq_){//trying to copy the latest scan before next 
		seq = scan_.seq_;
		scan = scan_;
	}

	scan.lidar_pose_x_ = lidar_x;
	scan.lidar_pose_y_ = lidar_y;
	scan.lidar_pose_yaw_ = lidar_t;

	int i = 0;
	if (!inv) {
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_min_ + (i++)*scan.angle_increment_)
			);
	} else {
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_max_ - (i++)*scan.angle_increment_)
			);
	}

	double valid_pct = 0.0;
	int valid_beams = scan.countValidBeams(&valid_pct);
	if(valid_beams == 0)
		return;

	{
		double l_x = -1.0, l_y = 0.0;
		double dx = l_x - g_x, dy = l_y - g_y;
		double len = sqrt(dx * dx + dy * dy);
		double ang = atan2(dy, dx) - g_angle;
		while (ang >  M_PI) ang -= 2.0 * M_PI;
		while (ang < -M_PI) ang += 2.0 * M_PI;
		sensorReset(l_x, l_y, len, ang);
	}

	for(auto &p : particles_)
		p.w_ *= p.likelihood(map_.get(), scan);

	alpha_ = normalizeBelief()/valid_beams;
	//alpha_ = nonPenetrationRate( particles_.size() / 20, map_.get(), scan); //new version
	ROS_INFO("ALPHA: %f / %f", alpha_, alpha_threshold_);
	if(alpha_ < alpha_threshold_ and valid_pct > open_space_threshold_){
		ROS_INFO("RESET");
		expansionReset();
		reset = true;
		for(auto &p : particles_)
			p.w_ *= p.likelihood(map_.get(), scan);
	} else {
		reset = false;
	}

	if(normalizeBelief() > 0.000001)
		resampling();
	else
		resetWeight();

	processed_seq_ = scan_.seq_;
}

void ExpResetMcl::expansionReset(void)
{
	for(auto &p : particles_){
		double length = 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_position_;
		double direction = 2*((double)rand()/RAND_MAX - 0.5)*M_PI;

		p.p_.x_ += length*cos(direction);
		p.p_.y_ += length*sin(direction);
		p.p_.t_ += 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_orientation_;
		p.w_ = 1.0/particles_.size();
	}
}

void ExpResetMcl::sensorReset(double lx, double ly, double length, double angle_rad)
{
	for(auto &p : particles_){
		if (((double)rand()/RAND_MAX) < 0.2 && reset)
		{
			p.p_.x_ += 2*((double)rand()/RAND_MAX - 0.5)*30.0;
			p.p_.y_ += 2*((double)rand()/RAND_MAX - 0.5)*30.0;
		}
		double dx = lx - p.p_.x_, dy = ly - p.p_.y_;
		double l = sqrt(dx * dx + dy * dy);
		//double angle = atan2(dy, dx) - p.p_.t_;
		double angle = atan2(dy, dx);
		while (angle >  M_PI) angle -= 2.0 * M_PI;
		while (angle < -M_PI) angle += 2.0 * M_PI;
		double d_angle = angle - angle_rad;
		while (d_angle >  M_PI) d_angle -= 2.0 * M_PI;
		while (d_angle < -M_PI) d_angle += 2.0 * M_PI;
		//double d_angle = (angle - (p.p_.t_ + angle_rad));
		p.p_.x_ -= (length-l)*cos(angle) * 0.1;
		p.p_.y_ -= (length-l)*sin(angle) * 0.1;
		p.p_.t_ += (d_angle - p.p_.t_) * 0.1;
		while (p.p_.t_ >  M_PI) p.p_.t_ -= 2.0 * M_PI;
		while (p.p_.t_ < -M_PI) p.p_.t_ += 2.0 * M_PI;
		//printf("angle_rad: %lf, angle: %lf, p.p_.t_: %lf\r\n", angle_rad, angle, p.p_.t_);
	}
}

void ExpResetMcl::setGroundTruth(double x, double y, double angle_rad)
{
	g_x = x;
	g_y = y;
	g_angle = angle_rad;
	//printf("g_x: %lf, g_y: %lf, g_angle: %lf\r\n", g_x, g_y, g_angle);
}

}
