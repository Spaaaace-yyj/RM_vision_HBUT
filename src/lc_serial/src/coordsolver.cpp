/**
 * @file coordsolver.cpp
 * @brief 
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2023-04-09
 * 
 */

#include "lc_serial/coordsolver.h"


/**
 * @brief Construct a new Coord Solver:: Coord Solver object
 * @param max_iter 迭代法求解pitch补偿的最大迭代次数
 * @param stop_error 停止迭代的最小误差(单位m)
 * @param R_K_iter 龙格库塔法求解落点的迭代次数
 */
CoordSolver::CoordSolver(int max_iter = 10, float stop_error = 0.001, int R_K_iter = 50){
	this->max_iter = max_iter;
	this->stop_error = stop_error;
	this->R_K_iter = R_K_iter;
}

/**
 * @brief 计算Pitch角度
 * 
 * @param xyz 坐标
 * @return double Pitch角度
 */
double CoordSolver::calcPitch(Eigen::Vector3d &xyz)
{
    return (atan2(xyz[2], sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1])) * 180 / M_PI);
    // return (atan2(xyz[1], sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2])) * 180 / M_PI);
}

double CoordSolver::calcYaw(Eigen::Vector3d &xyz)
{
    return atan2(xyz[0], xyz[1]) * 180 / M_PI;
}

/**
 * @brief 计算目标Yaw,Pitch角度
 * @return Yaw与Pitch
*/
Eigen::Vector2d CoordSolver::calcYawPitch(Eigen::Vector3d &xyz)
{
    Eigen::Vector2d angle;
    //Yaw(逆时针)
    //Pitch(目标在上方为正)
    angle << calcYaw(xyz),calcPitch(xyz);
    return angle;
}

/**
 * @brief 计算Pitch轴偏移量
 * 
 * @param xyz 坐标
 * @return double Pitch偏移量
 */
double CoordSolver::dynamicCalcPitchOffset(Eigen::Vector3d xyz)
{
	//TODO:根据陀螺仪安装位置调整距离求解方式
	//降维，坐标系Y轴以垂直向上为正方向
	auto dist_vertical = xyz[2];
	auto vertical_tmp = dist_vertical;
	auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
	// auto dist_vertical = xyz[2];
	// auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
	auto pitch = atan(dist_vertical / dist_horizonal) * 180 / M_PI;
	auto pitch_new = pitch;
	//开始使用龙格库塔法求解弹道补偿
	for (int i = 0; i < max_iter; i++)
	{
		//TODO: 我们已经转到枪口坐标系下了，如果没有，可以选择迭代起点为枪口在当前坐标系下的坐标
		//初始化
		auto x = 0.0;
		auto y = 0.0;
		auto p = tan(pitch_new / 180 * M_PI);
		auto v = bullet_speed;
		auto u = v / sqrt(1 + pow(p,2));
		auto delta_x = dist_horizonal / R_K_iter;
		for (int j = 0; j < R_K_iter; j++)
		{
			auto k1_u = -k * u * sqrt(1 + pow(p, 2));
			auto k1_p = -g / pow(u, 2);
			auto k1_u_sum = u + k1_u * (delta_x / 2);
			auto k1_p_sum = p + k1_p * (delta_x / 2);

			auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
			auto k2_p = -g / pow(k1_u_sum, 2);
			auto k2_u_sum = u + k2_u * (delta_x / 2);
			auto k2_p_sum = p + k2_p * (delta_x / 2);

			auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
			auto k3_p = -g / pow(k2_u_sum, 2);
			auto k3_u_sum = u + k3_u * (delta_x / 2);
			auto k3_p_sum = p + k3_p * (delta_x / 2);

			auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
			auto k4_p = -g / pow(k3_u_sum, 2);
			
			u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
			p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);
			
			x+=delta_x;
			y+=p * delta_x;
		}
		//评估迭代结果,若小于迭代精度需求则停止迭代
		auto error = dist_vertical - y;
		if (fabs(error) <= stop_error)
		{
			break;
		}
		else 
		{
			vertical_tmp+=error;
			// xyz_tmp[1] -= error;
			pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / M_PI;
		}
	}
	return pitch_new - pitch;
}
