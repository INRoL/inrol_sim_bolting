#pragma once

#include <string>

namespace inrol_sim {
  /**
   * initializes the simulation environment with the provided data root directory.
   */
  void initialize(std::string asset_root_directory = "./");

  /**
   * runs a single step of the simulation.
   * @param[in] pd desired position of the nut CoM.
   * @param[in] Rd desired orientation of the end effector of the robot manipulator.
   * @param[in] Rd_gripper desired orientation of the gripper.
   * @param[inout] xk_sim position of the nut CoM.
   * @param[inout] qk_sim orientation (quaternion x, y, z, w) of the nut. 
   * @param[inout] ak_sim joint angle of the robot manipulator.
   * @param[out] Fc_sim output contact force.
   */
  void sim_function(double pd[3], double Rd[3][3], double Rd_gripper[3][3],
                    double xk_sim[3], double qk_sim[4],
                    double ak_sim[8], double Fc_sim[6]);

  /**
   * initializes openGL visualization with the provided asset root directory.
   */
  void vis_initialize(std::string asset_root_directory);

  /**
   * runs a single step of the visualization.
   */
	void vis_function(double xk_sim[3], double qk_sim[4],
					double ak_sim[8]);
}
