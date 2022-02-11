^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense_hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2022-02-11)
------------------
* Merge pull request `#21 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/21>`_ from OUXT-Polaris/fix/depends_diagnostic_msgs
  add diagnostic_msgs to the depends
* add diagnostic_msgs to the depends
* [Bot] Update workflow (`#20 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/20>`_)
  * Setup workflow
  * remove old repos
  * remove quaternion_operation package
  Co-authored-by: MasayaKataoka <ms.kataoka@gmail.com>
* Setup workflow (`#19 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/19>`_)
* Setup workflow (`#18 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/18>`_)
* Setup workflow (`#17 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/17>`_)
* Setup workflow (`#16 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/16>`_)
* Setup workflow (`#15 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/15>`_)
* Merge pull request `#14 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/14>`_ from OUXT-Polaris/fix/galactic_build
  enable pass compile in galactic environment
* enable pass compile in galactic environment
* Merge pull request `#13 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/13>`_ from OUXT-Polaris/fix/update_mathod
  enable pass compile in galactic
* enable pass compile in galactic
* Setup workflow (`#12 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/12>`_)
* Merge pull request `#11 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/11>`_ from OUXT-Polaris/fix/ros2_control_branch
  update target branch
* fix errors in galactic
* Merge branch 'master' into fix/ros2_control_branch
* update target branch
* Setup workflow (`#10 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/10>`_)
* Setup workflow (`#9 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/9>`_)
* Setup workflow (`#8 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/8>`_)
* Setup workflow (`#7 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/7>`_)
* [Bot] Update workflow (`#6 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/6>`_)
  * Setup build test workflow
  * specify ros2_control version
  * update workflow
  Co-authored-by: Masaya Kataoka <ms.kataoka@gmail.com>
* Merge pull request `#5 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/5>`_ from OUXT-Polaris/feature/remove_old_workflow
  remove old workflow
* remove old workflow
* Merge pull request `#4 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/4>`_ from OUXT-Polaris/workflow/build_test
  [Bot] Update workflow
* add ros2_control to the repos file
* add ouxt_common to the repos
* add ouxt_lint_common
* Setup build test workflow
* Merge pull request `#3 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/3>`_ from OUXT-Polaris/feature/imu_driver
* enable publish imu data
* modify launch files
* add imu publisher class
* enable get value in hardware interface
* add isready function
* add toMsg function
* add imu handle
* Merge pull request `#2 <https://github.com/OUXT-Polaris/realsense_hardware_interface/issues/2>`_ from OUXT-Polaris/feature/add_depends
  update depends
* update depends
* Contributors: Masaya Kataoka, MasayaKataoka, wam-v-tan

0.0.1 (2021-07-21)
------------------
* add depends
* enable pass test
* enable print device infomation
* fix error
* enable specify serial
* apply reformat
* enable encode as color image
* fix error
* fix errors
* add qos option
* rename rviz package
* update rviz
* enable get image
* enable subscribe image
* fix size
* add util.cpp
* apply reformat
* enable transport image as shared memory
* add poco key param
* add frameToSet function
* update depends
* add poco to the depends
* enable get w value
* apply reformat
* fix problems in getting angular velocity
* add rviz file
* enable publish odom
* enable get odom message
* add depends
* apply reformat
* add angular acceleration interface
* enable passing rs2_pose
* enable load controller plugin
* add rs2_pose_publisher
* add getValue memnber function
* remove reference from member
* add handler to the interface
* enable export interface from handle struct
* add PoseHandle
* add data handle
* remove proto files
* add proto definition
* add toMsg functions
* modify cmake
* remove unused process
* use librealsense
* add some configuration
* add URDF
* apply reformat
* add .gitignore
* enable recieve values
* initial commit
* Contributors: Masaya Kataoka
